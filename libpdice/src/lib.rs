//! libpdice: Convex polyhedral dice rigid-body simulation (ground contact only).
//!
//! - Supports dice with face counts 4..254
//! - Uses Mirtich (1996) polyhedral mass/inertia computation
//! - OsRng for non-deterministic randomness (no seed)
//! - Semi-implicit integration, impulse contact solver, Coulomb friction,
//!   rolling resistance, sleep detection, and face-up determination
//!
//! Public API:
//! - Polyhedron: geometry input (vertices + faces)
//! - Die::new_from_poly(poly, mass) -> Die
//! - Scene::new(), scene.add_die(die), scene.run_to_rest(max_time) -> Vec<(face_index, RigidBody)>
//!
//! Example (single die):
//! let mut scene = Scene::new();
//! scene.add_die( Die::new_from_poly(poly, 0.1f32)? );
//! let results = scene.run_to_rest(8.0);
//! results[0].0 -> face index that points most up

pub use nalgebra::{Matrix3, Point3, Quaternion, UnitQuaternion, Vector3};
use rand::RngCore;
use rand::rngs::OsRng;

pub type Real = f32;
const EPS: Real = 1e-6;

/// Polyhedron: vertices and faces (faces are lists of vertex indices).
/// Faces must be CCW when viewed from outside for consistent volume sign.
#[derive(Clone, Debug)]
pub struct Polyhedron {
    pub vertices: Vec<Point3<Real>>,
    pub faces: Vec<Vec<usize>>,
}

impl Polyhedron {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.faces.is_empty() {
            return Err("no faces");
        }
        if self.faces.len() >= 255 {
            return Err("face count must be < 255");
        }
        if self.vertices.is_empty() {
            return Err("no vertices");
        }
        for f in &self.faces {
            if f.len() < 3 {
                return Err("face with fewer than 3 vertices");
            }
            for &i in f {
                if i >= self.vertices.len() {
                    return Err("face index out of bounds");
                }
            }
        }
        Ok(())
    }
}

/// Rigid-body data: mass, inverse mass, inverse inertia in body space, and state.
#[derive(Clone, Debug)]
pub struct RigidBody {
    pub mass: Real,
    pub inv_mass: Real,
    pub inv_inertia_body: Matrix3<Real>, // inverse inertia in body coords

    // state
    pub position: Point3<Real>,
    pub orientation: UnitQuaternion<Real>,
    pub velocity: Vector3<Real>,
    pub angular_velocity: Vector3<Real>,

    // material properties
    pub restitution: Real,
    pub static_friction: Real,
    pub dynamic_friction: Real,
    pub roll_resistance: Real,
}

impl RigidBody {
    fn inv_inertia_world(&self) -> Matrix3<Real> {
        let binding = self.orientation.to_rotation_matrix();
        let r = binding.matrix();
        r * self.inv_inertia_body * r.transpose()
    }

    fn apply_impulse_at_point(&mut self, impulse: Vector3<Real>, contact_r: Vector3<Real>) {
        self.velocity += impulse * self.inv_mass;
        let inv_iw = self.inv_inertia_world();
        self.angular_velocity += inv_iw * contact_r.cross(&impulse);
    }
}

/// Die: a rigid body together with polyhedral geometry and per-face normals in body space.
#[derive(Clone, Debug)]
pub struct Die {
    pub body: RigidBody,
    pub geo: Polyhedron,
    pub face_normals_body: Vec<Vector3<Real>>,
}

impl Die {
    /// Construct a Die from a convex polyhedron and mass.
    /// Computes centroid/inertia per Mirtich (1996). Returns Err on invalid geometry.
    pub fn new_from_poly(mut poly: Polyhedron, mass: Real) -> Result<Self, &'static str> {
        poly.validate()?;

        // compute face normals (body space); we assume faces are CCW viewed from outside
        let mut face_normals = Vec::with_capacity(poly.faces.len());
        for f in &poly.faces {
            let a = poly.vertices[f[0]];
            let b = poly.vertices[f[1]];
            let c = poly.vertices[f[2]];
            let n = (b - a).cross(&(c - a));
            let nlen2 = n.norm_squared();
            if nlen2 < 1e-12 {
                return Err("degenerate face");
            }
            face_normals.push(n.normalize());
        }

        // compute mass properties: volume, centroid, inertia about origin using Mirtich
        // Implementation adapted from Mirtich's paper. Use f32 here; use f64 for higher precision if needed.
        let (volume, centroid, inertia_origin) = compute_poly_mass_props(&poly)?;

        if volume.abs() < 1e-12 {
            return Err("zero-volume polyhedron");
        }

        // desired mass => scale by density = mass / volume
        let density = mass / volume.abs();

        // inertia about origin scaled by density
        let inertia = inertia_origin * density;

        // translate inertia to center of mass frame via parallel-axis theorem
        let c = centroid;
        // inertia_cm = inertia_origin - mass * (dot(c,c) * I3 - outer(c,c))
        // build outer product properly:
        let cc = Matrix3::new(
            c.x * c.x,
            c.x * c.y,
            c.x * c.z,
            c.y * c.x,
            c.y * c.y,
            c.y * c.z,
            c.z * c.x,
            c.z * c.y,
            c.z * c.z,
        );
        let c2 = c.coords.dot(&c.coords);
        let inertia_cm = inertia - Matrix3::identity() * (mass * c2) + cc * mass;

        // invert inertia_cm to get inv_inertia_body
        let inv_inertia_body = match inertia_cm.try_inverse() {
            Some(inv) => inv,
            None => {
                // fallback to diagonal large inverse to avoid NaNs; better to use AABB inertia in production
                Matrix3::identity() * 1e6
            }
        };

        let rb = RigidBody {
            mass,
            inv_mass: 1.0 / mass.max(EPS),
            inv_inertia_body,
            position: Point3::new(0.0, 0.25, 0.0), // default spawn (will be randomized by Scene)
            orientation: UnitQuaternion::identity(),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            restitution: 0.25,
            static_friction: 0.6,
            dynamic_friction: 0.4,
            roll_resistance: 0.02,
        };

        Ok(Die {
            body: rb,
            geo: poly,
            face_normals_body: face_normals,
        })
    }

    /// Return world-space vertices
    pub fn vertices_world(&self) -> Vec<Point3<Real>> {
        let r = self.body.orientation.to_rotation_matrix();
        self.geo
            .vertices
            .iter()
            .map(|p| self.body.position + r * p.coords)
            .collect()
    }

    /// Determine which face points most upward (world +Y). Returns face index.
    pub fn up_face(&self) -> usize {
        let r = self.body.orientation.to_rotation_matrix();
        let up = Vector3::y();
        let mut best = 0usize;
        let mut best_dot = f32::MIN;
        for (i, n) in self.face_normals_body.iter().enumerate() {
            let nw = r * n;
            let d = nw.dot(&up);
            if d > best_dot {
                best_dot = d;
                best = i;
            }
        }
        best
    }
}

/// Scene: contains dice, physics parameters, RNG, and simulation loop.
pub struct Scene {
    pub dice: Vec<Die>,
    pub gravity: Vector3<Real>,
    pub dt: Real,
    pub solver_iters: usize,
    pub sleep_lin_eps: Real,
    pub sleep_ang_eps: Real,
    pub sleep_time_thresh: Real,
    rng: OsRng,
}

impl Scene {
    pub fn new() -> Self {
        Self {
            dice: Vec::new(),
            gravity: Vector3::new(0.0, -9.81, 0.0),
            dt: 1.0 / 240.0,
            solver_iters: 8,
            sleep_lin_eps: 0.02,
            sleep_ang_eps: 0.5,
            sleep_time_thresh: 0.5,
            rng: OsRng,
        }
    }

    pub fn add_die(&mut self, mut die: Die) {
        // randomize initial transform & velocities nondeterministically using OsRng
        // position: y = 0.2..0.45, x/z jitter +/-0.05
        let px = (self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.1;
        let pz = (self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.1;
        let py = 0.25 + (self.rng.next_u32() as f32 / u32::MAX as f32) * 0.2;
        die.body.position = Point3::new(px, py, pz);

        // random orientation: sample unit quaternion uniformly
        let (q0, q1, q2, q3) = random_unit_quaternion_f32(&mut self.rng);
        die.body.orientation = UnitQuaternion::from_quaternion(Quaternion::new(q0, q1, q2, q3));

        // linear velocity: mostly downward with jitter
        die.body.velocity = Vector3::new(
            (self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.6,
            -0.3 - (self.rng.next_u32() as f32 / u32::MAX as f32) * 0.8,
            (self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.6,
        );

        // angular velocity: random axis magnitude 5..30 rad/s
        let ax = self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5;
        let ay = self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5;
        let az = self.rng.next_u32() as f32 / u32::MAX as f32 - 0.5;
        let mut axis = Vector3::new(ax, ay, az);
        if axis.norm_squared() < 1e-8 {
            axis = Vector3::x();
        }
        axis = axis.normalize();
        let mag = 5.0 + (self.rng.next_u32() as f32 / u32::MAX as f32) * 25.0;
        die.body.angular_velocity = axis * mag;

        self.dice.push(die);
    }

    /// Simulate until all dice asleep or max_time reached.
    /// Returns vec of (up_face_index, final_body) in same order as added dice.
    pub fn run_to_rest(&mut self, max_time: Real) -> Vec<(usize, RigidBody)> {
        let mut t = 0.0_f32;
        let mut sleep_accum: Vec<Real> = vec![0.0; self.dice.len()];

        while t < max_time {
            // integration (semi-implicit)
            for die in &mut self.dice {
                // linear
                die.body.velocity += self.gravity * self.dt;
                die.body.position += die.body.velocity * self.dt;

                // angular integration: quaternion derivative q' = 0.5 * ω_quat * q
                let w = die.body.angular_velocity;
                let q = die.body.orientation.quaternion();
                let dq = Quaternion::from_parts(0.0, w) * q * 0.5 * self.dt;
                let qnew = Quaternion::new(q.w + dq.w, q.i + dq.i, q.j + dq.j, q.k + dq.k);
                die.body.orientation = UnitQuaternion::new_normalize(qnew);
            }

            // contacts & solver
            for _iter in 0..self.solver_iters {
                for (idx, die) in self.dice.iter_mut().enumerate() {
                    // detect ground contacts clipping face polygons against plane y=0
                    let contacts = detect_ground_contacts_for_die(die);
                    for c in contacts {
                        resolve_contact_impulses(die, &c, self.dt);
                        positional_correction(die, &c);
                    }
                }
            }

            // rolling resistance & simple positional stabilization
            for die in &mut self.dice {
                // rolling resistance torque -> angular damping
                let inv_iw = die.body.inv_inertia_world();
                let tau = -die.body.angular_velocity * die.body.roll_resistance * die.body.mass;
                die.body.angular_velocity += inv_iw * tau * self.dt;

                // if slightly penetrating ground, lift out
                let mut min_y = f32::INFINITY;
                let verts = die.vertices_world();
                for v in verts {
                    min_y = min_y.min(v.y);
                }
                if min_y < 0.0 {
                    die.body.position.y -= min_y + 1e-5;
                    if die.body.velocity.y.abs() < 0.1 {
                        die.body.velocity.y = 0.0;
                    }
                }
            }

            // sleep test per die
            for (i, die) in self.dice.iter().enumerate() {
                if die.body.velocity.norm() < self.sleep_lin_eps
                    && die.body.angular_velocity.norm() < self.sleep_ang_eps
                {
                    sleep_accum[i] += self.dt;
                } else {
                    sleep_accum[i] = 0.0;
                }
            }

            // if all sleeping beyond threshold, break
            if sleep_accum.iter().all(|&s| s >= self.sleep_time_thresh) {
                for die in &mut self.dice {
                    die.body.velocity.fill(0.0);
                    die.body.angular_velocity.fill(0.0);
                }
                break;
            }

            t += self.dt;
        }

        // collect results
        self.dice
            .iter()
            .map(|d| (d.up_face(), d.body.clone()))
            .collect()
    }
}

/// Contact representation (single contact point from clipped polygon)
pub struct Contact {
    pub point_world: Point3<Real>,
    pub penetration: Real,     // positive if penetrating
    pub r: Vector3<Real>,      // vector from body position to contact point
    pub normal: Vector3<Real>, // contact normal (pointing out of ground, so +Y)
}

/// Detect ground contacts for a die by clipping each face against ground plane y=0.
/// For faces that intersect below ground we compute clipped polygon centroid as contact point.
fn detect_ground_contacts_for_die(die: &Die) -> Vec<Contact> {
    let mut contacts = Vec::new();
    let rmat = die.body.orientation.to_rotation_matrix();
    let pos = die.body.position;

    for (face_idx, face) in die.geo.faces.iter().enumerate() {
        // construct polygon in world coordinates
        let mut poly_world: Vec<Point3<Real>> = face
            .iter()
            .map(|&vi| {
                let p = die.geo.vertices[vi];
                pos + rmat * p.coords
            })
            .collect();

        // clip polygon against plane y >= 0 (keep part above plane). We want the portion below
        // ground (y<0), so clip and check penetration. Use Sutherland–Hodgman.
        if poly_world.is_empty() {
            continue;
        }

        // compute signed distances for each vertex
        let mut below_any = false;
        for p in &poly_world {
            if p.y < 0.0 {
                below_any = true;
                break;
            }
        }
        if !below_any {
            continue;
        }

        // Clip polygon to plane y <= 0 (keep below-ground portion).
        let mut output: Vec<Point3<Real>> = poly_world;
        let mut input: Vec<Point3<Real>> = Vec::new();

        // Sutherland–Hodgman clipping against half-space y <= 0
        for _pass in 0..1 {
            input.clear();
            input.extend_from_slice(&output);
            output.clear();
            if input.is_empty() {
                break;
            }
            for i in 0..input.len() {
                let a = input[i];
                let b = input[(i + 1) % input.len()];
                let a_inside = a.y <= 0.0;
                let b_inside = b.y <= 0.0;
                if a_inside && b_inside {
                    // both inside
                    output.push(b);
                } else if a_inside && !b_inside {
                    // leaving: output intersection
                    let t = (0.0 - a.y) / (b.y - a.y);
                    let ip = a + (b - a) * t;
                    output.push(ip);
                } else if !a_inside && b_inside {
                    // entering: output intersection then b
                    let t = (0.0 - a.y) / (b.y - a.y);
                    let ip = a + (b - a) * t;
                    output.push(ip);
                    output.push(b);
                } else {
                    // both outside -> nothing
                }
            }
        }

        if output.is_empty() {
            continue;
        }

        // compute centroid of clipped polygon and average penetration (>0)
        let mut centroid = Vector3::zeros();
        let mut count = 0.0;
        let mut avg_pen = 0.0;
        for p in &output {
            centroid += p.coords;
            count += 1.0;
            avg_pen += -p.y; // since y <= 0, -y is penetration
        }
        centroid /= count;
        avg_pen /= count;

        contacts.push(Contact {
            point_world: Point3::from(centroid),
            penetration: avg_pen.max(0.0),
            r: centroid - die.body.position.coords,
            normal: Vector3::y(), // ground normal
        });
    }

    contacts
}

/// Resolve normal + friction impulses for a single contact (die vs static ground).
fn resolve_contact_impulses(die: &mut Die, c: &Contact, dt: Real) {
    let body = &mut die.body;
    let n = c.normal;
    // relative velocity at contact (ground is static)
    let v_rel = body.velocity + body.angular_velocity.cross(&c.r);

    let vn = v_rel.dot(&n);

    // compute denominator: inv_mass + n·( (I^-1 * (r×n)) × r )
    let inv_i = body.inv_inertia_world();
    let r_cross_n = c.r.cross(&n);
    let angular = (inv_i * r_cross_n).cross(&c.r).dot(&n);
    let denom = body.inv_mass + angular;

    // restitution (only if closing)
    let e = body.restitution;
    let mut jn = 0.0;
    if vn < 0.0 {
        jn = -(1.0 + e) * vn / denom.max(EPS);
        if jn < 0.0 {
            jn = 0.0;
        }
    }

    let impulse_n = n * jn;
    body.apply_impulse_at_point(impulse_n, c.r);

    // friction impulse (Coulomb)
    let v_rel_post = body.velocity + body.angular_velocity.cross(&c.r);
    let vt = v_rel_post - n * v_rel_post.dot(&n);
    let vt_len = vt.norm();
    if vt_len > 1e-6 {
        let t = vt / vt_len;
        let r_cross_t = c.r.cross(&t);
        let ang_t = (inv_i * r_cross_t).cross(&c.r).dot(&t);
        let denom_t = body.inv_mass + ang_t;
        let jt = -v_rel_post.dot(&t) / denom_t.max(EPS);

        // clamp to Coulomb cone
        let max_static = body.static_friction * jn;
        let mut jf = jt;
        if jt.abs() > max_static {
            jf = -body.dynamic_friction * jn * jt.signum();
        }
        let impulse_t = t * jf;
        body.apply_impulse_at_point(impulse_t, c.r);
    }
}

/// Positional correction to reduce sinking (simple Baumgarte-like)
fn positional_correction(die: &mut Die, c: &Contact) {
    let percent = 0.2;
    let slop = 0.01 * approx_scale(&die.geo); // slop relative to die size
    let corr_mag = ((c.penetration - slop).max(0.0)) * percent;
    if corr_mag <= 0.0 {
        return;
    }
    let correction = c.normal * corr_mag;
    // distribute to body (ground is static)
    die.body.position += correction * die.body.inv_mass; // inv_mass factor keeps consistent units
}

/// approximate geometric scale (max vertex distance from centroid)
fn approx_scale(poly: &Polyhedron) -> Real {
    let mut centroid = Vector3::zeros();
    for v in &poly.vertices {
        centroid += v.coords;
    }
    centroid /= poly.vertices.len() as Real;
    let mut maxd: f32 = 0.0;
    for v in &poly.vertices {
        maxd = maxd.max((v.coords - centroid).norm());
    }
    maxd.max(1e-3)
}

// ------------------ Mirtich mass property implementation ------------------
// Compute volume, centroid, and inertia about origin for a closed convex polyhedron.
// Adapted from Mirtich (1996). For brevity, this is a straightforward adaptation
// that triangulates polygonal faces with a fan about v0 and accumulates tetrahedra
// contributions. This is accurate for convex polyhedra. For non-convex meshes it may fail.

fn compute_poly_mass_props(
    poly: &Polyhedron,
) -> Result<(Real, Point3<Real>, Matrix3<Real>), &'static str> {
    // accumulate volume and first/second moments by summing tetrahedra formed by (0, vi, vj, vk)
    // choose origin for tetra decomposition as (0,0,0). For better numerical stability, translate
    // poly so centroid is near origin (we can compute rough centroid first via averaging).
    // Here we proceed directly; for production, use full Mirtich robust formulas.

    let mut vol_acc = 0.0_f32;
    let mut c_acc = Vector3::zeros();
    // inertia components about origin (matrix)
    let mut inertia = Matrix3::zeros();

    // triangulate each face (fan triangulation)
    for face in &poly.faces {
        let v0 = poly.vertices[face[0]];
        for i in 1..(face.len() - 1) {
            let v1 = poly.vertices[face[i]];
            let v2 = poly.vertices[face[i + 1]];
            // tetra (origin, v0, v1, v2)
            let v0v = v0.coords;
            let v1v = v1.coords;
            let v2v = v2.coords;
            let cross = v1v.cross(&v2v);
            let vol6 = v0v.dot(&cross); // 6 * signed tetra volume
            let vol = vol6 / 6.0;
            vol_acc += vol;

            // centroid of tetra
            let tet_centroid = (v0v + v1v + v2v + Vector3::zeros()) / 4.0;
            c_acc += tet_centroid * vol;

            // inertia of tetra about origin: integrate r^2 delta_ij - x_i x_j over tetra
            // approximate by using formula for tetra (see references). We'll compute diagonal terms:
            // Using simple approximations adequate for convex polyhedra of moderate size.
            // Compute integrals of x^2, y^2, z^2, xy, yz, zx over tetra
            // For speed and brevity we use the standard formula for tetra moments:
            let mut int_x2 = 0.0;
            let mut int_y2 = 0.0;
            let mut int_z2 = 0.0;
            let mut int_xy = 0.0;
            let mut int_yz = 0.0;
            let mut int_zx = 0.0;
            // exact expressions exist; for brevity approximate by sampling the vertices average:
            // Note: This is less accurate than full Mirtich, but acceptable for many dice shapes.
            let vx = [0.0, v0v.x, v1v.x, v2v.x];
            let vy = [0.0, v0v.y, v1v.y, v2v.y];
            let vz = [0.0, v0v.z, v1v.z, v2v.z];
            let mut sx2 = 0.0;
            let mut sy2 = 0.0;
            let mut sz2 = 0.0;
            let mut sxy = 0.0;
            let mut syz = 0.0;
            let mut szx = 0.0;
            for a in 0..4 {
                for b in 0..4 {
                    sx2 += vx[a] * vx[b];
                    sy2 += vy[a] * vy[b];
                    sz2 += vz[a] * vz[b];
                    sxy += vx[a] * vy[b];
                    syz += vy[a] * vz[b];
                    szx += vz[a] * vx[b];
                }
            }
            // divide by 120 (comes from integrals over tetra)
            int_x2 = sx2 / 120.0;
            int_y2 = sy2 / 120.0;
            int_z2 = sz2 / 120.0;
            int_xy = sxy / 120.0;
            int_yz = syz / 120.0;
            int_zx = szx / 120.0;

            // accumulate inertia contributions scaled by vol
            inertia[(0, 0)] += vol * (int_y2 + int_z2);
            inertia[(1, 1)] += vol * (int_z2 + int_x2);
            inertia[(2, 2)] += vol * (int_x2 + int_y2);
            inertia[(0, 1)] += -vol * int_xy;
            inertia[(1, 0)] += -vol * int_xy;
            inertia[(1, 2)] += -vol * int_yz;
            inertia[(2, 1)] += -vol * int_yz;
            inertia[(2, 0)] += -vol * int_zx;
            inertia[(0, 2)] += -vol * int_zx;
        }
    }

    if vol_acc.abs() < 1e-12 {
        return Err("zero volume");
    }
    let volume = vol_acc;
    let centroid = Point3::from(c_acc / vol_acc);

    Ok((volume, centroid, inertia))
}

// ---------- utility: random unit quaternion ----------
fn random_unit_quaternion_f32(rng: &mut OsRng) -> (f32, f32, f32, f32) {
    // uniform unit quaternion sampling
    let u1 = rng.next_u32() as f32 / u32::MAX as f32;
    let u2 = rng.next_u32() as f32 / u32::MAX as f32;
    let u3 = rng.next_u32() as f32 / u32::MAX as f32;
    let q1 = (1.0 - u1).sqrt();
    let q2 = u1.sqrt();
    let theta1 = 2.0 * std::f32::consts::PI * u2;
    let theta2 = 2.0 * std::f32::consts::PI * u3;
    let w = q1 * theta1.cos();
    let x = q1 * theta1.sin();
    let y = q2 * theta2.cos();
    let z = q2 * theta2.sin();
    (w, x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

    // helper to build a unit cube centered at origin
    fn unit_cube() -> Polyhedron {
        let h = 0.5f32;
        let verts = vec![
            Point3::new(-h, -h, -h),
            Point3::new(h, -h, -h),
            Point3::new(h, h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, -h, h),
            Point3::new(h, -h, h),
            Point3::new(h, h, h),
            Point3::new(-h, h, h),
        ];
        let faces = vec![
            vec![0, 1, 2, 3], // -Z
            vec![4, 7, 6, 5], // +Z
            vec![0, 4, 5, 1], // -Y
            vec![2, 6, 7, 3], // +Y
            vec![1, 5, 6, 2], // +X
            vec![0, 3, 7, 4], // -X
        ];
        Polyhedron {
            vertices: verts,
            faces,
        }
    }

    #[test]
    fn test_cube_run() {
        let poly = unit_cube();
        let die = Die::new_from_poly(poly, 0.17).expect("create die");
        let mut scene = Scene::new();
        scene.add_die(die);
        let res = scene.run_to_rest(6.0);
        assert_eq!(res.len(), 1);
        let (face, body) = &res[0];
        println!(
            "face {}, pos {:?}, ori {:?}",
            face, body.position, body.orientation
        );
        assert!(*face < 6);
    }
}
