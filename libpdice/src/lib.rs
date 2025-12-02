//! physical_sim: simple rigid-body dice roll simulation (single cube/d6).
//! Safe, non-deterministic randomness via OS RNG (no seed).

use na::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use nalgebra as na;
use rand::RngCore;
use rand::rngs::OsRng;

pub type Real = f32;
const EPS: Real = 1e-6;

#[derive(Clone, Copy, Debug)]
pub struct RigidBody {
    pub mass: Real,
    inv_mass: Real,
    // inertia in body-space (diagonal for cube), store inverse
    inv_inertia_body: Matrix3<Real>,

    // state
    pub position: Vector3<Real>,
    pub orientation: UnitQuaternion<Real>,
    pub velocity: Vector3<Real>,
    pub angular_velocity: Vector3<Real>,

    // material
    pub restitution: Real,
    pub static_friction: Real,
    pub dynamic_friction: Real,
    pub roll_resistance: Real,
}

impl RigidBody {
    pub fn new_cube(mass: Real, half_extent: Real) -> Self {
        let m = mass.max(EPS);
        let inv_mass = 1.0 / m;

        // inertia tensor for cube (axis-aligned, body space)
        // I = (1/12) m * (h^2 + d^2) for each diagonal term; for cube all same
        let h2 = (2.0 * half_extent) * (2.0 * half_extent);
        let i = (1.0 / 12.0) * m * (h2 + h2); // (for axis x: y^2 + z^2), cube -> same
        let inv_i = Matrix3::identity() * (1.0 / i);

        Self {
            mass: m,
            inv_mass,
            inv_inertia_body: inv_i,
            position: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            restitution: 0.25,
            static_friction: 0.6,
            dynamic_friction: 0.4,
            roll_resistance: 0.02,
        }
    }

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

#[derive(Debug)]
pub struct Die {
    pub body: RigidBody,
    // geometry = cube with half extent
    pub half_extent: Real,
}

impl Die {
    pub fn new_cube(half_extent: Real, mass: Real) -> Self {
        Self {
            body: RigidBody::new_cube(mass, half_extent),
            half_extent,
        }
    }

    // return body-space vertices of cube
    fn vertices_body(&self) -> Vec<Vector3<Real>> {
        let h = self.half_extent;
        let mut v = Vec::with_capacity(8);
        for &sx in &[-1.0, 1.0] {
            for &sy in &[-1.0, 1.0] {
                for &sz in &[-1.0, 1.0] {
                    v.push(Vector3::new(sx * h, sy * h, sz * h));
                }
            }
        }
        v
    }

    // world-space vertex positions
    fn vertices_world(&self) -> Vec<Vector3<Real>> {
        let r = self.body.orientation.to_rotation_matrix();
        self.vertices_body()
            .into_iter()
            .map(|p| self.body.position + r * p)
            .collect()
    }

    // determine face index (0..5) pointing most upward (y is up)
    pub fn up_face(&self) -> usize {
        // faces: +Y (0), -Y (1), +X (2), -X (3), +Z (4), -Z (5)
        let rot = self.body.orientation.to_rotation_matrix();
        let axes = [
            rot * Vector3::y(),  // +Y
            rot * -Vector3::y(), // -Y
            rot * Vector3::x(),  // +X
            rot * -Vector3::x(), // -X
            rot * Vector3::z(),  // +Z
            rot * -Vector3::z(), // -Z
        ];
        let up = Vector3::y();
        let (idx, _) = axes
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.dot(&up).partial_cmp(&b.1.dot(&up)).unwrap())
            .unwrap();
        idx
    }
}

pub struct Scene {
    pub die: Die,
    pub gravity: Vector3<Real>,
    pub dt: Real,
    pub solver_iters: usize,
    // thresholds
    sleep_lin_eps: Real,
    sleep_ang_eps: Real,
    sleep_time_thresh: Real,
    rng: OsRng,
}

impl Scene {
    pub fn new_single_die() -> Self {
        let mut rng = OsRng;
        let mut die = Die::new_cube(0.0125, 0.17); // ~standard d6 size (half_extent ~12.5mm), mass ~170g
        // randomize initial state (non-deterministic)
        // position: 0.2..0.4m above ground, small xy jitter
        let px = (rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.04;
        let pz = (rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.04;
        let py = 0.25 + (rng.next_u32() as f32 / u32::MAX as f32) * 0.15;
        die.body.position = Vector3::new(px, py, pz);

        // random orientation: sample random unit quaternion
        let u1 = rng.next_u32() as f32 / u32::MAX as f32;
        let u2 = rng.next_u32() as f32 / u32::MAX as f32;
        let u3 = rng.next_u32() as f32 / u32::MAX as f32;
        let q = UnitQuaternion::from_quaternion(Quaternion::new(
            (1.0 - u1).sqrt() * (2.0 * std::f32::consts::PI * u2).sin(),
            (1.0 - u1).sqrt() * (2.0 * std::f32::consts::PI * u2).cos(),
            u1.sqrt() * (2.0 * std::f32::consts::PI * u3).sin(),
            u1.sqrt() * (2.0 * std::f32::consts::PI * u3).cos(),
        ));
        die.body.orientation = q;

        // linear velocity: mostly downward with jitter
        die.body.velocity = Vector3::new(
            (rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.6,
            -0.3 - (rng.next_u32() as f32 / u32::MAX as f32) * 0.8,
            (rng.next_u32() as f32 / u32::MAX as f32 - 0.5) * 0.6,
        );

        // angular velocity: random axis, magnitude 5..30 rad/s
        let ax = rng.next_u32() as f32 / u32::MAX as f32 - 0.5;
        let ay = rng.next_u32() as f32 / u32::MAX as f32 - 0.5;
        let az = rng.next_u32() as f32 / u32::MAX as f32 - 0.5;
        let mut axis = Vector3::new(ax, ay, az);
        if axis.norm_squared() < EPS {
            axis = Vector3::x();
        }
        axis = axis.normalize();
        let mag = 5.0 + (rng.next_u32() as f32 / u32::MAX as f32) * 25.0;
        die.body.angular_velocity = axis * mag;

        let body = die.body.clone();
        let scene = Scene {
            die,
            gravity: Vector3::new(0.0, -9.81, 0.0),
            dt: 1.0 / 240.0,
            solver_iters: 8,
            sleep_lin_eps: 0.02,
            sleep_ang_eps: 0.5,
            sleep_time_thresh: 0.5,
            rng,
        };
        scene
    }

    // single timestep
    fn integrate(&mut self) {
        let dt = self.dt;
        // semi-implicit (symplectic) Euler for linear
        let body = &mut self.die.body;
        let acc = self.gravity; // mass-normalized applied force is gravity (ignoring drag)
        body.velocity += acc * dt;
        body.position += body.velocity * dt;

        // angular integration (simple): ω <- ω + I^-1 * τ * dt (we apply roll resistance torque below)
        body.orientation = UnitQuaternion::new_normalize(
            (Quaternion::from_parts(0.0, body.angular_velocity) * body.orientation.quaternion())
                * (0.5 * dt)
                + body.orientation.quaternion(),
        );
    }

    // ground contact detection (plane y=0), returns vector of contact points (world pos, penetration, r vector)
    fn detect_contacts_ground(&self) -> Vec<(Vector3<Real>, Real, Vector3<Real>)> {
        let mut contacts = Vec::new();
        let verts = self.die.vertices_world();
        for v in verts {
            let penetration = 0.0 - v.y; // ground at y=0; positive penetration means below ground
            if penetration > 1e-5 {
                let r = v - self.die.body.position;
                contacts.push((v, penetration, r));
            }
        }
        contacts
    }

    // resolve contacts iteratively
    fn resolve_contacts(&mut self) {
        // iterate solver
        for _ in 0..self.solver_iters {
            let contacts = self.detect_contacts_ground();
            if contacts.is_empty() {
                break;
            }
            for (p_world, penetration, r) in contacts.iter() {
                // normal is up
                let n = Vector3::y();
                let body = &mut self.die.body;

                // relative velocity at contact: v + ω x r - ground(0)
                let v_rel = body.velocity + body.angular_velocity.cross(r);

                let vn = v_rel.dot(&n);
                // if separating and not penetrating, skip
                if vn > 0.0 && *penetration <= 0.0 {
                    continue;
                }

                // compute impulse scalar j for normal
                let inv_mass_sum = body.inv_mass;
                let inv_i = body.inv_inertia_world();
                let r_cross_n = r.cross(&n);
                let angular = (inv_i * r_cross_n).cross(r).dot(&n);
                let denom = inv_mass_sum + angular;
                let e = body.restitution;
                let jn = if denom > 0.0 {
                    -(1.0 + e) * vn / denom
                } else {
                    0.0
                };

                let jn_clamped = jn.max(0.0);
                let impulse_n = n * jn_clamped;

                body.apply_impulse_at_point(impulse_n, *r);

                // friction (Coulomb)
                let v_rel_post = body.velocity + body.angular_velocity.cross(r);
                let vt = v_rel_post - n * v_rel_post.dot(&n);
                let vt_len = vt.norm();
                if vt_len > 1e-6 {
                    let t = vt / vt_len;
                    let jt = -vt_len / denom.max(EPS);
                    // clamp
                    let max_static = body.static_friction * jn_clamped;
                    let mut jf = jt;
                    if jt.abs() > max_static {
                        // kinetic
                        jf = -body.dynamic_friction * jn_clamped * jt.signum();
                    }
                    let impulse_t = t * jf;
                    body.apply_impulse_at_point(impulse_t, *r);
                }

                // positional correction (baumgarte-like)
                let slop = 0.01 * body.half_extent(); // small slop relative to size
                let percent = 0.2;
                let corr = n * ((penetration - slop).max(0.0) * percent / (body.inv_mass));
                body.position += corr * body.inv_mass; // push out (simple)
            }
        }
    }

    // apply rolling resistance torque
    fn apply_rolling_resistance(&mut self) {
        let body = &mut self.die.body;
        let tau = -body.angular_velocity * body.roll_resistance * body.mass;
        let inv_i = body.inv_inertia_world();
        // convert torque to angular velocity change: Δω = I^-1 τ * dt
        body.angular_velocity += inv_i * tau * self.dt;
    }

    // sleep detection, returns true if sleeping
    fn check_sleep(&self, slept_time: Real) -> bool {
        let b = &self.die.body;
        b.velocity.norm() < self.sleep_lin_eps
            && b.angular_velocity.norm() < self.sleep_ang_eps
            && slept_time >= self.sleep_time_thresh
    }

    // run full simulation until all sleeping or max_time reached. returns final up face index and final body state.
    pub fn run_to_rest(&mut self, max_time: Real) -> (usize, RigidBody) {
        let mut t = 0.0_f32;
        let mut slept_accum = 0.0_f32;
        while t < max_time {
            self.integrate();
            // collision resolution
            self.resolve_contacts();
            // rolling resistance
            self.apply_rolling_resistance();

            // positional stabilization: if we are slightly penetrating ground, correct
            {
                let mut min_y = f32::INFINITY;
                for v in self.die.vertices_world() {
                    min_y = min_y.min(v.y);
                }
                if min_y < 0.0 {
                    self.die.body.position.y -= min_y + 1e-4;
                    // damp tiny velocities
                    if self.die.body.velocity.y.abs() < 0.1 {
                        self.die.body.velocity.y = 0.0;
                    }
                }
            }

            // aging for sleep
            if self.die.body.velocity.norm() < self.sleep_lin_eps
                && self.die.body.angular_velocity.norm() < self.sleep_ang_eps
            {
                slept_accum += self.dt;
            } else {
                slept_accum = 0.0;
            }

            if self.check_sleep(slept_accum) {
                // zero velocities
                self.die.body.velocity.fill(0.0);
                self.die.body.angular_velocity.fill(0.0);
                break;
            }

            t += self.dt;
        }

        let face = self.die.up_face();
        (face, self.die.body.clone())
    }
}

// helper for RigidBody half extent access (not part of RigidBody normally)
trait BodyExt {
    fn half_extent(&self) -> Real;
}
impl BodyExt for RigidBody {
    fn half_extent(&self) -> Real {
        0.0125
    } // default used above; only for positional correction slop
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_die_to_rest() {
        let mut scene = Scene::new_single_die();
        let (face, body) = scene.run_to_rest(8.0);
        println!(
            "rest face: {}, pos: {:?}, ori: {:?}",
            face, body.position, body.orientation
        );
        assert!(face < 6);
    }
}
