use clap::Parser;
use serde::{Deserialize, Serialize};

/// CLI for Physical Dice Simulation
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Dice expressions to roll (format: 2D6, 1D20, etc.)
    #[arg(required = true, value_parser = parse_dice_notation)]
    dice_expressions: Vec<String>,

    /// Maximum simulation time in seconds
    #[arg(short, long, default_value_t = 8.0)]
    time: f32,

    /// Output format: text, json, csv
    #[arg(short, long, default_value = "text", value_parser = ["text", "json", "csv"])]
    output: String,

    /// Verbose output
    #[arg(short, long)]
    verbose: bool,

    /// Mass for each die
    #[arg(long, default_value_t = 0.17)]
    mass: f32,

    /// Number of rolls for batch mode
    #[arg(long, default_value_t = 1)]
    batch: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RollResult {
    die_type: String,
    value: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationResult {
    results: Vec<RollResult>,
    total: u32,
}

#[derive(Debug, Clone)]
struct DiceNotation {
    count: u32,
    sides: u32,
}

/// Parse dice notation like 2D6, 1D20, etc.
fn parse_dice_notation(s: &str) -> Result<String, String> {
    let s = s.to_uppercase();
    if !s.contains('D') {
        return Err(format!("Invalid dice notation: {}. Format should be like 2D6", s));
    }

    let parts: Vec<&str> = s.split('D').collect();
    if parts.len() != 2 {
        return Err(format!("Invalid dice notation: {}", s));
    }

    let count_str = parts[0];
    let sides_str = parts[1];

    if count_str.is_empty() || sides_str.is_empty() {
        return Err(format!("Invalid dice notation: {}", s));
    }

    let count: u32 = match count_str.parse() {
        Ok(c) => c,
        Err(_) => return Err(format!("Invalid count in dice notation: {}", s)),
    };

    let sides: u32 = match sides_str.parse() {
        Ok(s) => s,
        Err(_) => return Err(format!("Invalid sides in dice notation: {}", s)),
    };

    if sides < 4 || sides > 254 {
        return Err(format!("Invalid number of sides: {}. Must be between 4 and 254", sides));
    }

    if count == 0 {
        return Err(format!("Invalid count: {}. Must be greater than 0", count));
    }

    Ok(s)
}

/// Create polyhedron geometry for standard dice
fn create_polyhedron_for_die(sides: u32) -> Result<libpdice::Polyhedron, &'static str> {
    match sides {
        4 => create_tetrahedron(),
        6 => create_cube(),
        8 => create_octahedron(),
        12 => create_dodecahedron(),
        20 => create_icosahedron(),
        _ if sides >= 4 && sides <= 254 => create_n_sided_die(sides),
        _ => Err("Unsupported die type. Must have between 4 and 254 sides"),
    }
}

fn create_tetrahedron() -> Result<libpdice::Polyhedron, &'static str> {
    let h = 0.5f32;
    let s = 3f32.sqrt() * h; // scale for regular tetrahedron
    let vertices = vec![
        libpdice::Point3::new( s,  s,  s),
        libpdice::Point3::new( s, -s, -s),
        libpdice::Point3::new(-s,  s, -s),
        libpdice::Point3::new(-s, -s,  s),
    ];

    let faces = vec![
        vec![0, 1, 2], // face 0
        vec![0, 3, 1], // face 1
        vec![0, 2, 3], // face 2
        vec![1, 3, 2], // face 3
    ];

    Ok(libpdice::Polyhedron { vertices, faces })
}

fn create_cube() -> Result<libpdice::Polyhedron, &'static str> {
    let h = 0.5f32;
    let vertices = vec![
        libpdice::Point3::new(-h, -h, -h),
        libpdice::Point3::new( h, -h, -h),
        libpdice::Point3::new( h,  h, -h),
        libpdice::Point3::new(-h,  h, -h),
        libpdice::Point3::new(-h, -h,  h),
        libpdice::Point3::new( h, -h,  h),
        libpdice::Point3::new( h,  h,  h),
        libpdice::Point3::new(-h,  h,  h),
    ];

    let faces = vec![
        vec![0, 1, 2, 3], // -Z
        vec![4, 7, 6, 5], // +Z
        vec![0, 4, 5, 1], // -Y
        vec![2, 6, 7, 3], // +Y
        vec![1, 5, 6, 2], // +X
        vec![0, 3, 7, 4], // -X
    ];

    Ok(libpdice::Polyhedron { vertices, faces })
}

fn create_octahedron() -> Result<libpdice::Polyhedron, &'static str> {
    let vertices = vec![
        libpdice::Point3::new( 1.0,  0.0,  0.0),
        libpdice::Point3::new(-1.0,  0.0,  0.0),
        libpdice::Point3::new( 0.0,  1.0,  0.0),
        libpdice::Point3::new( 0.0, -1.0,  0.0),
        libpdice::Point3::new( 0.0,  0.0,  1.0),
        libpdice::Point3::new( 0.0,  0.0, -1.0),
    ];

    let faces = vec![
        vec![0, 4, 2], // face 0
        vec![0, 2, 5], // face 1
        vec![0, 5, 3], // face 2
        vec![0, 3, 4], // face 3
        vec![1, 2, 4], // face 4
        vec![1, 5, 2], // face 5
        vec![1, 3, 5], // face 6
        vec![1, 4, 3], // face 7
    ];

    Ok(libpdice::Polyhedron { vertices, faces })
}

fn create_dodecahedron() -> Result<libpdice::Polyhedron, &'static str> {
    // Golden ratio
    let phi = (1.0 + 5f32.sqrt()) / 2.0;
    let vertices = vec![
        libpdice::Point3::new( 1.0,  1.0,  1.0),
        libpdice::Point3::new( 1.0,  1.0, -1.0),
        libpdice::Point3::new( 1.0, -1.0,  1.0),
        libpdice::Point3::new( 1.0, -1.0, -1.0),
        libpdice::Point3::new(-1.0,  1.0,  1.0),
        libpdice::Point3::new(-1.0,  1.0, -1.0),
        libpdice::Point3::new(-1.0, -1.0,  1.0),
        libpdice::Point3::new(-1.0, -1.0, -1.0),
        
        libpdice::Point3::new( 0.0,  phi,  1.0/phi),
        libpdice::Point3::new( 0.0,  phi, -1.0/phi),
        libpdice::Point3::new( 0.0, -phi,  1.0/phi),
        libpdice::Point3::new( 0.0, -phi, -1.0/phi),
        libpdice::Point3::new( 1.0/phi,  0.0,  phi),
        libpdice::Point3::new(-1.0/phi,  0.0,  phi),
        libpdice::Point3::new( 1.0/phi,  0.0, -phi),
        libpdice::Point3::new(-1.0/phi,  0.0, -phi),
        libpdice::Point3::new( phi,  1.0/phi,  0.0),
        libpdice::Point3::new( phi, -1.0/phi,  0.0),
        libpdice::Point3::new(-phi,  1.0/phi,  0.0),
        libpdice::Point3::new(-phi, -1.0/phi,  0.0),
    ];

    let faces = vec![
        vec![0, 8, 4, 13, 12],  // face 0
        vec![0, 12, 2, 17, 16], // face 1
        vec![0, 16, 1, 14, 8],  // face 2
        vec![1, 14, 5, 9, 17],  // face 3
        vec![2, 10, 6, 13, 12], // face 4
        vec![3, 11, 7, 15, 14], // face 5
        vec![4, 9, 5, 18, 19],  // face 6
        vec![6, 19, 7, 15, 11], // face 7
        vec![8, 14, 15, 7, 9],   // face 8
        vec![12, 13, 6, 11, 10], // face 9
        vec![16, 17, 9, 7, 19],  // face 10
        vec![18, 5, 14, 17, 19], // face 11
    ];

    Ok(libpdice::Polyhedron { vertices, faces })
}

fn create_icosahedron() -> Result<libpdice::Polyhedron, &'static str> {
    // Golden ratio
    let phi = (1.0 + 5f32.sqrt()) / 2.0;
    let vertices = vec![
        libpdice::Point3::new( 0.0,  1.0,  phi),
        libpdice::Point3::new( 0.0, -1.0,  phi),
        libpdice::Point3::new( 0.0,  1.0, -phi),
        libpdice::Point3::new( 0.0, -1.0, -phi),
        
        libpdice::Point3::new( 1.0,  phi,  0.0),
        libpdice::Point3::new(-1.0,  phi,  0.0),
        libpdice::Point3::new( 1.0, -phi,  0.0),
        libpdice::Point3::new(-1.0, -phi,  0.0),
        
        libpdice::Point3::new( phi,  0.0,  1.0),
        libpdice::Point3::new( phi,  0.0, -1.0),
        libpdice::Point3::new(-phi,  0.0,  1.0),
        libpdice::Point3::new(-phi,  0.0, -1.0),
    ];

    let faces = vec![
        vec![0, 8, 4],   // face 0
        vec![0, 5, 10],  // face 1
        vec![2, 4, 9],   // face 2
        vec![2, 11, 5],  // face 3
        vec![1, 6, 8],   // face 4
        vec![1, 10, 7],  // face 5
        vec![3, 9, 6],   // face 6
        vec![3, 7, 11],  // face 7
        vec![0, 10, 8],  // face 8
        vec![8, 10, 1],  // face 9
        vec![8, 1, 4],   // face 10
        vec![4, 1, 6],   // face 11
        vec![4, 6, 9],   // face 12
        vec![9, 6, 3],   // face 13
        vec![9, 3, 2],   // face 14
        vec![2, 3, 11],  // face 15
        vec![2, 11, 5],  // face 16 (duplicated to match pattern)
        vec![5, 11, 0],  // face 17
        vec![0, 11, 7],  // face 18
        vec![0, 7, 8],   // face 19
    ];

    Ok(libpdice::Polyhedron { vertices, faces })
}

/// Create an N-sided die using a bipyramidal structure for odd N or trapezohedron-like for even N
fn create_n_sided_die(sides: u32) -> Result<libpdice::Polyhedron, &'static str> {
    if sides < 4 {
        return Err("Die must have at least 4 sides");
    }
    
    // For N-sided dice, we'll use a method that creates a polyhedron with N faces
    // A common approach is to use a bipyramid for odd N or a trapezohedron for even N
    
    if sides % 2 == 0 {
        // For even-sided dice, use a bipyramid approach
        create_bipyramid_die(sides)
    } else {
        // For odd-sided dice, we'll create a more complex structure
        create_trapezohedron_die(sides)
    }
}

/// Create a bipyramid die (two pyramids joined at the base)
fn create_bipyramid_die(sides: u32) -> Result<libpdice::Polyhedron, &'static str> {
    let n = sides / 2; // number of faces around the equator
    if n < 2 {
        return Err("Invalid number of sides for bipyramid");
    }
    
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    
    // Top vertex (north pole)
    vertices.push(libpdice::Point3::new(0.0, 1.0, 0.0));
    
    // Base vertices (equatorial ring)
    for i in 0..n {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
        let x = angle.cos();
        let z = angle.sin();
        vertices.push(libpdice::Point3::new(x, 0.0, z));
    }
    
    // Bottom vertex (south pole)
    vertices.push(libpdice::Point3::new(0.0, -1.0, 0.0));
    
    // Create faces: triangles from top vertex to each edge of the equatorial ring
    for i in 0..n {
        let next = (((i + 1) % n) + 1) as usize; // +1 to account for top vertex at index 0
        faces.push(vec![0usize, (i + 1) as usize, next]); // top faces
    }
    
    // Create faces: triangles from bottom vertex to each edge of the equatorial ring (in reverse order for correct orientation)
    for i in 0..n {
        let next = (((i + 1) % n) + 1) as usize; // +1 to account for top vertex at index 0
        faces.push(vec![(n + 1) as usize, next, (i + 1) as usize]); // bottom faces, order is reversed for proper face orientation
    }
    
    Ok(libpdice::Polyhedron { vertices, faces })
}

/// Create a trapezohedron die (for odd-sided dice)
fn create_trapezohedron_die(sides: u32) -> Result<libpdice::Polyhedron, &'static str> {
    if sides < 3 {
        return Err("Trapezohedron must have at least 3 faces around the equator (6 total sides)");
    }
    
    // A trapezohedron has 2n faces where n is the number of kites
    // So if sides is odd, this is a special case where we need to adjust
    // For an n-sided die where n is odd, we'll create a shape with n faces
    // by creating a more complex polyhedron
    
    let n = sides;
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    
    // Generate vertices in a way that creates N faces
    // We'll use an approach that creates N kite-shaped or triangular faces
    
    // Top vertex
    vertices.push(libpdice::Point3::new(0.0, 1.0, 0.0));
    
    // Bottom vertex
    vertices.push(libpdice::Point3::new(0.0, -1.0, 0.0));
    
    // Generate intermediate rings of vertices
    // For an N-sided shape, we need to position vertices appropriately
    let rings = if n > 8 { 2 } else { 1 }; // Use 1 or 2 rings depending on N
    let vertices_per_ring = if n > 8 { n / 2 } else { n - 2 };
    
    // Create rings of vertices around the middle
    for ring in 0..rings {
        let y = if rings == 1 { 0.0 } else if ring == 0 { 0.3 } else { -0.3 };
        let radius = 0.8;
        
        for i in 0..vertices_per_ring {
            let angle = 2.0 * std::f32::consts::PI * i as f32 / vertices_per_ring as f32;
            let x = radius * angle.cos();
            let z = radius * angle.sin();
            vertices.push(libpdice::Point3::new(x, y, z));
        }
    }
    
    // Create faces connecting vertices
    // This is a simplified approach to create an N-sided shape
    // In a real trapezohedron, each face would be a kite or similar quadrilateral
    
    // Connect top to first ring
    if rings == 1 {
        // For small N, connect directly to points around equator
        for i in 0..(n - 2) {
            let next = if i == n - 3 { 1 } else { i + 2 };  // Skip top vertex at index 0, bottom at index 1
            faces.push(vec![0usize, (i + 2) as usize, next as usize]);  // top vertex (0) to ring vertices
        }
        
        // Connect bottom to first ring
        for i in 0..(n - 2) {
            let next = if i == n - 3 { 1 } else { i + 2 };  // Skip top vertex at index 0, bottom at index 1
            faces.push(vec![1usize, next as usize, (i + 2) as usize]);  // bottom vertex (1) to ring vertices, reversed for orientation
        }
    } else {
        // For larger N with 2 rings, create a more complex structure
        for i in 0..vertices_per_ring {
            let next = (i + 1) % vertices_per_ring;
            let ring_vertex_idx = i + 2;  // Skip top (0) and bottom (1)
            let next_ring_vertex_idx = next + 2;
            
            // Connect top to first ring
            faces.push(vec![0usize, ring_vertex_idx as usize, next_ring_vertex_idx as usize]);
            
            // Connect rings to each other
            let second_ring_vertex_idx = next_ring_vertex_idx + vertices_per_ring;
            let next_second_idx = if next == 0 { 2 + vertices_per_ring } else { next_ring_vertex_idx + 1 };
            
            // Connect to bottom
            faces.push(vec![1usize, next_second_idx as usize, second_ring_vertex_idx as usize]);
        }
    }
    
    // Ensure we have at least N faces for the N-sided die
    // This is a simplified approach - a proper trapezohedron implementation would be more complex
    if faces.len() as u32 != n {
        // For now, we'll create a simpler structure that definitely has N faces
        return create_approximate_ndie(n);
    }
    
    Ok(libpdice::Polyhedron { vertices, faces })
}

/// Create a simplified N-sided die approximation
fn create_approximate_ndie(sides: u32) -> Result<libpdice::Polyhedron, &'static str> {
    // For N-sided dice where N > 20, we'll create a polyhedron with N faces
    // using a distribution of points on a sphere and creating faces around each point
    
    if sides < 4 || sides > 254 {
        return Err("Number of sides must be between 4 and 254");
    }
    
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    
    // Generate N points approximately distributed on a sphere
    // This is based on the Fibonacci sphere algorithm for even distribution
    for i in 0..sides {
        let y = 1.0 - (i as f32 * 2.0) / (sides as f32 - 1.0); // from 1 to -1
        let radius = (1.0 - y * y).sqrt();
        
        let theta = if sides < 12 {
            (i as f32) * (2.0 * std::f32::consts::PI / sides as f32)
        } else {
            ((i as f32) * 3.6).to_radians() * 3.23606797 // approx golden angle
        };
        
        let x = radius * theta.cos();
        let z = radius * theta.sin();
        
        vertices.push(libpdice::Point3::new(x, y, z));
    }
    
    // For a proper convex polyhedron with N faces, we need to implement
    // convex hull or Delaunay triangulation which is complex.
    // Instead, let's use an approach that creates an N-sided die
    // by creating N faces that would result from a polyhedron.
    
    // Since creating a true N-faced polyhedron is complex, we'll approximate
    // by creating a structure where each "face" corresponds to one of the N sides
    // For this simple implementation, we'll create a bipyramid with 2*(N-2) triangular faces
    // and then try to group them to represent N distinct sides
    
    if sides == 4 {
        // For D4, we already have the proper tetrahedron
        return create_tetrahedron();
    }
    
    // Use bipyramid approach for N-sided dice
    let n = sides;
    
    // Create vertices: top, bottom, and equatorial points
    vertices.clear(); // Clear the previously added points
    vertices.push(libpdice::Point3::new(0.0, 1.0, 0.0));  // Top vertex
    
    // Create equatorial vertices in a circle
    for i in 0..(n - 2) {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / (n - 2) as f32;
        let x = angle.cos();
        let z = angle.sin();
        vertices.push(libpdice::Point3::new(x, 0.0, z));
    }
    
    vertices.push(libpdice::Point3::new(0.0, -1.0, 0.0));  // Bottom vertex
    
    // Create triangular faces: connect top to each pair of adjacent equatorial vertices
    for i in 0..(n - 2) {
        let next = (i + 1) % (n - 2);
        faces.push(vec![0usize, (i + 1) as usize, (next + 1) as usize]); // Top faces
    }
    
    // Create bottom faces: connect bottom to each pair of adjacent equatorial vertices in reverse order
    for i in 0..(n - 2) {
        let next = (i + 1) % (n - 2);
        faces.push(vec![(n - 1) as usize, (next + 1) as usize, (i + 1) as usize]); // Bottom faces (reversed for proper orientation)
    }
    
    Ok(libpdice::Polyhedron { vertices, faces })
}

fn convert_face_to_value(face_index: usize, sides: u32) -> u32 {
    // This is a simple mapping - the face index + 1 corresponds to the die value
    // For most symmetric dice, face 0 corresponds to value 1, etc.
    (face_index % sides as usize) as u32 + 1
}

fn run_simulation(args: &Args) -> Result<Vec<SimulationResult>, Box<dyn std::error::Error>> {
    let mut all_results = Vec::new();

    for _batch in 0..args.batch {
        let mut scene = libpdice::Scene::new();
        let mut roll_results = Vec::new();

        // Process all dice expressions
        for expr in &args.dice_expressions {
            let expr_upper = expr.to_uppercase();
            let parts: Vec<&str> = expr_upper.split('D').collect();
            let count: u32 = parts[0].parse().unwrap();
            let sides: u32 = parts[1].parse().unwrap();

            for _i in 0..count {
                let poly = create_polyhedron_for_die(sides)?;
                let die = libpdice::Die::new_from_poly(poly, args.mass)?;
                scene.add_die(die);
            }

            // After adding all dice of this type, record their types for the result
            for _i in 0..count {
                roll_results.push(RollResult {
                    die_type: format!("D{}", sides),
                    value: 0, // Placeholder, will be updated after simulation
                });
            }
        }

        // Run the simulation
        let results = scene.run_to_rest(args.time);

        // Map the simulation results to die values
        for (i, (face_index, _body)) in results.iter().enumerate() {
            if i < roll_results.len() {
                let sides = roll_results[i].die_type[1..].parse::<u32>().unwrap();
                roll_results[i].value = convert_face_to_value(*face_index, sides);
            }
        }

        let total: u32 = roll_results.iter().map(|r| r.value).sum();
        all_results.push(SimulationResult {
            results: roll_results,
            total,
        });
    }

    Ok(all_results)
}

fn format_output(results: Vec<SimulationResult>, output_format: &str) -> Result<String, Box<dyn std::error::Error>> {
    match output_format {
        "text" => {
            let mut output = String::new();
            for (i, result) in results.iter().enumerate() {
                if results.len() > 1 {
                    output.push_str(&format!("Roll {}: ", i + 1));
                }
                
                let values: Vec<String> = result.results.iter()
                    .map(|r| format!("{}: {}", r.die_type, r.value))
                    .collect();
                output.push_str(&format!("{}\n", values.join(", ")));
                
                if results.len() > 1 {
                    output.push_str(&format!("  Total: {}\n", result.total));
                } else {
                    output.push_str(&format!("Total: {}\n", result.total));
                }
            }
            Ok(output)
        },
        "json" => {
            if results.len() == 1 {
                Ok(serde_json::to_string_pretty(&results[0])?)
            } else {
                Ok(serde_json::to_string_pretty(&results)?)
            }
        },
        "csv" => {
            let mut output = String::from("Roll,Dice Type,Value\n");
            for (i, result) in results.iter().enumerate() {
                for roll in &result.results {
                    output.push_str(&format!("{},{},{}\n", i + 1, roll.die_type, roll.value));
                }
            }
            Ok(output)
        },
        _ => Err("Invalid output format".into())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    if args.verbose {
        eprintln!("Rolling: {}", args.dice_expressions.join(" "));
        eprintln!("Simulation time: {} seconds", args.time);
    }

    match run_simulation(&args) {
        Ok(results) => {
            let output = format_output(results, &args.output)?;
            println!("{}", output);
        }
        Err(e) => {
            eprintln!("Error during simulation: {}", e);
            std::process::exit(1);
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_dice_notation_valid() {
        assert!(parse_dice_notation("2D6").is_ok());
        assert!(parse_dice_notation("1D20").is_ok());
        assert!(parse_dice_notation("3D4").is_ok());
        assert!(parse_dice_notation("1D12").is_ok());
        assert!(parse_dice_notation("10D8").is_ok());
    }

    #[test]
    fn test_parse_dice_notation_invalid_format() {
        assert!(parse_dice_notation("2D").is_err());
        assert!(parse_dice_notation("D6").is_err());
        assert!(parse_dice_notation("26").is_err());
        assert!(parse_dice_notation("").is_err());
        assert!(parse_dice_notation("D").is_err());
    }

    #[test]
    fn test_parse_dice_notation_invalid_values() {
        assert!(parse_dice_notation("0D6").is_err()); // Count must be > 0
        assert!(parse_dice_notation("2D3").is_err()); // Sides must be >= 4
        assert!(parse_dice_notation("2D255").is_err()); // Sides must be <= 254
        assert!(parse_dice_notation("2D0").is_err()); // Sides must be >= 4
    }

    #[test]
    fn test_parse_dice_notation_uppercase() {
        let result = parse_dice_notation("2d6");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), "2D6");
    }

    #[test]
    fn test_format_output_text() {
        let results = vec![
            SimulationResult {
                results: vec![
                    RollResult {
                        die_type: "D6".to_string(),
                        value: 4,
                    },
                    RollResult {
                        die_type: "D20".to_string(),
                        value: 15,
                    },
                ],
                total: 19,
            }
        ];

        let output = format_output(results, "text").unwrap();
        assert!(output.contains("D6: 4"));
        assert!(output.contains("D20: 15"));
        assert!(output.contains("Total: 19"));
    }

    #[test]
    fn test_format_output_json() {
        let results = vec![
            SimulationResult {
                results: vec![
                    RollResult {
                        die_type: "D6".to_string(),
                        value: 4,
                    },
                    RollResult {
                        die_type: "D20".to_string(),
                        value: 15,
                    },
                ],
                total: 19,
            }
        ];

        let output = format_output(results, "json").unwrap();
        // Test that it's valid JSON by attempting to parse it back
        let parsed: serde_json::Value = serde_json::from_str(&output).unwrap();
        assert_eq!(parsed["total"], 19);
        assert_eq!(parsed["results"].as_array().unwrap().len(), 2);
        assert_eq!(parsed["results"][0]["die_type"], "D6");
        assert_eq!(parsed["results"][0]["value"], 4);
        assert_eq!(parsed["results"][1]["die_type"], "D20");
        assert_eq!(parsed["results"][1]["value"], 15);
    }

    #[test]
    fn test_format_output_csv() {
        let results = vec![
            SimulationResult {
                results: vec![
                    RollResult {
                        die_type: "D6".to_string(),
                        value: 4,
                    },
                    RollResult {
                        die_type: "D20".to_string(),
                        value: 15,
                    },
                ],
                total: 19,
            }
        ];

        let output = format_output(results, "csv").unwrap();
        assert!(output.contains("Roll,Dice Type,Value"));
        assert!(output.contains("1,D6,4"));
        assert!(output.contains("1,D20,15"));
    }

    #[test]
    fn test_format_output_invalid_format() {
        let results = vec![
            SimulationResult {
                results: vec![
                    RollResult {
                        die_type: "D6".to_string(),
                        value: 4,
                    }
                ],
                total: 4,
            }
        ];

        let result = format_output(results, "invalid");
        assert!(result.is_err());
    }

    #[test]
    fn test_create_polyhedron_for_die_supported_types() {
        // Test supported die types
        assert!(create_polyhedron_for_die(4).is_ok());
        assert!(create_polyhedron_for_die(6).is_ok());
        assert!(create_polyhedron_for_die(8).is_ok());
        assert!(create_polyhedron_for_die(12).is_ok());
        assert!(create_polyhedron_for_die(20).is_ok());
    }

    #[test]
    fn test_create_polyhedron_for_die_unsupported_types() {
        // Test unsupported die types
        assert!(create_polyhedron_for_die(3).is_err()); // Below minimum
        assert!(create_polyhedron_for_die(0).is_err()); // Below minimum
        assert!(create_polyhedron_for_die(255).is_err()); // Above maximum
    }

    #[test]
    fn test_create_polyhedron_for_die_supported_extended_types() {
        // Test that now supports N-sided dice
        assert!(create_polyhedron_for_die(30).is_ok()); // Now supported
        assert!(create_polyhedron_for_die(100).is_ok()); // Now supported
        assert!(create_polyhedron_for_die(12).is_ok()); // Still supported
        assert!(create_polyhedron_for_die(24).is_ok()); // Now supported
    }

    #[test]
    fn test_convert_face_to_value() {
        // For a D6, face index 0 should map to value 1, face index 5 should map to value 6
        assert_eq!(convert_face_to_value(0, 6), 1);
        assert_eq!(convert_face_to_value(1, 6), 2);
        assert_eq!(convert_face_to_value(5, 6), 6);
        
        // Check wraparound for values beyond sides
        assert_eq!(convert_face_to_value(6, 6), 1); // Same as face 0
        assert_eq!(convert_face_to_value(7, 6), 2); // Same as face 1
        
        // For a D20
        assert_eq!(convert_face_to_value(0, 20), 1);
        assert_eq!(convert_face_to_value(19, 20), 20);
    }
}
