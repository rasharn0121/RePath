use repath::{RePathfinder, RePathSettingsBuilder, save_metrics_to_csv};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    // Configure pathfinding settings
    let settings = RePathSettingsBuilder::new("navmesh_varied.obj")
        .precompute_radius(100.0)
        .build();

    let pathfinder = RePathfinder::new(settings)?;

    // Define start and end coordinates (within navmesh_varied.obj bounds)
    let start_coords = (100.0, 0.0, 100.0);
    let end_coords = (200.0, 0.0, 200.0);

    // Find a path
    match pathfinder.find_path(start_coords, end_coords)? {
        Some(path) => {
            println!("✓ Found path with {} waypoints", path.len());

            // Print first few waypoints
            for (i, node) in path.iter().take(5).enumerate() {
                println!("  Waypoint {}: ({:.2}, {:.2}, {:.2})",
                    i, node.x, node.y, node.z);
            }
            if path.len() > 5 {
                println!("  ... and {} more waypoints", path.len() - 5);
            }
        }
        None => {
            println!("✗ No path found between the coordinates");
        }
    }

    // Print statistics
    let stats = pathfinder.stats().snapshot();
    println!("\n{}", stats);

    // Export statistics to CSV using utils function
    save_metrics_to_csv("pathfinding_stats.csv", &stats)?;
    println!("\n✓ Statistics exported to pathfinding_stats.csv");

    Ok(())
}
