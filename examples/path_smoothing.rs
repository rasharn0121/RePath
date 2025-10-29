use repath::{RePathfinder, RePathSettingsBuilder};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    println!("=== Path Smoothing Demonstration ===\n");

    // Test 1: Without smoothing
    println!("Test 1: Pathfinding WITHOUT smoothing");
    let settings_no_smooth = RePathSettingsBuilder::new("navmesh_varied.obj")
        .enable_path_smoothing(false)
        .build();

    let pathfinder = RePathfinder::new(settings_no_smooth)?;

    let start = (500.0, 0.0, 500.0);
    let end = (2500.0, 0.0, 2500.0);

    if let Some(path) = pathfinder.find_path(start, end)? {
        println!("  Path found with {} waypoints (RAW)", path.len());

        // Show first few waypoints
        for (i, node) in path.iter().take(3).enumerate() {
            println!("    Waypoint {}: ({:.1}, {:.1}, {:.1})", i, node.x, node.y, node.z);
        }
        if path.len() > 3 {
            println!("    ... {} more waypoints ...", path.len() - 3);
        }
    }

    // Test 2: With moderate smoothing
    println!("\nTest 2: Pathfinding WITH moderate smoothing (10°)");
    let settings_moderate = RePathSettingsBuilder::new("navmesh_varied.obj")
        .enable_path_smoothing(true)
        .smoothing_angle_threshold(10.0)
        .build();

    let pathfinder = RePathfinder::new(settings_moderate)?;

    if let Some(path) = pathfinder.find_path(start, end)? {
        println!("  Path found with {} waypoints (SMOOTHED)", path.len());

        for (i, node) in path.iter().take(3).enumerate() {
            println!("    Waypoint {}: ({:.1}, {:.1}, {:.1})", i, node.x, node.y, node.z);
        }
        if path.len() > 3 {
            println!("    ... {} more waypoints ...", path.len() - 3);
        }
    }

    // Test 3: With aggressive smoothing
    println!("\nTest 3: Pathfinding WITH aggressive smoothing (5°)");
    let settings_aggressive = RePathSettingsBuilder::new("navmesh_varied.obj")
        .enable_path_smoothing(true)
        .smoothing_angle_threshold(5.0)
        .build();

    let pathfinder = RePathfinder::new(settings_aggressive)?;

    if let Some(path) = pathfinder.find_path(start, end)? {
        println!("  Path found with {} waypoints (AGGRESSIVE)", path.len());

        for (i, node) in path.iter().take(3).enumerate() {
            println!("    Waypoint {}: ({:.1}, {:.1}, {:.1})", i, node.x, node.y, node.z);
        }
        if path.len() > 3 {
            println!("    ... {} more waypoints ...", path.len() - 3);
        }
    }

    println!("\n=== Smoothing Impact ===");
    println!("Lower angle threshold = more aggressive smoothing = fewer waypoints");
    println!("Typical reduction: 30-70% fewer waypoints");
    println!("Benefits:");
    println!("  • Faster to process in game logic");
    println!("  • Smoother, more natural paths");
    println!("  • Reduced network bandwidth for multiplayer");
    println!("  • Better path visualization");

    Ok(())
}
