use std::collections::VecDeque;
use repath::{RePathfinder, RePathSettingsBuilder};
use dashmap::DashMap;
use std::sync::Arc;

#[test]
fn test_pathfinding_connected_nodes() {
    // Use builder pattern with the correct navmesh file
    let settings = RePathSettingsBuilder::new("navmesh_varied.obj")
        .precompute_radius(100.0)
        .total_precompute_pairs(100)
        .build();

    // Create pathfinder
    let pathfinder = RePathfinder::new(settings).expect("Failed to create pathfinder");

    // Get access to the graph for testing
    let graph = &pathfinder.graph;

    // Find connected nodes
    let (start_node_id, goal_node_id) = find_connected_nodes(graph).expect("No connected nodes found");

    // Print node information for debugging
    println!(
        "Start node ID: {}, Position: {:?}",
        start_node_id, graph.nodes[start_node_id]
    );
    println!(
        "Goal node ID: {}, Position: {:?}",
        goal_node_id, graph.nodes[goal_node_id]
    );

    // Create cache and stats for A* call
    let cache = DashMap::new();
    let stats = Arc::new(repath::metrics::PathfindingStats::new());

    // Run the A* algorithm to find a path between the start and goal nodes
    let path = graph.a_star(start_node_id, goal_node_id, &cache, &stats);

    // Assert that a path was found
    assert!(path.is_some(), "No path found between start and goal nodes");
}

fn find_connected_nodes(graph: &repath::graph::Graph) -> Option<(usize, usize)> {
    for start_node_id in 0..graph.nodes.len() {
        for goal_node_id in (start_node_id + 1)..graph.nodes.len() {
            if are_nodes_connected(graph, start_node_id, goal_node_id) {
                return Some((start_node_id, goal_node_id));
            }
        }
    }
    None
}

fn are_nodes_connected(graph: &repath::graph::Graph, start: usize, goal: usize) -> bool {
    let mut visited = vec![false; graph.nodes.len()];
    let mut queue = VecDeque::new();
    queue.push_back(start);

    while let Some(current) = queue.pop_front() {
        if current == goal {
            return true;
        }
        if visited[current] {
            continue;
        }
        visited[current] = true;
        for edge in &graph.edges[current] {
            if !visited[edge.to] {
                queue.push_back(edge.to);
            }
        }
    }
    false
}
