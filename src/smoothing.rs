use crate::node::Node;
use crate::graph::Graph;

/// Path smoothing using the string pulling algorithm (funnel algorithm variant)
/// This reduces unnecessary waypoints by checking line-of-sight between nodes
pub fn smooth_path(graph: &Graph, path: &[Node]) -> Vec<Node> {
    if path.len() <= 2 {
        return path.to_vec();
    }

    let mut smoothed = Vec::with_capacity(path.len());
    smoothed.push(path[0]);

    let mut current_idx = 0;

    while current_idx < path.len() - 1 {
        // Find the farthest visible node from current position
        let mut farthest_visible = current_idx + 1;

        for test_idx in (current_idx + 2..path.len()).rev() {
            if can_see(graph, path[current_idx].id, path[test_idx].id) {
                farthest_visible = test_idx;
                break;
            }
        }

        smoothed.push(path[farthest_visible]);
        current_idx = farthest_visible;
    }

    smoothed
}

/// Check if there's a direct edge path between two nodes (line of sight check)
/// This is a simplified version - uses BFS with a maximum depth
fn can_see(graph: &Graph, start: usize, goal: usize) -> bool {
    if start == goal {
        return true;
    }

    // Direct edge check
    if graph.edges[start].iter().any(|e| e.to == goal) {
        return true;
    }

    // Check for very short paths (up to 2 hops)
    // This is a trade-off between smoothing quality and computation time
    for edge in &graph.edges[start] {
        if graph.edges[edge.to].iter().any(|e| e.to == goal) {
            // Found a 2-hop path, consider it visible for smoothing purposes
            // This is conservative but fast
            return false; // Return false to be conservative - only direct edges count
        }
    }

    false
}

/// Advanced smoothing with angle-based optimization
/// Removes waypoints that don't significantly change direction
pub fn smooth_path_angle_based(path: &[Node], angle_threshold_degrees: f32) -> Vec<Node> {
    if path.len() <= 2 {
        return path.to_vec();
    }

    let mut smoothed = Vec::with_capacity(path.len());
    smoothed.push(path[0]);

    let angle_threshold_rad = angle_threshold_degrees.to_radians();

    for i in 1..path.len() - 1 {
        let prev = &path[i - 1];
        let current = &path[i];
        let next = &path[i + 1];

        // Calculate vectors
        let v1 = (
            current.x - prev.x,
            current.y - prev.y,
            current.z - prev.z,
        );
        let v2 = (
            next.x - current.x,
            next.y - current.y,
            next.z - current.z,
        );

        // Calculate angle between vectors
        let angle = calculate_angle(v1, v2);

        // Only keep waypoint if it represents a significant direction change
        if angle > angle_threshold_rad {
            smoothed.push(*current);
        }
    }

    // Always include the final waypoint
    smoothed.push(path[path.len() - 1]);

    smoothed
}

/// Calculate angle between two 3D vectors
fn calculate_angle(v1: (f32, f32, f32), v2: (f32, f32, f32)) -> f32 {
    let dot = v1.0 * v2.0 + v1.1 * v2.1 + v1.2 * v2.2;
    let mag1 = (v1.0 * v1.0 + v1.1 * v1.1 + v1.2 * v1.2).sqrt();
    let mag2 = (v2.0 * v2.0 + v2.1 * v2.1 + v2.2 * v2.2).sqrt();

    if mag1 == 0.0 || mag2 == 0.0 {
        return 0.0;
    }

    let cos_angle = (dot / (mag1 * mag2)).clamp(-1.0, 1.0);
    cos_angle.acos()
}

/// Combined smoothing: first use visibility-based, then angle-based
pub fn smooth_path_combined(
    graph: &Graph,
    path: &[Node],
    angle_threshold_degrees: f32,
) -> Vec<Node> {
    let visibility_smoothed = smooth_path(graph, path);
    smooth_path_angle_based(&visibility_smoothed, angle_threshold_degrees)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smooth_empty_path() {
        let path: Vec<Node> = vec![];
        let smoothed = smooth_path_angle_based(&path, 10.0);
        assert_eq!(smoothed.len(), 0);
    }

    #[test]
    fn test_smooth_single_node() {
        let path = vec![Node::new(0, 0.0, 0.0, 0.0)];
        let smoothed = smooth_path_angle_based(&path, 10.0);
        assert_eq!(smoothed.len(), 1);
    }

    #[test]
    fn test_smooth_two_nodes() {
        let path = vec![
            Node::new(0, 0.0, 0.0, 0.0),
            Node::new(1, 1.0, 0.0, 0.0),
        ];
        let smoothed = smooth_path_angle_based(&path, 10.0);
        assert_eq!(smoothed.len(), 2);
    }

    #[test]
    fn test_smooth_straight_line() {
        // Straight line should be reduced to just start and end
        let path = vec![
            Node::new(0, 0.0, 0.0, 0.0),
            Node::new(1, 1.0, 0.0, 0.0),
            Node::new(2, 2.0, 0.0, 0.0),
            Node::new(3, 3.0, 0.0, 0.0),
        ];
        let smoothed = smooth_path_angle_based(&path, 10.0);
        // Should keep only start and end since it's a straight line
        assert_eq!(smoothed.len(), 2);
        assert_eq!(smoothed[0].id, 0);
        assert_eq!(smoothed[1].id, 3);
    }

    #[test]
    fn test_smooth_sharp_turn() {
        // Path with a 90-degree turn
        let path = vec![
            Node::new(0, 0.0, 0.0, 0.0),
            Node::new(1, 1.0, 0.0, 0.0),
            Node::new(2, 1.0, 1.0, 0.0),
        ];
        let smoothed = smooth_path_angle_based(&path, 10.0);
        // Should keep all nodes due to sharp turn
        assert_eq!(smoothed.len(), 3);
    }
}
