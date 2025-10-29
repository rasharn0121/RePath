use std::collections::BinaryHeap;
use std::sync::Arc;
use dashmap::DashMap;
use crate::graph::{Graph, State};
use crate::path::Path;
use crate::metrics::PathfindingStats;
use crate::memory_pool;

/// Bidirectional A* search - searches from both start and goal simultaneously
/// More efficient for long-distance paths as it explores fewer nodes
pub fn bidirectional_a_star(
    graph: &Graph,
    start: usize,
    goal: usize,
    cache: &DashMap<(usize, usize), Option<Path>>,
    stats: &PathfindingStats,
) -> Option<Path> {
    // Early exit checks (same as regular A*)
    if start == goal {
        stats.record_early_exit_same_node();
        return Some(Arc::new(vec![graph.nodes[start]]));
    }

    let cache_key = (start, goal);
    if let Some(result) = cache.get(&cache_key) {
        stats.record_cache_hit();
        return result.clone();
    }

    stats.record_cache_miss();

    // Check if nodes are adjacent
    if graph.edges[start].iter().any(|e| e.to == goal) {
        stats.record_early_exit_adjacent();
        let path = vec![graph.nodes[start], graph.nodes[goal]];
        let result = Some(Arc::new(path));
        cache.insert(cache_key, result.clone());
        return result;
    }

    let num_nodes = graph.nodes.len();

    memory_pool::with_pool(num_nodes, |pool| {
        let (came_from_forward, g_score_forward, f_score_forward, closed_forward) = pool.get_mut();

        // We need separate data structures for backward search
        // Allocate them locally (could optimize with a second pool)
        let mut came_from_backward = vec![None; num_nodes];
        let mut g_score_backward = vec![f32::INFINITY; num_nodes];
        let mut f_score_backward = vec![f32::INFINITY; num_nodes];
        let mut closed_backward = vec![false; num_nodes];

        let mut open_forward = BinaryHeap::with_capacity(num_nodes / 2);
        let mut open_backward = BinaryHeap::with_capacity(num_nodes / 2);

        // Initialize forward search
        g_score_forward[start] = 0.0;
        f_score_forward[start] = graph.heuristic(start, goal);
        open_forward.push(State {
            cost: f_score_forward[start],
            position: start,
        });

        // Initialize backward search
        g_score_backward[goal] = 0.0;
        f_score_backward[goal] = graph.heuristic(goal, start);
        open_backward.push(State {
            cost: f_score_backward[goal],
            position: goal,
        });

        let mut best_path_cost = f32::INFINITY;
        let mut meeting_point: Option<usize> = None;
        let mut nodes_explored = 0;

        // Alternate between forward and backward search
        while !open_forward.is_empty() && !open_backward.is_empty() {
            // Forward step
            if let Some(State { cost: _, position: current }) = open_forward.pop() {
                if closed_forward[current] {
                    continue;
                }
                closed_forward[current] = true;
                nodes_explored += 1;

                // Check if we've met the backward search
                if closed_backward[current] {
                    let path_cost = g_score_forward[current] + g_score_backward[current];
                    if path_cost < best_path_cost {
                        best_path_cost = path_cost;
                        meeting_point = Some(current);
                    }
                }

                // If we found a path and current node is worse, we're done
                if meeting_point.is_some() && g_score_forward[current] >= best_path_cost {
                    break;
                }

                // Expand forward
                for edge in &graph.edges[current] {
                    let neighbor = edge.to;
                    if closed_forward[neighbor] {
                        continue;
                    }

                    let tentative_g = g_score_forward[current] + edge.cost;
                    if tentative_g < g_score_forward[neighbor] {
                        came_from_forward[neighbor] = Some(current);
                        g_score_forward[neighbor] = tentative_g;
                        f_score_forward[neighbor] = tentative_g + graph.heuristic(neighbor, goal);
                        open_forward.push(State {
                            cost: f_score_forward[neighbor],
                            position: neighbor,
                        });
                    }
                }
            }

            // Backward step
            if let Some(State { cost: _, position: current }) = open_backward.pop() {
                if closed_backward[current] {
                    continue;
                }
                closed_backward[current] = true;
                nodes_explored += 1;

                // Check if we've met the forward search
                if closed_forward[current] {
                    let path_cost = g_score_forward[current] + g_score_backward[current];
                    if path_cost < best_path_cost {
                        best_path_cost = path_cost;
                        meeting_point = Some(current);
                    }
                }

                // If we found a path and current node is worse, we're done
                if meeting_point.is_some() && g_score_backward[current] >= best_path_cost {
                    break;
                }

                // Expand backward
                for edge in &graph.edges[current] {
                    let neighbor = edge.to;
                    if closed_backward[neighbor] {
                        continue;
                    }

                    let tentative_g = g_score_backward[current] + edge.cost;
                    if tentative_g < g_score_backward[neighbor] {
                        came_from_backward[neighbor] = Some(current);
                        g_score_backward[neighbor] = tentative_g;
                        f_score_backward[neighbor] = tentative_g + graph.heuristic(neighbor, start);
                        open_backward.push(State {
                            cost: f_score_backward[neighbor],
                            position: neighbor,
                        });
                    }
                }
            }
        }

        stats.record_nodes_explored(nodes_explored);

        // Reconstruct path if found
        if let Some(meeting) = meeting_point {
            let mut forward_path = Vec::new();
            let mut current = meeting;

            // Build forward path
            forward_path.push(graph.nodes[current]);
            while let Some(next) = came_from_forward[current] {
                forward_path.push(graph.nodes[next]);
                current = next;
            }
            forward_path.reverse();

            // Build backward path
            let mut backward_path = Vec::new();
            current = meeting;
            while let Some(next) = came_from_backward[current] {
                backward_path.push(graph.nodes[next]);
                current = next;
            }

            // Combine paths (forward_path already has meeting point, so start from index 1 of backward)
            forward_path.extend(backward_path);

            let result = Some(Arc::new(forward_path));
            cache.insert(cache_key, result.clone());
            return result;
        }

        // No path found
        cache.insert(cache_key, None);
        None
    })
}
