use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::Arc;
use dashmap::DashMap;
use rand::prelude::*;
use kiddo::{KdTree, SquaredEuclidean};
use crate::edge::Edge;
use crate::node::Node;
use crate::path::Path;
use crate::utils::distance;
use crate::error::{RePathError, Result};
use crate::metrics::PathfindingStats;
use crate::memory_pool;

/// Type alias for edge filter function
pub type EdgeFilter = dyn Fn(&Node, &Node, f32) -> bool;

pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Vec<Edge>>,
    spatial_index: Option<KdTree<f32, 3>>,
}

impl Default for Graph {
    fn default() -> Self {
        Self::new()
    }
}

impl Graph {
    pub fn new() -> Self {
        Graph {
            nodes: Vec::new(),
            edges: Vec::new(),
            spatial_index: None,
        }
    }

    pub fn add_node(&mut self, node: Node) {
        self.nodes.push(node);
        self.edges.push(Vec::new());
    }

    pub fn add_edge(&mut self, from: usize, to: usize, cost: f32) {
        self.edges[from].push(Edge { to, cost });
    }

    /// Build the spatial index for fast nearest neighbor queries
    /// Uses a try-catch pattern to handle edge cases with duplicate positions
    pub fn build_spatial_index(&mut self) {
        // Try to build KD-tree, but fall back gracefully if there are issues
        // (e.g., too many nodes with identical positions)
        let mut kdtree = KdTree::new();
        let mut success = true;

        for (id, node) in self.nodes.iter().enumerate() {
            // Catch panics from kiddo when there are too many duplicate positions
            if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                kdtree.add(&[node.x, node.y, node.z], id as u64);
            })).is_err() {
                log::warn!("Failed to build spatial index due to duplicate positions, falling back to linear search");
                success = false;
                break;
            }
        }

        if success {
            self.spatial_index = Some(kdtree);
            log::info!("Spatial index built successfully");
        } else {
            self.spatial_index = None;
            log::info!("Using linear search for nearest node queries");
        }
    }

    pub fn heuristic(&self, start: usize, goal: usize) -> f32 {
        let start_node = &self.nodes[start];
        let goal_node = &self.nodes[goal];
        distance(&(start_node.x, start_node.y, start_node.z), &(goal_node.x, goal_node.y, goal_node.z))
    }


    /// A* pathfinding with optional edge filtering for hybrid mode
    ///
    /// # Arguments
    /// * `start` - Starting node index
    /// * `goal` - Goal node index
    /// * `cache` - Path cache
    /// * `stats` - Statistics tracker
    /// * `edge_filter` - Optional filter function. If Some, edges are filtered using this predicate.
    ///   Filter receives (from_node, to_node, edge_cost) and returns true if edge should be used.
    ///
    /// # Returns
    /// The path as a vector of nodes, or None if no path exists
    pub fn a_star_filtered(
        &self,
        start: usize,
        goal: usize,
        cache: &DashMap<(usize, usize), Option<Path>>,
        stats: &PathfindingStats,
        edge_filter: Option<&EdgeFilter>,
    ) -> Option<Path> {
        // Early exit: if start equals goal, return single-node path
        if start == goal {
            stats.record_early_exit_same_node();
            return Some(Arc::new(vec![self.nodes[start]]));
        }

        let cache_key = (start, goal);

        // Only use cache if no filter is applied (filters change pathfinding behavior)
        if edge_filter.is_none() {
            if let Some(result) = cache.get(&cache_key) {
                stats.record_cache_hit();
                return result.clone();
            }
            stats.record_cache_miss();
        }

        // Early exit: check if nodes are directly connected (adjacent)
        if let Some(edge) = self.edges[start].iter().find(|e| e.to == goal) {
            // Apply filter if present
            if let Some(filter) = edge_filter {
                if !filter(&self.nodes[start], &self.nodes[goal], edge.cost) {
                    // Edge is filtered out, can't use direct path
                } else {
                    stats.record_early_exit_adjacent();
                    let path = vec![self.nodes[start], self.nodes[goal]];
                    let result = Some(Arc::new(path));
                    if edge_filter.is_none() {
                        cache.insert(cache_key, result.clone());
                    }
                    return result;
                }
            } else {
                stats.record_early_exit_adjacent();
                let path = vec![self.nodes[start], self.nodes[goal]];
                let result = Some(Arc::new(path));
                cache.insert(cache_key, result.clone());
                return result;
            }
        }

        let num_nodes = self.nodes.len();

        // Use thread-local memory pool to reuse allocations
        memory_pool::with_pool(num_nodes, |pool| {
            let (came_from, g_score, f_score, closed_set) = pool.get_mut();

            let mut open_set = BinaryHeap::with_capacity(num_nodes);

            g_score[start] = 0.0;
            f_score[start] = self.heuristic(start, goal);

            open_set.push(State {
                cost: f_score[start],
                position: start,
            });

            let mut nodes_explored = 0;

            while let Some(State { cost: _, position: current }) = open_set.pop() {
                if current == goal {
                    // Path found
                    stats.record_nodes_explored(nodes_explored);
                    let mut total_path = Vec::new();
                    let mut current = current;

                    total_path.push(self.nodes[current]);

                    while let Some(next) = came_from[current] {
                        total_path.push(self.nodes[next]);
                        current = next;
                    }

                    total_path.reverse();

                    let result = Some(Arc::new(total_path));

                    // Cache the result only if no filter
                    if edge_filter.is_none() {
                        cache.insert(cache_key, result.clone());
                    }

                    return result;
                }

                if closed_set[current] {
                    continue;
                }
                closed_set[current] = true;
                nodes_explored += 1;

                for edge in &self.edges[current] {
                    let neighbor = edge.to;

                    if closed_set[neighbor] {
                        continue;
                    }

                    // Apply edge filter if present
                    if let Some(filter) = edge_filter {
                        if !filter(&self.nodes[current], &self.nodes[neighbor], edge.cost) {
                            continue; // Skip this edge
                        }
                    }

                    let tentative_g_score = g_score[current] + edge.cost;

                    if tentative_g_score < g_score[neighbor] {
                        came_from[neighbor] = Some(current);
                        g_score[neighbor] = tentative_g_score;
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal);
                        open_set.push(State {
                            cost: f_score[neighbor],
                            position: neighbor,
                        });
                    }
                }
            }

            // No path found - record nodes explored before returning
            stats.record_nodes_explored(nodes_explored);

            // Cache the non-result only if no filter
            if edge_filter.is_none() {
                cache.insert(cache_key, None);
            }

            None
        }) // End of memory_pool::with_pool closure
    }

    pub fn a_star(
        &self,
        start: usize,
        goal: usize,
        cache: &DashMap<(usize, usize), Option<Path>>,
        stats: &PathfindingStats,
    ) -> Option<Path> {
        // Delegate to filtered version with no filter
        self.a_star_filtered(start, goal, cache, stats, None)
    }

    pub fn nearest_node(&self, x: f32, y: f32, z: f32) -> Result<usize> {
        // Use spatial index if available for O(log n) lookup
        if let Some(ref kdtree) = self.spatial_index {
            let nearest = kdtree.nearest_one::<SquaredEuclidean>(&[x, y, z]);
            return Ok(nearest.item as usize);
        }

        // Fallback to linear search O(n) if no spatial index
        self.nodes
            .iter()
            .enumerate()
            .map(|(id, node)| {
                let d = distance(&(node.x, node.y, node.z), &(x, y, z));
                (d, id)
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal))
            .map(|(_, id)| id)
            .ok_or(RePathError::NearestNodeNotFound(x, y, z))
    }

    pub fn random_node(&self) -> Option<usize> {
        let node_ids: Vec<_> = (0..self.nodes.len()).collect();
        if node_ids.is_empty() {
            None
        } else {
            let mut rng = thread_rng();
            Some(*node_ids.choose(&mut rng).unwrap())
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct State {
    pub cost: f32,
    pub position: usize,
}

impl State {
    pub fn new(cost: f32, position: usize) -> Self {
        Self { cost, position }
    }
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for State {}
