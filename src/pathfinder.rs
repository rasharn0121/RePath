use crate::graph::Graph;
use crate::settings::RePathSettings;
use crate::utils::{nodes_within_radius, parse_obj};
use crate::error::{Result, RePathError};
use crate::metrics::PathfindingStats;
use crate::smoothing;
use crate::bidirectional;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::sync::Arc;
use dashmap::DashMap;
use rand::prelude::*;
use crate::path::Path;

/// The RePathfinder struct holds the graph and cache used for pathfinding.
pub struct RePathfinder {
    pub graph: Graph,
    cache: Arc<DashMap<(usize, usize), Option<Path>>>,
    stats: Arc<PathfindingStats>,
    settings: RePathSettings,
}

impl RePathfinder {
    /// Creates a new RePathfinder instance with the given settings.
    /// This includes loading the graph from the provided navmesh file and precomputing paths.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The navmesh file cannot be loaded
    /// - The OBJ file format is invalid
    /// - No valid nodes are found in the navmesh
    pub fn new(settings: RePathSettings) -> Result<Self> {
        let mut graph = parse_obj(&settings.navmesh_filename)?;

        // Build spatial index for fast nearest neighbor queries
        log::info!("Building spatial index for {} nodes", graph.nodes.len());
        graph.build_spatial_index();

        let cache = Arc::new(DashMap::new());

        let stats = Arc::new(PathfindingStats::new());

        // Only precompute if enabled
        if settings.use_precomputed_cache {
            let precompute_start = std::time::Instant::now();
            let node_ids: Vec<_> = (0..graph.nodes.len()).collect();

            log::info!("Starting precomputation of {} path pairs", settings.total_precompute_pairs);

            // Use a temporary stats object for precomputation (won't be exposed to user)
            let precompute_stats = PathfindingStats::new();

            // Precompute paths between random pairs of nodes within a specified radius
            (0..settings.total_precompute_pairs)
                .into_par_iter()
                .for_each(|_| {
                    let mut rng = rand::thread_rng();
                    if let Some(&start_node_id) = node_ids.choose(&mut rng) {
                        let start_node = &graph.nodes[start_node_id];
                        let mut nearby_nodes =
                            nodes_within_radius(&graph, start_node, settings.precompute_radius);

                        // Remove the start node from the list of nearby nodes if present
                        nearby_nodes.retain(|&id| id != start_node_id);

                        if let Some(&goal_node_id) = nearby_nodes.choose(&mut rng) {
                            if start_node_id != goal_node_id {
                                graph.a_star(start_node_id, goal_node_id, &cache, &precompute_stats);
                            }
                        }
                    }
                });

                let precompute_duration = precompute_start.elapsed();
            log::info!("Precomputation completed in {:?}", precompute_duration);
        } else {
            log::info!("Precomputation disabled");
        }

        let pathfinder = RePathfinder {
            graph,
            cache,
            stats,
            settings,
        };

        Ok(pathfinder)
    }

    /// Finds a path from start_coords to end_coords.
    ///
    /// # Errors
    ///
    /// Returns None if:
    /// - No nearest node can be found for the start or end coordinates
    /// - No path exists between the start and end nodes
    pub fn find_path(&self, start_coords: (f32, f32, f32), end_coords: (f32, f32, f32)) -> Result<Option<Path>> {
        self.stats.record_request();
        let start = std::time::Instant::now();

        let start_node_id = self.graph.nearest_node(start_coords.0, start_coords.1, start_coords.2)?;
        let end_node_id = self.graph.nearest_node(end_coords.0, end_coords.1, end_coords.2)?;

        // Choose algorithm based on settings
        let mut result = if self.settings.use_bidirectional_search {
            bidirectional::bidirectional_a_star(&self.graph, start_node_id, end_node_id, &self.cache, &self.stats)
        } else {
            self.graph.a_star(start_node_id, end_node_id, &self.cache, &self.stats)
        };

        // Apply path smoothing if enabled
        if self.settings.enable_path_smoothing {
            if let Some(path) = result {
                let smoothed = smoothing::smooth_path_combined(
                    &self.graph,
                    &path,
                    self.settings.smoothing_angle_threshold,
                );
                result = Some(Arc::new(smoothed));
            }
        }

        self.stats.record_computation_time(start.elapsed());

        if result.is_some() {
            self.stats.record_success();
        } else {
            self.stats.record_failure();
        }

        Ok(result)
    }

    /// Get a reference to the pathfinding statistics
    pub fn stats(&self) -> &Arc<PathfindingStats> {
        &self.stats
    }

    /// Finds a path from start_coords to end_coords using multiple threads.
    /// IMPROVED: First finds the complete path, then segments it at actual waypoints
    /// and refines those segments in parallel for better accuracy.
    ///
    /// # Arguments
    ///
    /// * `start_coords` - Starting coordinates (x, y, z)
    /// * `end_coords` - Ending coordinates (x, y, z)
    /// * `segment_count` - Number of segments to split the path into (should be > 1 for multithreading)
    ///
    /// # Errors
    ///
    /// Returns an error if nearest nodes cannot be found for any coordinates.
    /// Returns None if no path exists.
    ///
    /// # Note
    /// For short paths, single-threaded pathfinding is faster due to lower overhead.
    /// This method is best for long-distance paths where parallel processing provides benefit.
    pub fn find_path_multithreaded(
        &self,
        start_coords: (f32, f32, f32),
        end_coords: (f32, f32, f32),
        segment_count: u16,
    ) -> Result<Option<Path>> {
        if segment_count <= 1 {
            return self.find_path(start_coords, end_coords);
        }

        // Step 1: Find the initial path using single-threaded search
        // This gives us actual navmesh waypoints to use for segmentation
        let initial_path = self.find_path(start_coords, end_coords)?;

        if initial_path.is_none() {
            return Ok(None);
        }

        let initial_path = initial_path.unwrap();

        // If path is too short to benefit from segmentation, return as-is
        if initial_path.len() < segment_count as usize * 2 {
            return Ok(Some(initial_path));
        }

        // Step 2: Segment the path at evenly-spaced waypoints
        let path_len = initial_path.len();
        let segment_size = path_len / segment_count as usize;

        // Create waypoint indices for segmentation
        let mut waypoint_indices = vec![0];
        for i in 1..segment_count {
            waypoint_indices.push(i as usize * segment_size);
        }
        waypoint_indices.push(path_len - 1);

        // Step 3: Refine each segment in parallel
        // This can find better sub-paths within each segment
        let segments: Vec<_> = waypoint_indices.windows(2)
            .map(|w| (initial_path[w[0]].id, initial_path[w[1]].id))
            .collect();

        let paths: std::result::Result<Vec<_>, RePathError> = segments
            .into_par_iter()
            .map(|(start_id, end_id)| {
                // Use the algorithm based on settings
                let result = if self.settings.use_bidirectional_search {
                    bidirectional::bidirectional_a_star(&self.graph, start_id, end_id, &self.cache, &self.stats)
                } else {
                    self.graph.a_star(start_id, end_id, &self.cache, &self.stats)
                };
                Ok(result)
            })
            .collect();

        let paths = paths?;

        // Step 4: Combine the refined segments
        let mut full_path = Vec::new();
        for path_option in paths {
            if let Some(path) = path_option {
                if !full_path.is_empty() {
                    full_path.pop(); // Remove duplicate node at segment boundary
                }
                full_path.extend(path.iter());
            } else {
                // Segment failed - fall back to initial path
                return Ok(Some(initial_path));
            }
        }

        // Apply smoothing if enabled
        let mut result = Some(Arc::new(full_path));
        if self.settings.enable_path_smoothing {
            if let Some(path) = result {
                let smoothed = smoothing::smooth_path_combined(
                    &self.graph,
                    &path,
                    self.settings.smoothing_angle_threshold,
                );
                result = Some(Arc::new(smoothed));
            }
        }

        Ok(result)
    }
}
