use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::time::Duration;

/// Statistics tracking for pathfinding operations
#[derive(Debug)]
pub struct PathfindingStats {
    /// Total number of pathfinding requests
    total_requests: AtomicUsize,

    /// Number of cache hits
    cache_hits: AtomicUsize,

    /// Number of cache misses
    cache_misses: AtomicUsize,

    /// Number of early exits (start == goal)
    early_exits_same_node: AtomicUsize,

    /// Number of adjacent node paths
    early_exits_adjacent: AtomicUsize,

    /// Total nodes explored in A* searches
    nodes_explored: AtomicUsize,

    /// Total computation time in microseconds
    total_computation_time_us: AtomicU64,

    /// Number of successful paths found
    successful_paths: AtomicUsize,

    /// Number of failed path searches
    failed_paths: AtomicUsize,
}

impl PathfindingStats {
    pub fn new() -> Self {
        Self {
            total_requests: AtomicUsize::new(0),
            cache_hits: AtomicUsize::new(0),
            cache_misses: AtomicUsize::new(0),
            early_exits_same_node: AtomicUsize::new(0),
            early_exits_adjacent: AtomicUsize::new(0),
            nodes_explored: AtomicUsize::new(0),
            total_computation_time_us: AtomicU64::new(0),
            successful_paths: AtomicUsize::new(0),
            failed_paths: AtomicUsize::new(0),
        }
    }

    pub fn record_request(&self) {
        self.total_requests.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_cache_hit(&self) {
        self.cache_hits.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_cache_miss(&self) {
        self.cache_misses.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_early_exit_same_node(&self) {
        self.early_exits_same_node.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_early_exit_adjacent(&self) {
        self.early_exits_adjacent.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_nodes_explored(&self, count: usize) {
        self.nodes_explored.fetch_add(count, Ordering::Relaxed);
    }

    pub fn record_computation_time(&self, duration: Duration) {
        self.total_computation_time_us.fetch_add(duration.as_micros() as u64, Ordering::Relaxed);
    }

    pub fn record_success(&self) {
        self.successful_paths.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_failure(&self) {
        self.failed_paths.fetch_add(1, Ordering::Relaxed);
    }

    /// Get a snapshot of current statistics
    pub fn snapshot(&self) -> StatsSnapshot {
        StatsSnapshot {
            total_requests: self.total_requests.load(Ordering::Relaxed),
            cache_hits: self.cache_hits.load(Ordering::Relaxed),
            cache_misses: self.cache_misses.load(Ordering::Relaxed),
            early_exits_same_node: self.early_exits_same_node.load(Ordering::Relaxed),
            early_exits_adjacent: self.early_exits_adjacent.load(Ordering::Relaxed),
            nodes_explored: self.nodes_explored.load(Ordering::Relaxed),
            total_computation_time_us: self.total_computation_time_us.load(Ordering::Relaxed),
            successful_paths: self.successful_paths.load(Ordering::Relaxed),
            failed_paths: self.failed_paths.load(Ordering::Relaxed),
        }
    }

    /// Reset all statistics
    pub fn reset(&self) {
        self.total_requests.store(0, Ordering::Relaxed);
        self.cache_hits.store(0, Ordering::Relaxed);
        self.cache_misses.store(0, Ordering::Relaxed);
        self.early_exits_same_node.store(0, Ordering::Relaxed);
        self.early_exits_adjacent.store(0, Ordering::Relaxed);
        self.nodes_explored.store(0, Ordering::Relaxed);
        self.total_computation_time_us.store(0, Ordering::Relaxed);
        self.successful_paths.store(0, Ordering::Relaxed);
        self.failed_paths.store(0, Ordering::Relaxed);
    }
}

impl Default for PathfindingStats {
    fn default() -> Self {
        Self::new()
    }
}

/// A snapshot of pathfinding statistics at a point in time
#[derive(Debug, Clone, Copy)]
pub struct StatsSnapshot {
    pub total_requests: usize,
    pub cache_hits: usize,
    pub cache_misses: usize,
    pub early_exits_same_node: usize,
    pub early_exits_adjacent: usize,
    pub nodes_explored: usize,
    pub total_computation_time_us: u64,
    pub successful_paths: usize,
    pub failed_paths: usize,
}

impl StatsSnapshot {
    /// Calculate cache hit rate as a percentage
    pub fn cache_hit_rate(&self) -> f64 {
        if self.total_requests == 0 {
            0.0
        } else {
            (self.cache_hits as f64 / self.total_requests as f64) * 100.0
        }
    }

    /// Calculate success rate as a percentage
    pub fn success_rate(&self) -> f64 {
        if self.total_requests == 0 {
            0.0
        } else {
            (self.successful_paths as f64 / self.total_requests as f64) * 100.0
        }
    }

    /// Calculate average computation time in microseconds
    pub fn avg_computation_time_us(&self) -> f64 {
        if self.total_requests == 0 {
            0.0
        } else {
            self.total_computation_time_us as f64 / self.total_requests as f64
        }
    }

    /// Calculate average nodes explored per search
    pub fn avg_nodes_explored(&self) -> f64 {
        let full_searches = self.total_requests
            .saturating_sub(self.cache_hits)
            .saturating_sub(self.early_exits_same_node)
            .saturating_sub(self.early_exits_adjacent);
        if full_searches == 0 {
            0.0
        } else {
            self.nodes_explored as f64 / full_searches as f64
        }
    }
}

impl std::fmt::Display for StatsSnapshot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Pathfinding Statistics:")?;
        writeln!(f, "  Total Requests: {}", self.total_requests)?;
        writeln!(f, "  Successful Paths: {}", self.successful_paths)?;
        writeln!(f, "  Failed Paths: {}", self.failed_paths)?;
        writeln!(f, "  Success Rate: {:.2}%", self.success_rate())?;
        writeln!(f, "  Cache Hits: {}", self.cache_hits)?;
        writeln!(f, "  Cache Hit Rate: {:.2}%", self.cache_hit_rate())?;
        writeln!(f, "  Early Exits (Same Node): {}", self.early_exits_same_node)?;
        writeln!(f, "  Early Exits (Adjacent): {}", self.early_exits_adjacent)?;
        writeln!(f, "  Total Nodes Explored: {}", self.nodes_explored)?;
        writeln!(f, "  Avg Nodes Explored: {:.2}", self.avg_nodes_explored())?;
        writeln!(f, "  Total Computation Time: {:.2}ms", self.total_computation_time_us as f64 / 1000.0)?;
        write!(f, "  Avg Computation Time: {:.2}µs", self.avg_computation_time_us())
    }
}
