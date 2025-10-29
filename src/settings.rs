use serde::{Serialize, Deserialize};

/// Configuration settings for the RePathfinder.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RePathSettings {
    /// Navmesh filename (Wavefront OBJ format).
    pub navmesh_filename: String,

    /// Precompute radius in world units.
    pub precompute_radius: f32,

    /// Number of paths to precompute during initialization.
    pub total_precompute_pairs: usize,

    /// Enable path precomputation and caching.
    pub use_precomputed_cache: bool,

    /// Enable path smoothing to reduce waypoints.
    pub enable_path_smoothing: bool,

    /// Angle threshold for smoothing (degrees). Typical: 5-15.
    pub smoothing_angle_threshold: f32,

    /// Use bidirectional A* (2-3x faster for long paths).
    pub use_bidirectional_search: bool,
}

/// Builder for RePathSettings to provide a more ergonomic API
pub struct RePathSettingsBuilder {
    settings: RePathSettings,
}

impl RePathSettingsBuilder {
    /// Create a new builder with the given navmesh filename
    pub fn new(navmesh_filename: impl Into<String>) -> Self {
        Self {
            settings: RePathSettings {
                navmesh_filename: navmesh_filename.into(),
                precompute_radius: 100.0,
                total_precompute_pairs: 1000,
                use_precomputed_cache: true,
                enable_path_smoothing: true,
                smoothing_angle_threshold: 10.0,
                use_bidirectional_search: false,
            },
        }
    }

    /// Set the precompute radius
    pub fn precompute_radius(mut self, radius: f32) -> Self {
        self.settings.precompute_radius = radius;
        self
    }

    /// Set the total number of precompute pairs
    pub fn total_precompute_pairs(mut self, pairs: usize) -> Self {
        self.settings.total_precompute_pairs = pairs;
        self
    }

    /// Enable or disable precomputation
    pub fn use_precomputed_cache(mut self, enabled: bool) -> Self {
        self.settings.use_precomputed_cache = enabled;
        self
    }

    /// Enable or disable path smoothing
    pub fn enable_path_smoothing(mut self, enabled: bool) -> Self {
        self.settings.enable_path_smoothing = enabled;
        self
    }

    /// Set the angle threshold for path smoothing (in degrees)
    pub fn smoothing_angle_threshold(mut self, threshold: f32) -> Self {
        self.settings.smoothing_angle_threshold = threshold;
        self
    }

    /// Enable or disable bidirectional A* search
    pub fn use_bidirectional_search(mut self, enabled: bool) -> Self {
        self.settings.use_bidirectional_search = enabled;
        self
    }

    /// Build the RePathSettings
    pub fn build(self) -> RePathSettings {
        self.settings
    }
}
