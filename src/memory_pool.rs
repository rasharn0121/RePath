use std::cell::RefCell;

/// Thread-local memory pool for A* algorithm scratch space
/// Reuses allocations across multiple pathfinding calls to reduce GC pressure
pub struct AStarMemoryPool {
    came_from: Vec<Option<usize>>,
    g_score: Vec<f32>,
    f_score: Vec<f32>,
    closed_set: Vec<bool>,
}

impl AStarMemoryPool {
    fn new() -> Self {
        Self {
            came_from: Vec::new(),
            g_score: Vec::new(),
            f_score: Vec::new(),
            closed_set: Vec::new(),
        }
    }

    /// Prepare the pool for a search with the given number of nodes
    /// Reuses existing allocations when possible
    pub fn prepare(&mut self, num_nodes: usize) {
        // Resize if needed (grows but never shrinks to avoid repeated allocations)
        if self.came_from.len() < num_nodes {
            self.came_from.resize(num_nodes, None);
            self.g_score.resize(num_nodes, f32::INFINITY);
            self.f_score.resize(num_nodes, f32::INFINITY);
            self.closed_set.resize(num_nodes, false);
        }

        // Reset values for reuse
        self.came_from[..num_nodes].fill(None);
        self.g_score[..num_nodes].fill(f32::INFINITY);
        self.f_score[..num_nodes].fill(f32::INFINITY);
        self.closed_set[..num_nodes].fill(false);
    }

    /// Get mutable references to the scratch vectors
    #[allow(clippy::type_complexity)]
    pub fn get_mut(&mut self) -> (&mut Vec<Option<usize>>, &mut Vec<f32>, &mut Vec<f32>, &mut Vec<bool>) {
        (
            &mut self.came_from,
            &mut self.g_score,
            &mut self.f_score,
            &mut self.closed_set,
        )
    }
}

thread_local! {
    static POOL: RefCell<AStarMemoryPool> = RefCell::new(AStarMemoryPool::new());
}

/// Borrow the thread-local memory pool for A* pathfinding
pub fn with_pool<F, R>(num_nodes: usize, f: F) -> R
where
    F: FnOnce(&mut AStarMemoryPool) -> R,
{
    POOL.with(|pool| {
        let mut pool = pool.borrow_mut();
        pool.prepare(num_nodes);
        f(&mut pool)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_reuse() {
        // First use - will allocate
        with_pool(100, |pool| {
            let (came_from, g_score, f_score, closed_set) = pool.get_mut();
            assert!(came_from.len() >= 100);
            assert!(g_score.len() >= 100);
            assert!(f_score.len() >= 100);
            assert!(closed_set.len() >= 100);

            // Modify some values
            g_score[0] = 42.0;
            closed_set[0] = true;
        });

        // Second use - should reuse and reset
        with_pool(100, |pool| {
            let (came_from, g_score, f_score, closed_set) = pool.get_mut();
            // Values should be reset
            assert_eq!(g_score[0], f32::INFINITY);
            assert_eq!(closed_set[0], false);
        });
    }

    #[test]
    fn test_pool_grows() {
        // Start small
        with_pool(10, |pool| {
            let (came_from, _, _, _) = pool.get_mut();
            let initial_capacity = came_from.capacity();
            assert!(initial_capacity >= 10);
        });

        // Request larger - should grow
        with_pool(1000, |pool| {
            let (came_from, _, _, _) = pool.get_mut();
            assert!(came_from.len() >= 1000);
        });

        // Back to small - should still have large capacity
        with_pool(10, |pool| {
            let (came_from, _, _, _) = pool.get_mut();
            // Capacity should be at least 1000 from previous use
            assert!(came_from.capacity() >= 1000);
        });
    }
}
