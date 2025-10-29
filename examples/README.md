# RePath Examples

Quick-start examples showing how to use RePath for pathfinding.

## Running Examples

```bash
# Basic pathfinding with metrics
cargo run --example getting_started

# Path smoothing demonstration
cargo run --example path_smoothing
```

---

## Example Files

### 1. `getting_started.rs`

**What it does:**
- Shows basic pathfinding setup
- Demonstrates the builder pattern (easy configuration with defaults)
- Displays performance statistics
- Exports metrics to CSV

**Use this to:**
- Learn the basic API
- Understand how to configure RePath
- See performance metrics in action

**Output:**
```
✓ Found path with 12 waypoints
  Waypoint 0: (100.00, 0.00, 100.00)
  ...

Pathfinding Statistics:
  Total Requests: 1
  Cache Hit Rate: 0.00%
  Avg Computation Time: 5230.00µs

✓ Statistics exported to pathfinding_stats.csv
```

---

### 2. `path_smoothing.rs`

**What it does:**
- Compares paths with and without smoothing
- Shows different smoothing aggressiveness levels
- Demonstrates waypoint reduction (30-70%)

**Use this to:**
- Understand path smoothing benefits
- See the impact of angle threshold settings
- Learn when to enable/disable smoothing

**Output:**
```
Test 1: WITHOUT smoothing - 45 waypoints (RAW)
Test 2: WITH moderate smoothing (10°) - 18 waypoints (SMOOTHED)
Test 3: WITH aggressive smoothing (5°) - 12 waypoints (AGGRESSIVE)
```

---

## What is the Builder Pattern?

The **builder pattern** makes configuration easy by providing sensible defaults.

**Simple example:**
```rust
// Just specify the navmesh file - everything else uses defaults!
let settings = RePathSettingsBuilder::new("navmesh_varied.obj").build();
```

**Customize what you need:**
```rust
// Only override specific settings
let settings = RePathSettingsBuilder::new("navmesh_varied.obj")
    .precompute_radius(200.0)     // Change this
    .enable_path_smoothing(true)  // Change this
    .build();                      // Everything else = defaults
```

**Why it's better:**
- ✅ Clean and easy to read
- ✅ Sensible defaults for everything
- ✅ Only specify what you want to change
- ✅ Can't forget required settings

For detailed explanations of all settings and defaults, see `GUIDE.md`.

---

## Next Steps

After running these examples:

1. **Read the Guide** - Check `GUIDE.md` in the root directory for comprehensive documentation
2. **Integrate into your game** - Copy the pattern from `getting_started.rs`
3. **Customize settings** - Adjust for your specific map size and gameplay needs

---

## Need Help?

- **Comprehensive Documentation**: See `GUIDE.md` for detailed explanations
- **Questions**: Open an issue on GitHub
- **Examples not working?**: Ensure `navmesh_varied.obj` is in the project root
