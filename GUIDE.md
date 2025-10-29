# RePath Complete Guide

The comprehensive guide to understanding and using RePath for pathfinding.

---

## Table of Contents

### Getting Started
- [What is Pathfinding?](#what-is-pathfinding)
- [What is the A* Algorithm?](#what-is-the-a-algorithm)
- [What is a Navmesh?](#what-is-a-navmesh)
- [Quick Start](#quick-start)

### Core Concepts
- [Understanding the Builder Pattern](#understanding-the-builder-pattern)
- [How Pathfinding Works in RePath](#how-pathfinding-works-in-repath)
- [The Path Cache System](#the-path-cache-system)

### Configuration
- [All Settings Explained](#all-settings-explained)
  - [precompute_radius](#1-precompute_radius)
  - [total_precompute_pairs](#2-total_precompute_pairs)
  - [use_precomputed_cache](#3-use_precomputed_cache)
  - [enable_path_smoothing](#4-enable_path_smoothing)
  - [smoothing_angle_threshold](#5-smoothing_angle_threshold)
  - [use_bidirectional_search](#6-use_bidirectional_search)

### Advanced Topics
- [Performance Metrics](#performance-metrics)
- [Creating Custom Navmeshes](#creating-custom-navmeshes)
- [Optimization Strategies](#optimization-strategies)
- [Troubleshooting](#troubleshooting)

---

## What is Pathfinding?

**Pathfinding** is the process of finding the shortest valid route between two points while avoiding obstacles.

### Real-World Analogy
Think of GPS navigation in your car:
- You enter a destination (point B)
- GPS knows your current location (point A)
- It calculates a route avoiding blocked roads and obstacles
- You follow the waypoints (turn-by-turn directions)

### In Games
NPCs (non-player characters) use pathfinding to:
- Navigate around walls and obstacles
- Find the player's location
- Patrol predefined routes
- Move naturally through the game world

**RePath handles all of this automatically** - you just provide start/end coordinates and a navmesh.

---

## What is the A* Algorithm?

**A* (pronounced "A-star")** is a proven pathfinding algorithm that finds the shortest path efficiently.

### How It Works

Imagine you're navigating a maze:

```
S = Start, G = Goal, █ = Wall

┌─────────────┐
│ S     █     │
│       █     │
│   █████  G  │
│             │
└─────────────┘
```

**Simple approach (inefficient):**
1. Try every possible path randomly
2. Eventually stumble upon the exit
3. Takes forever!

**A* approach (smart):**
1. At each step, evaluate which direction gets you closer to the goal
2. Avoid paths you've already tried
3. Consider both distance traveled AND distance remaining
4. Always picks the most promising path first

**Result:** Finds the shortest path much faster!

### Why A* is Special

| Algorithm | Behavior | Speed |
|-----------|----------|-------|
| **Dijkstra** | Explores everywhere equally | Slow but thorough |
| **Greedy Best-First** | Only looks at distance to goal | Fast but can pick wrong path |
| **A*** | Combines both strategies | Fast AND finds shortest path ✓ |

### In RePath

RePath uses A* but makes it even faster with:
- **Early exits** for trivial cases (start == goal, adjacent nodes)
- **Spatial indexing** (KD-tree) for fast nearest neighbor queries
- **Bidirectional search** searches from both start and goal simultaneously
- **Caching** stores computed paths for instant reuse

---

## What is a Navmesh?

A **navmesh (navigation mesh)** is a 3D model representing walkable surfaces in your game world.

### Visual Explanation

```
Your Game World:          Navmesh (simplified):
┌─────────────┐          ┌─────────────┐
│ [Building]  │          │ ▓▓▓▓▓▓▓▓▓▓  │  ▓ = Non-walkable
│ ▓▓▓▓▓▓▓▓    │          │ ▓▓▓▓▓▓▓▓    │  □ = Walkable
│        │    │    →     │ □□□□□□□□    │
│ Road   Tree │          │ □□□□ ▓▓     │
│        ▓▓   │          │ □□□□ ▓▓     │
└─────────────┘          └─────────────┘
```

### What's Inside a Navmesh

The navmesh is saved as a **.obj file** (Wavefront OBJ format) containing:

1. **Vertices** - 3D points (x, y, z coordinates)
2. **Faces** - Triangles connecting vertices
3. **Edges** - Connections between adjacent triangles

**Example OBJ file:**
```obj
# Vertices (3D points)
v 100.0 0.0 100.0
v 105.0 0.0 100.0
v 100.0 0.0 105.0

# Faces (triangles using vertex indices)
f 1 2 3
```

### How RePath Uses the Navmesh

When you call `find_path(start, end)`:

1. **Find start triangle** - Which navmesh triangle contains the start point?
2. **Find end triangle** - Which triangle contains the end point?
3. **Run A*** - Navigate across connected triangles from start to end
4. **Return waypoints** - Give back the path as a series of 3D points

### The Provided Navmesh

RePath includes `navmesh_varied.obj` for learning and testing:

| Property | Value |
|----------|-------|
| **Size** | 4km × 4km |
| **Vertices** | ~40,000 points |
| **Triangles** | ~80,000 faces |
| **Terrain** | Varied elevation with obstacles |
| **Use for** | Testing, learning, benchmarking |

**When you need a custom navmesh:**
- Your game has unique level layouts
- Different scale (small dungeons vs large worlds)
- Specific gameplay needs (platformers, RTS, etc.)

See [Creating Custom Navmeshes](#creating-custom-navmeshes) for details.

---

## Quick Start

### Minimal Example

```rust
use repath::{RePathfinder, RePathSettingsBuilder};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Configure with defaults (only specify navmesh file)
    let settings = RePathSettingsBuilder::new("navmesh_varied.obj").build();

    // 2. Create the pathfinder
    let pathfinder = RePathfinder::new(settings)?;

    // 3. Find a path
    let start = (100.0, 0.0, 100.0);
    let end = (200.0, 0.0, 200.0);

    match pathfinder.find_path(start, end)? {
        Some(path) => println!("Found path with {} waypoints!", path.len()),
        None => println!("No path exists"),
    }

    Ok(())
}
```

**That's it!** RePath handles everything else automatically.

---

## Understanding the Builder Pattern

The **builder pattern** makes configuration easy by providing sensible defaults.

### The Old Way (Manual)

```rust
// Have to specify EVERY field manually
let settings = RePathSettings {
    navmesh_filename: "navmesh.obj".to_string(),
    precompute_radius: 100.0,
    total_precompute_pairs: 1000,
    use_precomputed_cache: true,
    enable_path_smoothing: true,
    smoothing_angle_threshold: 10.0,
    use_bidirectional_search: false,
};
```

❌ **Problems:**
- Verbose and repetitive
- Easy to forget a field
- No sensible defaults
- Must know all 7 fields

### The New Way (Builder)

```rust
// Only specify what you want to change!
let settings = RePathSettingsBuilder::new("navmesh.obj")
    .precompute_radius(200.0)  // Override default
    .build();                   // Everything else uses defaults
```

✅ **Benefits:**
- Clean and concise
- Sensible defaults for everything
- Only override what you need
- Chainable method calls

---

## How Pathfinding Works in RePath

### Step-by-Step Process

When you call `pathfinder.find_path(start, end)`:

#### 1. Initialization (happens once)

```
Load navmesh.obj
    ↓
Parse vertices and triangles
    ↓
Build graph (nodes = triangles, edges = connections)
    ↓
Create KD-tree for fast spatial queries
    ↓
Precompute common paths (if enabled)
    ↓
Ready for pathfinding!
```

#### 2. Finding a Path (every call)

```
find_path(start, end) called
    ↓
Check cache - already computed?
    ├─ YES → Return instantly (cache hit)
    └─ NO  → Continue ↓

Early exit checks:
    ├─ Start == End? → Return [start] (instant)
    ├─ Adjacent nodes? → Return [start, end] (instant)
    └─ Must search → Continue ↓

Find nearest navmesh node to start (KD-tree)
    ↓
Find nearest navmesh node to end (KD-tree)
    ↓
Run A* algorithm:
    ├─ Standard A* OR
    └─ Bidirectional A* (if enabled)
    ↓
Path found?
    ├─ YES → Apply smoothing (if enabled)
    └─ NO  → Return None
    ↓
Store in cache for future use
    ↓
Return path (Vec of waypoints)
```

---

## The Path Cache System

### What is Path Caching?

**Caching** stores computed paths so they can be reused instantly without recalculation.

### How It Works

```
First NPC: "Find path from A to B"
    ↓
RePath: "Not in cache. Computing..."
    ↓
Run A* (takes 5ms)
    ↓
Store result: Cache[A→B] = path
    ↓
Return path to NPC

---

Second NPC: "Find path from A to B"
    ↓
RePath: "Found in cache!"
    ↓
Return cached path (instant - <1µs)
```

### Two Types of Caching

#### 1. Precomputed Cache (Startup)

**When:** During initialization
**How:** Randomly generates path pairs within `precompute_radius`

**Benefit:** Common short-distance paths are instant from the start

#### 2. Runtime Cache (During Gameplay)

**When:** As paths are requested
**How:** Every computed path is automatically cached

---

## All Settings Explained

### 1. precompute_radius

**Type:** `f32` (meters)
**Default:** `100.0`

#### What It Means

The maximum distance for randomly generating path pairs during startup precomputation.

#### How It Works

```
Startup precomputation:
    ↓
For i = 1 to total_precompute_pairs:
    ↓
    Pick random point A on navmesh
    ↓
    Pick random point B within precompute_radius of A
    ↓
    Compute path A→B using A*
    ↓
    Store in cache
```

#### Why It's Useful

- **Faster initial paths**: Common short-distance paths are precomputed
- **Improved early-game performance**: No delay for first pathfinding requests
- **Better cache hit rate**: More paths available immediately

#### When to Change

| Map Type | Recommended Value | Reasoning |
|----------|-------------------|-----------|
| **Small dungeons** | 20-50m | NPCs mostly move short distances |
| **Medium levels** | 100-200m | Balanced coverage (default is good) |
| **Large open worlds** | 500-1000m | NPCs travel longer distances |
| **Disable precomputation** | 0m | Skip if startup time is critical |

---

### 2. total_precompute_pairs

**Type:** `usize` (count)
**Default:** `1000`

#### What It Means

Number of random path pairs to compute and cache during startup.

#### How It Works

```
Precomputation process:
    ↓
total_precompute_pairs = 1000
    ↓
Generate 1000 random (start, end) pairs
    ├─ All pairs must be within precompute_radius
    ├─ Points are randomly distributed across navmesh
    └─ Each pair is unique
    ↓
For each pair:
    Compute path using A*
    Store in cache
    ↓
Result: Cache now contains 1000 precomputed paths
```

#### Why It's Useful

- **More cached paths**: Higher value = more paths available instantly
- **Better cache coverage**: Increases chance of cache hit
- **Tradeoff**: More pairs = longer startup time

**Startup Time Examples (approximate):**
- 100 pairs: ~0.5 seconds
- 1000 pairs: ~1-3 seconds
- 5000 pairs: ~5-15 seconds
- 10000 pairs: ~10-30 seconds

---

### 3. use_precomputed_cache

**Type:** `bool`
**Default:** `true`

#### What It Means

Whether to enable the path caching system (both precomputed and runtime cache).

#### Why It's Useful

**Enabled (true - default):**
- ✅ Massive performance boost for repeated paths
- ✅ 60-80% of requests return instantly (typical)
- ✅ Reduced CPU usage over time

**Disabled (false):**
- ✅ Instant startup (no precomputation delay)
- ✅ No memory used for cache
- ⚠️ Every path request runs full A* (slower)

---

### 4. enable_path_smoothing

**Type:** `bool`
**Default:** `true`

#### What It Means

Whether to remove unnecessary waypoints from paths to make them more natural.

#### How It Works

**Without Smoothing (Raw A* output):**
```
Path follows every navmesh triangle:
●→●→●→●→●→●→●→●→●→●→●→●
(45 waypoints, lots of tiny adjustments)
```

**With Smoothing:**
```
Result: ●────→●────→●────→●────→●
(18 waypoints, smooth curves)
```

#### Why It's Useful

**Benefits:**
- **30-70% fewer waypoints** typical reduction
- **More natural movement** - characters don't zigzag
- **Lower network bandwidth** - fewer points to transmit (multiplayer)
- **Faster game logic** - less waypoints to process
- **Better visualization** - smoother path rendering

---

### 5. smoothing_angle_threshold

**Type:** `f32` (degrees)
**Default:** `10.0`

#### What It Means

How aggressively smoothing removes waypoints. Lower value = more aggressive smoothing.

#### How It Works

```
For each three consecutive waypoints (A, B, C):
    ↓
Calculate angle at waypoint B
    ↓
Is angle < smoothing_angle_threshold?
    ├─ YES → Remove waypoint B (path is straight enough)
    └─ NO  → Keep waypoint B (significant turn)
```

#### Why It's Useful

**Adjusts smoothing aggressiveness:**

| Threshold | Waypoints Removed | Path Quality | Use When |
|-----------|-------------------|--------------|----------|
| **5-8°** | Maximum (very aggressive) | Very smooth | Open terrain, few obstacles |
| **10°** | Balanced (default) | Smooth & safe | General use ✓ |
| **15-20°** | Conservative | More waypoints | Tight spaces, precision needed |

---

### 6. use_bidirectional_search

**Type:** `bool`
**Default:** `false`

#### What It Means

Use bidirectional A* algorithm that searches from both start and goal simultaneously.

#### How It Works

**Standard A* (default):**
```
Start searching from START node
    ↓
Explore outward toward GOAL
    ↓
    S ●───→───→───→───→● G
```

**Bidirectional A*:**
```
Start TWO searches simultaneously:
    ↓
Forward search from START
Backward search from GOAL
    ↓
    S ●───→───→ ← ←───●  G
           ↓      ↑
         They meet in middle!
```

#### Why It's Useful

**Advantages:**
- **2-3x faster** for long-distance paths (>500m)
- **Explores fewer nodes** - both searches meet in middle
- **Better for large maps** - scales well with distance

**Disadvantages:**
- **Slight overhead** for short paths (<100m)

**Performance Comparison:**

| Path Distance | Standard A* | Bidirectional A* | Speedup |
|---------------|-------------|------------------|---------|
| 50m (short) | 2ms | 2.5ms | 0.8x (slower) |
| 200m (medium) | 8ms | 6ms | 1.3x |
| 500m (long) | 25ms | 10ms | 2.5x |
| 2000m (very long) | 150ms | 50ms | 3x |

---

## Performance Metrics

### Understanding the Statistics

When you call `pathfinder.stats().snapshot()`, you get these metrics:

```rust
let stats = pathfinder.stats().snapshot();
println!("{}", stats);
```

**Output:**
```
Pathfinding Statistics:
  Total Requests: 150
  Successful Paths: 145
  Failed Paths: 5
  Success Rate: 96.67%
  Cache Hits: 95
  Cache Hit Rate: 63.33%
  Early Exits (Same Node): 10
  Early Exits (Adjacent): 15
  Total Nodes Explored: 12,450
  Avg Nodes Explored: 83.00
  Total Computation Time: 245.50ms
  Avg Computation Time: 1636.67µs
```

### What Each Metric Means

- **Total Requests**: Number of times `find_path()` was called
- **Cache Hit Rate**: Percentage returned from cache (60-80% is good)
- **Early Exits**: Times when trivial optimizations applied
- **Nodes Explored**: How many navmesh nodes A* examined
- **Computation Time**: Time spent pathfinding

---

## Creating Custom Navmeshes

### Method 1: Game Engine Tools (Easiest)

#### Unity
1. Use built-in NavMesh system
2. Open: `Window → AI → Navigation`
3. Click "Bake"
4. Export as OBJ

#### Unreal Engine
1. Add `Nav Mesh Bounds Volume`
2. Press `P` to visualize
3. Export using plugins

#### Godot
1. Add `NavigationRegion3D` node
2. Bake navigation mesh
3. Export to OBJ

### Method 2: Blender

1. Import your game world geometry
2. Create simplified walkable surface
3. Remove obstacles
4. Triangulate mesh
5. Export as Wavefront OBJ

---

## Optimization Strategies

### For Small Maps (<500m)

```rust
let settings = RePathSettingsBuilder::new("small_map.obj")
    .precompute_radius(50.0)
    .use_bidirectional_search(false)
    .build();
```

### For Large Maps (2km+)

```rust
let settings = RePathSettingsBuilder::new("large_map.obj")
    .precompute_radius(500.0)
    .total_precompute_pairs(5000)
    .use_bidirectional_search(true)  // ✓ Enable
    .build();
```

---

## Troubleshooting

### Paths go through walls
- Rebuild navmesh excluding obstacles
- Check for gaps in navmesh
- Verify scale matches game world

### No path found
- Check start/end points are on navmesh
- Verify navmesh connectivity
- Add logging to debug

### Pathfinding is slow
- Check cache hit rate
- Enable bidirectional search for large maps
- Increase precomputation

### Too many waypoints
- Enable path smoothing
- Lower angle threshold (5-8°)

---

*This guide covers RePath v0.1.0 with optimization updates.*
