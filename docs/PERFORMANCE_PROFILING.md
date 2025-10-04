# Performance Profiling Guide

This document provides guidance on profiling and optimizing bevy_regolith performance.

## Quick Start

### Running the Performance Benchmark

```bash
# Run with release optimizations (required for accurate measurements)
cargo run --example performance_benchmark --release

# Run with flamegraph profiling (requires cargo-flamegraph)
cargo install flamegraph
cargo flamegraph --example performance_benchmark
```

### Benchmark Controls

- **1-9**: Set particle count (1000-9000)
- **0**: Set particle count to 10000
- **Space**: Add 1000 particles
- **C**: Clear all particles
- **P**: Print performance statistics

## Performance Targets

### Phase 1 (CPU-based PBD)
- **Target**: 5,000-10,000 particles @ 60 FPS
- **Current**: Successfully tested with 3,000+ particles
- **Physics Update**: ~3-5ms per frame (estimated)
- **Constraint Iterations**: 5 per frame

### Phase 2 (Optimized CPU)
- **Target**: 10,000-15,000 particles @ 60 FPS
- **Optimizations**: Spatial hash improvements, SIMD, particle sleeping

### Phase 3 (GPU Compute)
- **Target**: 100,000+ particles @ 60 FPS
- **Approach**: Compute shaders for physics pipeline

## Profiling Tools

### 1. cargo-flamegraph

**Installation:**
```bash
cargo install flamegraph
```

**Usage:**
```bash
# Generate flamegraph
cargo flamegraph --example performance_benchmark

# Open flamegraph.svg in browser
```

**What to Look For:**
- Wide bars indicate hot paths (functions taking most time)
- Focus on functions in the physics pipeline:
  - `solve_constraints`
  - `rebuild_spatial_hash`
  - `query_neighbors`
  - `predict_positions`

### 2. Bevy Built-in Diagnostics

The performance benchmark example includes FPS monitoring:
- Press **P** to print detailed statistics
- Window title shows current FPS
- Tracks min/max/average FPS over time

## Known Bottlenecks

### 1. Constraint Solving (solve_constraints)

**Current Implementation:**
```rust
// O(n * m * iterations) where:
// n = number of particles
// m = average neighbors per particle
// iterations = 5
```

**Potential Optimizations:**
- [ ] Use parallel iteration with `par_iter()` (rayon)
- [ ] Reduce neighbor query radius
- [ ] Implement adaptive iteration count
- [ ] Cache neighbor lists between iterations

### 2. Spatial Hash Rebuild

**Current Implementation:**
```rust
// O(n) rebuild every frame
// Uses HashMap with Vec allocations
```

**Potential Optimizations:**
- [ ] Use pre-allocated Vec pools
- [ ] Implement incremental updates (only move changed particles)
- [ ] Use FxHashMap instead of std HashMap
- [ ] Consider fixed-size grid arrays for bounded domains

### 3. Neighbor Queries

**Current Implementation:**
```rust
// O(k³) where k = cell_radius
// Allocates new Vec for each query
```

**Potential Optimizations:**
- [ ] Pre-allocate neighbor buffer
- [ ] Use iterators instead of Vec allocation
- [ ] Implement distance-based early exit
- [ ] Cache queries for static particles

### 4. Rigid Body Collisions

**Current Implementation:**
```rust
// O(n * r) where:
// n = number of particles
// r = number of rigid bodies
```

**Potential Optimizations:**
- [ ] Use spatial partitioning for rigid bodies
- [ ] Implement broad-phase collision detection
- [ ] Skip collision checks for distant particles
- [ ] Use AABB tests before detailed collision

## Optimization Strategies

### Short-term (CPU Optimizations)

1. **Parallel Processing**
   - Use `par_iter()` for particle updates
   - Parallelize constraint solving iterations
   - Requires careful handling of mutable access

2. **Memory Optimization**
   - Reduce allocations in hot paths
   - Use object pools for temporary buffers
   - Consider SoA (Structure of Arrays) layout

3. **Algorithm Improvements**
   - Implement particle sleeping
   - Adaptive iteration counts
   - Better spatial partitioning

### Long-term (GPU Migration)

1. **Compute Shader Pipeline**
   - Position prediction on GPU
   - Constraint solving on GPU
   - Spatial hash on GPU
   - Only sync visible particles to CPU

2. **Hybrid Approach**
   - GPU for particle physics
   - CPU for rigid body interaction
   - Efficient GPU-CPU synchronization

## Benchmarking Methodology

### Standard Test Cases

1. **Baseline Test**: 3,000 particles, no rigid bodies
2. **Stress Test**: 10,000 particles, no rigid bodies
3. **Interaction Test**: 5,000 particles + 5 rigid bodies
4. **Extreme Test**: 20,000 particles (expected to drop below 60 FPS)

### Metrics to Track

- **FPS**: Frames per second (target: 60+)
- **Frame Time**: Milliseconds per frame (target: <16.67ms)
- **Physics Time**: Time spent in physics systems
- **Particle Count**: Number of active particles
- **Memory Usage**: Heap allocations and peak memory

### Recording Results

```bash
# Run benchmark and record results
cargo run --example performance_benchmark --release > benchmark_results.txt

# Press P periodically to print stats
# Test with different particle counts (1000, 3000, 5000, 10000)
```

## Performance Analysis Checklist

- [ ] Run flamegraph to identify hot paths
- [ ] Profile with Tracy for detailed system breakdown
- [ ] Test with 1k, 3k, 5k, 10k particles
- [ ] Measure frame time distribution (min/max/avg)
- [ ] Check for memory leaks or excessive allocations
- [ ] Verify spatial hash efficiency (cells per particle)
- [ ] Test rigid body collision performance
- [ ] Profile with different constraint iteration counts

## Next Steps

After profiling, prioritize optimizations based on:
1. **Impact**: How much time is spent in this code path?
2. **Complexity**: How difficult is the optimization?
3. **Risk**: Could it introduce bugs or instability?

Focus on high-impact, low-complexity optimizations first.

## Advanced Profiling (Optional)

For more detailed profiling, you can integrate Tracy or other profilers:

1. **Tracy Profiler**: Add `tracing-tracy` dependency and instrument systems
2. **perf**: Use Linux perf tools with `cargo build --release`
3. **Instruments**: Use macOS Instruments for detailed profiling

## References

- [cargo-flamegraph](https://github.com/flamegraph-rs/flamegraph)
- [Bevy Performance Tips](https://bevyengine.org/learn/book/getting-started/performance/)
- [Position-Based Dynamics Paper](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf)
- [Tracy Profiler](https://github.com/wolfpld/tracy) (optional advanced tool)