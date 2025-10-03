# Bevy Regolith 🌙

A Bevy plugin for granular physics simulation using Position-Based Dynamics (PBD), designed for prototyping lunar regolith and other granular materials.

## Overview

`bevy_regolith` is a CPU-based prototype for testing granular physics algorithms with rigid body interactions. It serves as a development platform before migrating to the GPU-accelerated standalone [`regolith`](../regolith) engine.

## Features

- 🎮 **CPU-Based PBD**: Position-Based Dynamics solver running on CPU for easy debugging
- 🪨 **Granular Materials**: Lunar regolith, sand, snow presets
- 🎯 **Rigid Body Interaction**: Test particle-rigid body coupling
- 📊 **Debug Visualization**: Spatial grid, velocities, constraints
- 🎨 **Interactive**: Runtime parameter tuning with UI
- 🔧 **Bevy Native**: Full ECS integration with Bevy 0.17

## Quick Start

### Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
bevy = "0.17"
bevy_regolith = { path = "../bevy_regolith" }
```

### Basic Example

```rust
use bevy::prelude::*;
use bevy_regolith::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RegolithPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    // Camera with orbit controls
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 5.0, 10.0)
                .looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        OrbitCamera::default(),
    ));
    
    // Light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            ..default()
        },
        ..default()
    });
}
```

### Controls

- **Space**: Spawn 100 particles
- **R**: Spawn 500 particles
- **Right Mouse**: Rotate camera
- **Mouse Wheel**: Zoom camera
- **WASD**: Pan camera focus
- **QE**: Move camera up/down

## Examples

### Basic Simulation

```bash
cargo run --example basic
```

### Interactive Sandbox

```bash
cargo run --example sandbox
```

### Rigid Body Test

```bash
cargo run --example rigid_body_test
```

## Architecture

### Component Structure

```rust
// Particle components
ParticlePosition(Vec3)
ParticleVelocity(Vec3)
ParticlePrevPosition(Vec3)
ParticleMass(f32)
ParticleMaterial(usize)
ParticleRadius(f32)
ActiveParticle
```

### System Pipeline

```
FixedUpdate (60 Hz):
1. Predict Positions (apply gravity, integrate velocity)
2. Rebuild Spatial Hash (for neighbor queries)
3. Solve Constraints (particle-particle, particle-ground)
4. Update Velocities (from position changes)
5. Apply Damping

Update:
- Handle Input (spawn particles)
- Update Rendering (sync transforms)
- Debug Visualization
```

### Material Presets

```rust
// Lunar regolith
Material::lunar_regolith()

// Dry sand
Material::sand()

// Snow
Material::snow()
```

## Performance

**Current Target (CPU):**
- 5,000-10,000 particles @ 60 FPS
- 5 constraint iterations
- ~3-5ms physics update time

**Optimization Strategies:**
- Spatial hashing for O(n) neighbor queries
- Parallel iteration where possible
- Particle sleeping for static regions

## Development Roadmap

### Phase 1: Core Prototype ✅
- [x] Architecture design
- [x] Implementation plan
- [ ] Basic PBD solver
- [ ] Particle spawning
- [ ] Rendering system

### Phase 2: Features
- [ ] Rigid body interaction
- [ ] Debug visualization
- [ ] UI parameter tuning
- [ ] Performance profiling

### Phase 3: GPU Migration
- [ ] Compute shader implementation
- [ ] GPU buffer management
- [ ] Performance comparison

### Phase 4: Regolith Integration
- [ ] Wrap standalone `regolith` engine
- [ ] Maintain same API surface
- [ ] Focus on ECS integration

## Documentation

- [`BEVY_REGOLITH_ARCHITECTURE.md`](BEVY_REGOLITH_ARCHITECTURE.md) - Detailed architecture
- [`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md) - Step-by-step implementation guide
- [`../regolith/ARCHITECTURE.md`](../regolith/ARCHITECTURE.md) - Standalone engine architecture

## Comparison with Standalone Regolith

| Feature | bevy_regolith | regolith |
|---------|---------------|----------|
| **Platform** | CPU | GPU (compute shaders) |
| **Purpose** | Prototype & test | Production-ready |
| **Performance** | 5-10k particles | 500k-1M particles |
| **Dependencies** | Bevy | Standalone (wgpu) |
| **Use Case** | Development | Final integration |

## Contributing

This is a prototype project. Contributions welcome for:
- Performance optimizations
- Additional material presets
- Debug visualization improvements
- Documentation

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

## Acknowledgments

- **Bevy Engine**: ECS framework
- **PBD Paper**: Müller et al. "Position Based Dynamics" (2007)
- **Regolith Engine**: GPU architecture reference

## Related Projects

- [`regolith`](../regolith) - Standalone GPU-accelerated granular physics engine
- [`bevy_rapier`](https://github.com/dimforge/bevy_rapier) - Rigid body physics integration pattern

---

**Status**: 🚧 In Development - Architecture Complete, Implementation Pending

**Next Step**: Switch to Code mode to begin implementation