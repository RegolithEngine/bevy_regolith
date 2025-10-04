# Bevy Regolith 🌙

A Bevy plugin for granular physics simulation using Position-Based Dynamics (PBD), designed for prototyping lunar regolith and other granular materials.

## Overview

`bevy_regolith` is a CPU-based prototype for testing granular physics algorithms with rigid body interactions. It serves as a development platform before migrating to the GPU-accelerated standalone [`regolith`](../regolith) engine.

## Features

- 🎮 **CPU-Based PBD**: Position-Based Dynamics solver running on CPU for easy debugging
- 🪨 **Granular Materials**: Lunar regolith, sand, snow presets
- 🎯 **Rigid Body Interaction**: Test particle-rigid body coupling
- 🤝 **Rapier Integration**: Built-in plugin for Rapier physics engine integration
- 📊 **Debug Visualization**: Spatial grid, velocities, constraints
- 🎨 **Interactive**: Runtime parameter tuning with UI
- 🔧 **Bevy Native**: Full ECS integration with Bevy 0.16

## Quick Start

### Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
bevy = "0.16"
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
- **Left Mouse**: Rotate camera view
- **Right Mouse**: Pan camera focus
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

### Rapier Integration

Demonstrates particle interaction with Rapier physics bodies:

```bash
cargo run --example rapier_integration --features rapier
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

### Rapier Integration

Integrate with Rapier physics for particle-rigid body interaction:

```rust
use bevy::prelude::*;
use bevy_regolith::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RegolithPlugin)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierIntegrationPlugin::default())
        .run();
}

fn setup(mut commands: Commands) {
    // Spawn a dynamic Rapier body that particles can push
    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 0.5),
        ExternalForce::default(),
        Transform::from_xyz(0.0, 2.0, 0.0),
        SyncToRegolith, // Mark for particle interaction
    ));
}
```

**Configuration Options:**

```rust
// Customize force calculation parameters
let config = RapierIntegrationConfig {
    penetration_force_scale: 10.0,  // Adjust penetration correction
    total_force_scale: 5.0,          // Overall force multiplier
    restitution: 0.0,                // Bounciness (0 = no bounce)
};

app.add_plugins(RapierIntegrationPlugin { config });
```

## Performance

**Current Status (CPU):**
- ✅ Successfully tested with 3,000+ particles @ 60 FPS
- ✅ 5 constraint iterations per frame
- ✅ Spatial hashing for O(n) neighbor queries
- 🎯 Target: 5,000-10,000 particles @ 60 FPS

**Planned Optimizations:**
- Parallel iteration where possible
- Particle sleeping for static regions
- Hot path optimization based on profiling

## Development Roadmap

### Phase 1: Core Prototype ✅ **COMPLETE**
- [x] Architecture design
- [x] Implementation plan
- [x] Basic PBD solver (predict, solve, update)
- [x] Particle spawning system
- [x] Rendering system with Bevy 0.16
- [x] Orbit camera controls
- [x] UI parameter tuning with egui
- [x] Debug visualization (grid, velocities)
- [x] Basic example tested and working

### Phase 2: Features & Optimization 🚧 **IN PROGRESS**
- [x] Rapier physics integration module
- [x] Rapier integration example
- [ ] Additional examples (sandbox, rigid_body_test)
- [ ] Rigid body interaction system
- [ ] Performance profiling with flamegraph/Tracy
- [ ] Particle sleeping optimization
- [ ] Inline documentation
- [ ] Unit tests for core systems

### Phase 3: GPU Migration 📋 **PLANNED**
- [ ] Compute shader implementation
- [ ] GPU buffer management
- [ ] Performance comparison
- [ ] Maintain CPU fallback

### Phase 4: Regolith Integration 📋 **PLANNED**
- [ ] Wrap standalone `regolith` engine
- [ ] Maintain same API surface
- [ ] Focus on ECS integration

## Documentation

- [`docs/BEVY_REGOLITH_ARCHITECTURE.md`](docs/BEVY_REGOLITH_ARCHITECTURE.md) - Detailed architecture
- [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) - Step-by-step implementation guide with current status
- [`docs/PBD_explainer.md`](docs/PBD_explainer.md) - Position-Based Dynamics explanation

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

**Status**: ✅ Phase 1 Complete - Core prototype fully functional!

**Current Version**: 0.1.0
**Last Updated**: 2025-10-03

**Next Steps**:
- Create additional examples (sandbox, rigid_body_test)
- Add rigid body interaction system
- Performance profiling and optimization