# Bevy Regolith - Architecture Document

## Overview

`bevy_regolith` is a Bevy plugin for prototyping granular physics simulations using Position-Based Dynamics (PBD). This project serves as:

1. **Prototype Platform**: Test PBD algorithms and rigid body interactions on CPU
2. **Integration Layer**: Eventually wrap the standalone `regolith` GPU engine
3. **Development Tool**: Visualize and tune granular physics parameters

## Design Philosophy

### Core Principles

1. **Prototype First**: Start with CPU implementation for easier debugging
2. **Bevy-Native**: Use ECS patterns and Bevy's built-in systems
3. **Migration Path**: Design for eventual GPU compute shader migration
4. **Visual Feedback**: Rich debug visualization for understanding behavior
5. **Interactive**: Runtime parameter tuning and particle spawning

### Development Phases

**Phase 1: CPU Prototype (Current)**
- Implement PBD on CPU using Bevy ECS
- Focus on correctness and visual feedback
- Test rigid body interaction patterns
- Establish API patterns for future GPU version

**Phase 2: GPU Migration**
- Move particle update to compute shaders
- Keep ECS for high-level coordination
- Maintain same API surface

**Phase 3: Regolith Integration**
- Replace custom implementation with `regolith` crate
- Provide thin Bevy wrapper over regolith API
- Focus on ECS integration and rendering

## Project Structure

```
bevy_regolith/
├── Cargo.toml
├── LICENSE-MIT
├── LICENSE-APACHE
├── README.md
├── BEVY_REGOLITH_ARCHITECTURE.md    # This file
├── src/
│   ├── lib.rs                        # Plugin and public API
│   ├── particle.rs                   # Particle components and data
│   ├── material.rs                   # Material definitions
│   ├── spawner.rs                    # Simple particle spawning
│   ├── solver.rs                     # PBD solver system
│   ├── spatial.rs                    # Spatial hashing for neighbors
│   ├── collision.rs                  # Collision detection and response
│   ├── rigid_body.rs                 # Rigid body interaction
│   ├── rendering.rs                  # Particle visualization
│   ├── camera.rs                     # Orbit camera controller
│   ├── ui.rs                         # Debug UI and parameter tuning
│   └── debug.rs                      # Debug visualization (gizmos)
└── examples/
    ├── basic.rs                      # Minimal example
    ├── sandbox.rs                    # Interactive sandbox
    └── rigid_body_test.rs            # Rigid body interaction demo
```

## Core Data Structures

### Particle Components

```rust
/// Particle position in world space
#[derive(Component)]
pub struct ParticlePosition(pub Vec3);

/// Particle velocity
#[derive(Component)]
pub struct ParticleVelocity(pub Vec3);

/// Previous position for PBD
#[derive(Component)]
pub struct ParticlePrevPosition(pub Vec3);

/// Particle mass
#[derive(Component)]
pub struct ParticleMass(pub f32);

/// Material reference
#[derive(Component)]
pub struct ParticleMaterial(pub MaterialId);

/// Particle radius for collision
#[derive(Component)]
pub struct ParticleRadius(pub f32);

/// Marker component for active particles
#[derive(Component)]
pub struct ActiveParticle;
```

### Material System

```rust
/// Material properties for granular media
#[derive(Clone, Copy, Debug)]
pub struct Material {
    /// Density in kg/m³
    pub density: f32,
    
    /// Friction coefficient
    pub friction: f32,
    
    /// Coefficient of restitution (bounciness)
    pub restitution: f32,
    
    /// Particle radius in meters
    pub particle_radius: f32,
    
    /// Visual color
    pub color: Color,
}

impl Material {
    pub fn lunar_regolith() -> Self {
        Self {
            density: 1500.0,      // kg/m³
            friction: 0.8,        // High friction
            restitution: 0.1,     // Low bounce
            particle_radius: 0.02, // 2cm particles
            color: Color::rgb(0.7, 0.65, 0.6), // Gray-tan
        }
    }
    
    pub fn sand() -> Self {
        Self {
            density: 1600.0,
            friction: 0.6,
            restitution: 0.2,
            particle_radius: 0.02,
            color: Color::rgb(0.9, 0.8, 0.6),
        }
    }
}

/// Resource storing all materials
#[derive(Resource)]
pub struct MaterialRegistry {
    materials: Vec<Material>,
}

pub type MaterialId = usize;
```

### Spatial Hashing

```rust
/// Spatial hash grid for efficient neighbor queries
#[derive(Resource)]
pub struct SpatialHash {
    /// Grid cell size (typically 2-3x particle radius)
    cell_size: f32,
    
    /// Hash map from cell coordinates to particle entities
    grid: HashMap<IVec3, Vec<Entity>>,
    
    /// Bounds of the simulation domain
    bounds: Aabb,
}

impl SpatialHash {
    /// Insert particle into grid
    pub fn insert(&mut self, entity: Entity, position: Vec3);
    
    /// Query neighbors within radius
    pub fn query_neighbors(&self, position: Vec3, radius: f32) -> Vec<Entity>;
    
    /// Clear and rebuild grid
    pub fn rebuild(&mut self, particles: Query<(Entity, &ParticlePosition)>);
}
```

### PBD Solver Configuration

```rust
/// PBD solver parameters
#[derive(Resource)]
pub struct SolverConfig {
    /// Gravity vector
    pub gravity: Vec3,
    
    /// Number of constraint solver iterations
    pub iterations: u32,
    
    /// Time step (typically 1/60)
    pub dt: f32,
    
    /// Damping factor [0-1]
    pub damping: f32,
    
    /// Enable/disable collision detection
    pub enable_collisions: bool,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            iterations: 5,
            dt: 1.0 / 60.0,
            damping: 0.01,
            enable_collisions: true,
        }
    }
}
```

## System Architecture

### System Execution Order

```rust
impl Plugin for RegolithPlugin {
    fn build(&self, app: &mut App) {
        app
            .init_resource::<SolverConfig>()
            .init_resource::<MaterialRegistry>()
            .init_resource::<SpatialHash>()
            .add_systems(Startup, setup_materials)
            .add_systems(Update, (
                // Input handling
                handle_spawn_input,
                orbit_camera_system,
                
                // Physics (in FixedUpdate for determinism)
            ).chain())
            .add_systems(FixedUpdate, (
                // PBD solver pipeline
                predict_positions,
                rebuild_spatial_hash,
                solve_constraints.run_if(|config: Res<SolverConfig>| config.enable_collisions),
                update_velocities,
                apply_damping,
                
                // Rigid body interaction
                compute_particle_forces_on_rigid_bodies,
            ).chain())
            .add_systems(Update, (
                // Rendering and debug
                render_particles,
                debug_draw_spatial_grid,
                update_ui,
            ).after(FixedUpdate));
    }
}
```

### PBD Solver Pipeline

**1. Predict Positions**
```rust
fn predict_positions(
    mut particles: Query<(
        &ParticlePosition,
        &mut ParticlePrevPosition,
        &mut ParticleVelocity,
        &ParticleMass,
    )>,
    config: Res<SolverConfig>,
) {
    for (pos, mut prev_pos, mut vel, mass) in particles.iter_mut() {
        // Store current position
        prev_pos.0 = pos.0;
        
        // Apply external forces (gravity)
        vel.0 += config.gravity * config.dt;
        
        // Predict new position
        pos.0 += vel.0 * config.dt;
    }
}
```

**2. Rebuild Spatial Hash**
```rust
fn rebuild_spatial_hash(
    mut spatial_hash: ResMut<SpatialHash>,
    particles: Query<(Entity, &ParticlePosition), With<ActiveParticle>>,
) {
    spatial_hash.rebuild(particles);
}
```

**3. Solve Constraints**
```rust
fn solve_constraints(
    mut particles: Query<(
        Entity,
        &mut ParticlePosition,
        &ParticleRadius,
        &ParticleMaterial,
    )>,
    spatial_hash: Res<SpatialHash>,
    materials: Res<MaterialRegistry>,
    config: Res<SolverConfig>,
) {
    for _ in 0..config.iterations {
        // Particle-particle collision constraints
        for (entity, mut pos, radius, mat_id) in particles.iter_mut() {
            let neighbors = spatial_hash.query_neighbors(pos.0, radius.0 * 3.0);
            
            for neighbor in neighbors {
                if neighbor == entity { continue; }
                
                // Solve collision constraint
                // ... (detailed implementation below)
            }
        }
        
        // Ground plane constraint
        for (_, mut pos, radius, _) in particles.iter_mut() {
            if pos.0.y < radius.0 {
                pos.0.y = radius.0;
            }
        }
    }
}
```

**4. Update Velocities**
```rust
fn update_velocities(
    mut particles: Query<(
        &ParticlePosition,
        &ParticlePrevPosition,
        &mut ParticleVelocity,
    )>,
    config: Res<SolverConfig>,
) {
    for (pos, prev_pos, mut vel) in particles.iter_mut() {
        // Velocity from position change
        vel.0 = (pos.0 - prev_pos.0) / config.dt;
    }
}
```

## Collision Detection

### Particle-Particle Collision

```rust
fn solve_particle_collision(
    pos_a: &mut Vec3,
    pos_b: &mut Vec3,
    radius_a: f32,
    radius_b: f32,
    mass_a: f32,
    mass_b: f32,
) {
    let delta = *pos_b - *pos_a;
    let dist = delta.length();
    let min_dist = radius_a + radius_b;
    
    if dist < min_dist && dist > 0.0001 {
        let correction = (min_dist - dist) / dist;
        let dir = delta.normalize();
        
        // Mass-weighted correction
        let total_mass = mass_a + mass_b;
        let correction_a = dir * correction * (mass_b / total_mass);
        let correction_b = dir * correction * (mass_a / total_mass);
        
        *pos_a -= correction_a;
        *pos_b += correction_b;
    }
}
```

### Ground Plane Collision

```rust
fn solve_ground_collision(
    pos: &mut Vec3,
    radius: f32,
    restitution: f32,
) {
    if pos.y < radius {
        pos.y = radius;
        // Apply restitution in velocity update
    }
}
```

## Rigid Body Interaction

### Force Computation

```rust
#[derive(Component)]
pub struct RigidBodyForces {
    pub force: Vec3,
    pub torque: Vec3,
}

fn compute_particle_forces_on_rigid_bodies(
    particles: Query<(&ParticlePosition, &ParticleVelocity, &ParticleMass)>,
    mut rigid_bodies: Query<(&Transform, &mut RigidBodyForces)>,
    spatial_hash: Res<SpatialHash>,
) {
    // Reset forces
    for (_, mut forces) in rigid_bodies.iter_mut() {
        forces.force = Vec3::ZERO;
        forces.torque = Vec3::ZERO;
    }
    
    // Accumulate forces from particle collisions
    for (rb_transform, mut forces) in rigid_bodies.iter_mut() {
        let nearby = spatial_hash.query_neighbors(
            rb_transform.translation,
            5.0, // Query radius
        );
        
        for particle_entity in nearby {
            if let Ok((pos, vel, mass)) = particles.get(particle_entity) {
                // Check collision with rigid body
                // Compute contact force
                // Accumulate force and torque
            }
        }
    }
}
```

## Rendering

### Instanced Mesh Rendering

```rust
fn render_particles(
    mut commands: Commands,
    particles: Query<(Entity, &ParticlePosition, &ParticleMaterial), Added<ActiveParticle>>,
    materials: Res<MaterialRegistry>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials_assets: ResMut<Assets<StandardMaterial>>,
) {
    // Create sphere mesh once
    let sphere_mesh = meshes.add(Sphere::new(1.0).mesh().ico(2).unwrap());
    
    for (entity, pos, mat_id) in particles.iter() {
        let material = &materials.materials[mat_id.0];
        
        commands.entity(entity).insert(PbrBundle {
            mesh: sphere_mesh.clone(),
            material: materials_assets.add(StandardMaterial {
                base_color: material.color,
                ..default()
            }),
            transform: Transform::from_translation(pos.0)
                .with_scale(Vec3::splat(material.particle_radius)),
            ..default()
        });
    }
}
```

## Camera System

### Orbit Camera

```rust
#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
}

fn orbit_camera_system(
    mut camera: Query<(&mut Transform, &mut OrbitCamera)>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: EventReader<MouseMotion>,
    mouse_wheel: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    // Mouse drag to rotate
    // Mouse wheel to zoom
    // WASD to pan focus point
}
```

## Debug Visualization

### Spatial Grid Visualization

```rust
fn debug_draw_spatial_grid(
    mut gizmos: Gizmos,
    spatial_hash: Res<SpatialHash>,
    config: Res<SolverConfig>,
) {
    if !config.debug_draw_grid {
        return;
    }
    
    // Draw grid cells that contain particles
    for (cell_coord, entities) in spatial_hash.grid.iter() {
        if !entities.is_empty() {
            let cell_center = cell_coord.as_vec3() * spatial_hash.cell_size;
            gizmos.cuboid(
                Transform::from_translation(cell_center)
                    .with_scale(Vec3::splat(spatial_hash.cell_size)),
                Color::rgba(0.0, 1.0, 0.0, 0.2),
            );
        }
    }
}
```

## UI System

### Parameter Tuning UI

```rust
fn update_ui(
    mut contexts: EguiContexts,
    mut config: ResMut<SolverConfig>,
    particle_count: Query<&ActiveParticle>,
) {
    egui::Window::new("Regolith Controls").show(contexts.ctx_mut(), |ui| {
        ui.heading("Solver Parameters");
        
        ui.add(egui::Slider::new(&mut config.gravity.y, -20.0..=0.0)
            .text("Gravity"));
        
        ui.add(egui::Slider::new(&mut config.iterations, 1..=20)
            .text("Iterations"));
        
        ui.add(egui::Slider::new(&mut config.damping, 0.0..=1.0)
            .text("Damping"));
        
        ui.separator();
        ui.label(format!("Active Particles: {}", particle_count.iter().count()));
        
        if ui.button("Spawn 100 Particles").clicked() {
            // Trigger spawn event
        }
        
        if ui.button("Clear All").clicked() {
            // Trigger clear event
        }
    });
}
```

## Performance Considerations

### CPU Optimization Strategies

1. **Spatial Hashing**: O(n) neighbor queries instead of O(n²)
2. **Parallel Iteration**: Use Bevy's parallel queries where possible
3. **Particle Sleeping**: Mark stationary particles as inactive
4. **Broad Phase Culling**: Skip particles outside active region
5. **Fixed Time Step**: Use `FixedUpdate` for deterministic physics

### Expected Performance (CPU)

- **Target**: 5,000-10,000 particles @ 60 FPS
- **Bottleneck**: Constraint solving (O(n × neighbors × iterations))
- **Profiling**: Use `bevy_framepace` and Tracy profiler

### GPU Migration Path

When moving to GPU compute shaders:

1. **Keep ECS Structure**: Entities still exist, but data lives in GPU buffers
2. **Sync Points**: Copy particle data to/from GPU each frame
3. **Shader Pipeline**: Implement same PBD steps as compute shaders
4. **Rendering**: Use indirect draw calls for instanced rendering

## Integration with Standalone Regolith

### Phase 3: Regolith Wrapper

```rust
// Future API when integrating with regolith crate
pub struct RegolithPlugin {
    solver: regolith::Solver,
}

impl Plugin for RegolithPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_systems(FixedUpdate, (
                sync_bevy_to_regolith,
                step_regolith_solver,
                sync_regolith_to_bevy,
            ).chain());
    }
}

fn step_regolith_solver(
    mut solver: ResMut<regolith::Solver>,
    time: Res<Time>,
) {
    solver.step(time.delta_seconds());
}
```

## Examples

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

fn setup(
    mut commands: Commands,
    mut materials: ResMut<MaterialRegistry>,
) {
    // Add camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 5.0, 10.0)
                .looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        OrbitCamera {
            focus: Vec3::ZERO,
            distance: 10.0,
            yaw: 0.0,
            pitch: 0.3,
        },
    ));
    
    // Add light
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            -0.5,
            0.5,
            0.0,
        )),
        ..default()
    });
    
    // Spawn initial particles
    let regolith_mat = materials.add(Material::lunar_regolith());
    
    for x in -5..5 {
        for z in -5..5 {
            for y in 0..3 {
                spawn_particle(
                    &mut commands,
                    Vec3::new(x as f32 * 0.1, y as f32 * 0.1 + 5.0, z as f32 * 0.1),
                    regolith_mat,
                );
            }
        }
    }
}
```

## Success Metrics

**Technical Goals:**
- 5,000+ particles @ 60 FPS on CPU
- Stable particle stacking (angle of repose)
- Smooth rigid body interaction
- Sub-5ms physics update time

**Development Goals:**
- Clean API for particle spawning
- Intuitive parameter tuning
- Rich debug visualization
- Clear migration path to GPU

## References

- **Bevy ECS**: https://bevyengine.org/learn/book/
- **PBD Paper**: Müller et al. "Position Based Dynamics" (2007)
- **Regolith Architecture**: See `../regolith/ARCHITECTURE.md`
- **Spatial Hashing**: Teschner et al. "Optimized Spatial Hashing" (2003)