# Bevy Regolith - Implementation Plan

This document provides a step-by-step implementation guide for building the `bevy_regolith` prototype.

## Prerequisites

- Rust 1.75+ installed
- Basic understanding of Bevy ECS
- Familiarity with Position-Based Dynamics (optional but helpful)

## Phase 1: Project Setup

### Step 1.1: Create Cargo.toml

```toml
[package]
name = "bevy_regolith"
version = "0.1.0"
edition = "2021"
authors = ["Kyle Matthew Johnson <wilelabs@gmail.com>"]
license = "MIT OR Apache-2.0"
description = "Bevy plugin for granular physics simulation using Position-Based Dynamics"
repository = "https://github.com/RegolithEngine/bevy_regolith"
keywords = ["bevy", "physics", "particles", "granular", "pbd"]
categories = ["game-engines", "simulation"]

[dependencies]
# Bevy 0.17.0
bevy = { version = "0.17", default-features = false, features = [
    "bevy_asset",
    "bevy_winit",
    "bevy_core_pipeline",
    "bevy_pbr",
    "bevy_render",
    "bevy_sprite",
    "bevy_text",
    "bevy_ui",
    "x11",  # Linux window support
] }

# Math and utilities
glam = "0.27"

# UI for parameter tuning
bevy_egui = "0.31"

[dev-dependencies]
# For profiling
bevy_framepace = "0.17"

[[example]]
name = "basic"
path = "examples/basic.rs"

[[example]]
name = "sandbox"
path = "examples/sandbox.rs"

[[example]]
name = "rigid_body_test"
path = "examples/rigid_body_test.rs"

[profile.dev]
opt-level = 1  # Better performance in dev mode

[profile.dev.package."*"]
opt-level = 3  # Optimize dependencies

[profile.release]
lto = "thin"
codegen-units = 1
```

### Step 1.2: Create Directory Structure

```bash
mkdir -p src examples
touch src/lib.rs
touch src/particle.rs
touch src/material.rs
touch src/spawner.rs
touch src/solver.rs
touch src/spatial.rs
touch src/collision.rs
touch src/rigid_body.rs
touch src/rendering.rs
touch src/camera.rs
touch src/ui.rs
touch src/debug.rs
touch examples/basic.rs
touch examples/sandbox.rs
touch examples/rigid_body_test.rs
```

## Phase 2: Core Data Structures

### Step 2.1: Particle Components (src/particle.rs)

```rust
use bevy::prelude::*;

/// Particle position in world space
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticlePosition(pub Vec3);

/// Particle velocity
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleVelocity(pub Vec3);

/// Previous position for PBD solver
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticlePrevPosition(pub Vec3);

/// Particle mass in kg
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleMass(pub f32);

/// Reference to material properties
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleMaterial(pub usize);

/// Particle radius for collision detection
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleRadius(pub f32);

/// Marker component for active particles
#[derive(Component, Debug, Clone, Copy)]
pub struct ActiveParticle;

/// Bundle for spawning particles
#[derive(Bundle)]
pub struct ParticleBundle {
    pub position: ParticlePosition,
    pub velocity: ParticleVelocity,
    pub prev_position: ParticlePrevPosition,
    pub mass: ParticleMass,
    pub material: ParticleMaterial,
    pub radius: ParticleRadius,
    pub active: ActiveParticle,
}

impl ParticleBundle {
    pub fn new(position: Vec3, material_id: usize, radius: f32, mass: f32) -> Self {
        Self {
            position: ParticlePosition(position),
            velocity: ParticleVelocity(Vec3::ZERO),
            prev_position: ParticlePrevPosition(position),
            mass: ParticleMass(mass),
            material: ParticleMaterial(material_id),
            radius: ParticleRadius(radius),
            active: ActiveParticle,
        }
    }
}
```

### Step 2.2: Material System (src/material.rs)

```rust
use bevy::prelude::*;

/// Material properties for granular media
#[derive(Clone, Copy, Debug)]
pub struct Material {
    /// Density in kg/m³
    pub density: f32,
    
    /// Friction coefficient [0-1]
    pub friction: f32,
    
    /// Coefficient of restitution (bounciness) [0-1]
    pub restitution: f32,
    
    /// Particle radius in meters
    pub particle_radius: f32,
    
    /// Visual color
    pub color: Color,
}

impl Material {
    /// Lunar regolith preset
    pub fn lunar_regolith() -> Self {
        Self {
            density: 1500.0,
            friction: 0.8,
            restitution: 0.1,
            particle_radius: 0.02,
            color: Color::srgb(0.7, 0.65, 0.6),
        }
    }
    
    /// Dry sand preset
    pub fn sand() -> Self {
        Self {
            density: 1600.0,
            friction: 0.6,
            restitution: 0.2,
            particle_radius: 0.02,
            color: Color::srgb(0.9, 0.8, 0.6),
        }
    }
    
    /// Snow preset
    pub fn snow() -> Self {
        Self {
            density: 400.0,
            friction: 0.4,
            restitution: 0.1,
            particle_radius: 0.015,
            color: Color::srgb(0.95, 0.95, 1.0),
        }
    }
}

/// Resource storing all materials
#[derive(Resource, Default)]
pub struct MaterialRegistry {
    pub materials: Vec<Material>,
}

impl MaterialRegistry {
    pub fn new() -> Self {
        Self {
            materials: Vec::new(),
        }
    }
    
    pub fn add(&mut self, material: Material) -> usize {
        let id = self.materials.len();
        self.materials.push(material);
        id
    }
    
    pub fn get(&self, id: usize) -> Option<&Material> {
        self.materials.get(id)
    }
}
```

### Step 2.3: Spatial Hash (src/spatial.rs)

```rust
use bevy::prelude::*;
use std::collections::HashMap;

/// Spatial hash grid for efficient neighbor queries
#[derive(Resource)]
pub struct SpatialHash {
    /// Grid cell size (typically 2-3x particle radius)
    pub cell_size: f32,
    
    /// Hash map from cell coordinates to particle entities
    grid: HashMap<IVec3, Vec<Entity>>,
    
    /// Bounds of the simulation domain
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}

impl SpatialHash {
    pub fn new(cell_size: f32, bounds_min: Vec3, bounds_max: Vec3) -> Self {
        Self {
            cell_size,
            grid: HashMap::new(),
            bounds_min,
            bounds_max,
        }
    }
    
    /// Convert world position to grid cell coordinates
    fn world_to_cell(&self, position: Vec3) -> IVec3 {
        ((position - self.bounds_min) / self.cell_size).as_ivec3()
    }
    
    /// Clear and rebuild the spatial hash grid
    pub fn rebuild(&mut self, particles: &Query<(Entity, &crate::particle::ParticlePosition)>) {
        self.grid.clear();
        
        for (entity, pos) in particles.iter() {
            let cell = self.world_to_cell(pos.0);
            self.grid.entry(cell).or_insert_with(Vec::new).push(entity);
        }
    }
    
    /// Query neighbors within a radius
    pub fn query_neighbors(&self, position: Vec3, radius: f32) -> Vec<Entity> {
        let mut neighbors = Vec::new();
        let cell = self.world_to_cell(position);
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        
        // Check neighboring cells
        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                for dz in -cell_radius..=cell_radius {
                    let neighbor_cell = cell + IVec3::new(dx, dy, dz);
                    
                    if let Some(entities) = self.grid.get(&neighbor_cell) {
                        neighbors.extend(entities.iter().copied());
                    }
                }
            }
        }
        
        neighbors
    }
    
    /// Get all occupied cells (for debug visualization)
    pub fn occupied_cells(&self) -> impl Iterator<Item = (&IVec3, &Vec<Entity>)> {
        self.grid.iter()
    }
}

impl Default for SpatialHash {
    fn default() -> Self {
        Self::new(
            0.1,  // 10cm cells
            Vec3::new(-10.0, -2.0, -10.0),
            Vec3::new(10.0, 15.0, 10.0),
        )
    }
}
```

### Step 2.4: Solver Configuration (src/solver.rs - Part 1)

```rust
use bevy::prelude::*;

/// PBD solver configuration
#[derive(Resource, Clone)]
pub struct SolverConfig {
    /// Gravity vector (m/s²)
    pub gravity: Vec3,
    
    /// Number of constraint solver iterations
    pub iterations: u32,
    
    /// Time step (seconds)
    pub dt: f32,
    
    /// Velocity damping [0-1]
    pub damping: f32,
    
    /// Enable collision detection
    pub enable_collisions: bool,
    
    /// Enable debug visualization
    pub debug_draw_grid: bool,
    pub debug_draw_velocities: bool,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            iterations: 5,
            dt: 1.0 / 60.0,
            damping: 0.01,
            enable_collisions: true,
            debug_draw_grid: false,
            debug_draw_velocities: false,
        }
    }
}
```

## Phase 3: PBD Solver Implementation

### Step 3.1: Predict Positions (src/solver.rs - Part 2)

```rust
use crate::particle::*;

/// System: Predict new positions based on velocity and external forces
pub fn predict_positions(
    mut particles: Query<(
        &mut ParticlePosition,
        &mut ParticlePrevPosition,
        &mut ParticleVelocity,
    )>,
    config: Res<SolverConfig>,
) {
    for (mut pos, mut prev_pos, mut vel) in particles.iter_mut() {
        // Store current position
        prev_pos.0 = pos.0;
        
        // Apply gravity
        vel.0 += config.gravity * config.dt;
        
        // Predict new position
        pos.0 += vel.0 * config.dt;
    }
}
```

### Step 3.2: Rebuild Spatial Hash (src/solver.rs - Part 3)

```rust
use crate::spatial::SpatialHash;

/// System: Rebuild spatial hash grid for efficient neighbor queries
pub fn rebuild_spatial_hash(
    mut spatial_hash: ResMut<SpatialHash>,
    particles: Query<(Entity, &ParticlePosition), With<ActiveParticle>>,
) {
    spatial_hash.rebuild(&particles);
}
```

### Step 3.3: Solve Constraints (src/collision.rs)

```rust
use bevy::prelude::*;
use crate::particle::*;
use crate::spatial::SpatialHash;
use crate::material::MaterialRegistry;
use crate::solver::SolverConfig;

/// System: Solve particle-particle and particle-ground collision constraints
pub fn solve_constraints(
    mut particles: Query<(
        Entity,
        &mut ParticlePosition,
        &ParticleRadius,
        &ParticleMass,
        &ParticleMaterial,
    )>,
    spatial_hash: Res<SpatialHash>,
    materials: Res<MaterialRegistry>,
    config: Res<SolverConfig>,
) {
    // Iterate constraint solver
    for _ in 0..config.iterations {
        // Collect position updates to avoid borrow checker issues
        let mut position_updates: Vec<(Entity, Vec3)> = Vec::new();
        
        // Particle-particle collisions
        for (entity, pos, radius, mass, mat_id) in particles.iter() {
            let neighbors = spatial_hash.query_neighbors(pos.0, radius.0 * 3.0);
            let mut correction = Vec3::ZERO;
            
            for neighbor_entity in neighbors {
                if neighbor_entity == entity {
                    continue;
                }
                
                if let Ok((_, neighbor_pos, neighbor_radius, neighbor_mass, _)) = 
                    particles.get(neighbor_entity) 
                {
                    let delta = neighbor_pos.0 - pos.0;
                    let dist = delta.length();
                    let min_dist = radius.0 + neighbor_radius.0;
                    
                    if dist < min_dist && dist > 0.0001 {
                        let penetration = min_dist - dist;
                        let dir = delta / dist;
                        
                        // Mass-weighted correction
                        let total_mass = mass.0 + neighbor_mass.0;
                        let weight = neighbor_mass.0 / total_mass;
                        correction -= dir * penetration * weight * 0.5;
                    }
                }
            }
            
            position_updates.push((entity, correction));
        }
        
        // Apply position updates
        for (entity, correction) in position_updates {
            if let Ok((_, mut pos, _, _, _)) = particles.get_mut(entity) {
                pos.0 += correction;
            }
        }
        
        // Ground plane collision
        for (_, mut pos, radius, _, mat_id) in particles.iter_mut() {
            if pos.0.y < radius.0 {
                pos.0.y = radius.0;
            }
        }
    }
}
```

### Step 3.4: Update Velocities (src/solver.rs - Part 4)

```rust
/// System: Update velocities from position changes
pub fn update_velocities(
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

/// System: Apply velocity damping
pub fn apply_damping(
    mut particles: Query<&mut ParticleVelocity>,
    config: Res<SolverConfig>,
) {
    let damping_factor = 1.0 - config.damping;
    
    for mut vel in particles.iter_mut() {
        vel.0 *= damping_factor;
    }
}
```

## Phase 4: Particle Spawning

### Step 4.1: Simple Spawner (src/spawner.rs)

```rust
use bevy::prelude::*;
use crate::particle::*;
use crate::material::MaterialRegistry;

/// System: Spawn particles on key press
pub fn handle_spawn_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    materials: Res<MaterialRegistry>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        spawn_particle_cluster(&mut commands, Vec3::new(0.0, 5.0, 0.0), 0, 100);
    }
    
    if keyboard.just_pressed(KeyCode::KeyR) {
        spawn_particle_cluster(&mut commands, Vec3::new(0.0, 8.0, 0.0), 0, 500);
    }
}

/// Helper: Spawn a cluster of particles
pub fn spawn_particle_cluster(
    commands: &mut Commands,
    center: Vec3,
    material_id: usize,
    count: usize,
) {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    
    for _ in 0..count {
        let offset = Vec3::new(
            rng.gen_range(-0.5..0.5),
            rng.gen_range(-0.5..0.5),
            rng.gen_range(-0.5..0.5),
        );
        
        let position = center + offset;
        
        commands.spawn(ParticleBundle::new(
            position,
            material_id,
            0.02,  // radius
            0.001, // mass
        ));
    }
}

/// System: Setup initial particles
pub fn setup_initial_particles(
    mut commands: Commands,
    materials: Res<MaterialRegistry>,
) {
    // Spawn a pile of particles
    for x in -5..5 {
        for z in -5..5 {
            for y in 0..3 {
                let position = Vec3::new(
                    x as f32 * 0.05,
                    y as f32 * 0.05 + 2.0,
                    z as f32 * 0.05,
                );
                
                commands.spawn(ParticleBundle::new(
                    position,
                    0, // material_id
                    0.02,
                    0.001,
                ));
            }
        }
    }
}
```

## Phase 5: Rendering

### Step 5.1: Particle Rendering (src/rendering.rs)

```rust
use bevy::prelude::*;
use crate::particle::*;
use crate::material::MaterialRegistry;

/// System: Add visual representation to newly spawned particles
pub fn setup_particle_rendering(
    mut commands: Commands,
    particles: Query<(Entity, &ParticlePosition, &ParticleRadius, &ParticleMaterial), Added<ActiveParticle>>,
    materials: Res<MaterialRegistry>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create sphere mesh (reuse for all particles)
    let sphere_mesh = meshes.add(Sphere::new(1.0).mesh().ico(2).unwrap());
    
    for (entity, pos, radius, mat_id) in particles.iter() {
        let material = materials.get(mat_id.0).unwrap();
        
        let material_handle = standard_materials.add(StandardMaterial {
            base_color: material.color,
            perceptual_roughness: 0.8,
            metallic: 0.0,
            ..default()
        });
        
        commands.entity(entity).insert(PbrBundle {
            mesh: sphere_mesh.clone(),
            material: material_handle,
            transform: Transform::from_translation(pos.0)
                .with_scale(Vec3::splat(radius.0)),
            ..default()
        });
    }
}

/// System: Update particle visual transforms
pub fn update_particle_transforms(
    mut particles: Query<(&ParticlePosition, &mut Transform), With<ActiveParticle>>,
) {
    for (pos, mut transform) in particles.iter_mut() {
        transform.translation = pos.0;
    }
}
```

## Phase 6: Camera System

### Step 6.1: Orbit Camera (src/camera.rs)

```rust
use bevy::prelude::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};

#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub sensitivity: f32,
    pub zoom_speed: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            focus: Vec3::ZERO,
            distance: 10.0,
            yaw: 0.0,
            pitch: 0.3,
            sensitivity: 0.003,
            zoom_speed: 0.5,
        }
    }
}

/// System: Handle orbit camera controls
pub fn orbit_camera_system(
    mut camera_query: Query<(&mut Transform, &mut OrbitCamera)>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_wheel: EventReader<MouseWheel>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    let (mut transform, mut orbit) = camera_query.single_mut();
    
    // Mouse drag to rotate
    if mouse_button.pressed(MouseButton::Right) {
        for motion in mouse_motion.read() {
            orbit.yaw -= motion.delta.x * orbit.sensitivity;
            orbit.pitch -= motion.delta.y * orbit.sensitivity;
            orbit.pitch = orbit.pitch.clamp(-1.5, 1.5);
        }
    } else {
        mouse_motion.clear();
    }
    
    // Mouse wheel to zoom
    for wheel in mouse_wheel.read() {
        orbit.distance -= wheel.y * orbit.zoom_speed;
        orbit.distance = orbit.distance.clamp(2.0, 50.0);
    }
    
    // WASD to pan focus
    let mut pan = Vec3::ZERO;
    let speed = 5.0 * time.delta_seconds();
    
    if keyboard.pressed(KeyCode::KeyW) { pan.z -= speed; }
    if keyboard.pressed(KeyCode::KeyS) { pan.z += speed; }
    if keyboard.pressed(KeyCode::KeyA) { pan.x -= speed; }
    if keyboard.pressed(KeyCode::KeyD) { pan.x += speed; }
    if keyboard.pressed(KeyCode::KeyQ) { pan.y -= speed; }
    if keyboard.pressed(KeyCode::KeyE) { pan.y += speed; }
    
    // Apply pan in camera space
    let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
    orbit.focus += rotation * pan;
    
    // Update camera transform
    let offset = rotation * Vec3::new(0.0, 0.0, orbit.distance);
    transform.translation = orbit.focus + offset;
    transform.look_at(orbit.focus, Vec3::Y);
}
```

## Phase 7: Plugin Assembly

### Step 7.1: Main Plugin (src/lib.rs)

```rust
use bevy::prelude::*;

pub mod particle;
pub mod material;
pub mod spawner;
pub mod solver;
pub mod spatial;
pub mod collision;
pub mod rendering;
pub mod camera;
pub mod ui;
pub mod debug;

pub mod prelude {
    pub use crate::particle::*;
    pub use crate::material::*;
    pub use crate::spawner::*;
    pub use crate::solver::*;
    pub use crate::camera::*;
    pub use crate::RegolithPlugin;
}

pub struct RegolithPlugin;

impl Plugin for RegolithPlugin {
    fn build(&self, app: &mut App) {
        app
            // Resources
            .init_resource::<solver::SolverConfig>()
            .init_resource::<material::MaterialRegistry>()
            .init_resource::<spatial::SpatialHash>()
            
            // Startup systems
            .add_systems(Startup, (
                setup_materials,
                spawner::setup_initial_particles,
            ).chain())
            
            // Update systems
            .add_systems(Update, (
                spawner::handle_spawn_input,
                camera::orbit_camera_system,
                rendering::setup_particle_rendering,
            ))
            
            // Fixed update (physics)
            .add_systems(FixedUpdate, (
                solver::predict_positions,
                solver::rebuild_spatial_hash,
                collision::solve_constraints,
                solver::update_velocities,
                solver::apply_damping,
            ).chain())
            
            // Late update (after physics)
            .add_systems(Update, (
                rendering::update_particle_transforms,
                debug::debug_draw_system,
            ).after(FixedUpdate));
    }
}

fn setup_materials(mut materials: ResMut<material::MaterialRegistry>) {
    materials.add(material::Material::lunar_regolith());
    materials.add(material::Material::sand());
    materials.add(material::Material::snow());
}
```

## Phase 8: Examples

### Step 8.1: Basic Example (examples/basic.rs)

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
    // Camera
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
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            -0.5,
            0.5,
            0.0,
        )),
        ..default()
    });
    
    // Ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(20.0, 20.0)),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        }),
        ..default()
    });
}
```

## Next Steps

1. **Implement remaining systems**: UI (src/ui.rs) and debug visualization (src/debug.rs)
2. **Add rigid body interaction**: Implement src/rigid_body.rs
3. **Test and profile**: Run examples and measure performance
4. **Optimize**: Identify bottlenecks and optimize hot paths
5. **Document**: Add inline documentation and usage examples

## Testing Strategy

1. **Visual Testing**: Run examples and observe particle behavior
2. **Performance Testing**: Profile with `cargo flamegraph` or Tracy
3. **Correctness Testing**: Verify angle of repose, energy conservation
4. **Stress Testing**: Gradually increase particle count to find limits

## Migration to GPU

When ready to move to GPU compute shaders:

1. Keep ECS structure intact
2. Replace solver systems with compute shader dispatch
3. Use GPU buffers for particle data
4. Sync particle data to/from GPU each frame
5. Reference the standalone `regolith` architecture for shader implementation