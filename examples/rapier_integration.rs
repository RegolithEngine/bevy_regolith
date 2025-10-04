//! Rapier integration example
//!
//! This example demonstrates how to use bevy_regolith particles
//! alongside Rapier physics for rigid body dynamics.
//!
//! Run with: cargo run --example rapier_integration --features rapier
//!
//! Controls:
//! - Space: Spawn 100 particles
//! - R: Spawn 500 particles
//! - C: Clear all particles
//! - Left Mouse: Rotate camera
//! - Right Mouse: Pan camera
//! - Mouse Wheel: Zoom
//! - WASD/QE: Move camera

use bevy::prelude::*;
use bevy_regolith::prelude::*;
use bevy_rapier3d::prelude::{
    Collider, ColliderMassProperties, ExternalForce, Friction, NoUserData,
    RapierDebugRenderPlugin, RapierPhysicsPlugin, Restitution, RigidBody as RapierRigidBody,
    Velocity,
};
use std::collections::HashMap;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Regolith - Rapier Integration".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RegolithPlugin)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .insert_resource(ParticleForceAccumulator::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (
            sync_rapier_to_regolith,
            spawn_particles_input,
            clear_particles_input,
            despawn_out_of_bounds_particles,
        ))
        .add_systems(FixedUpdate, (
            calculate_particle_forces
                .before(bevy_regolith::solver::predict_positions),
            apply_particle_forces
                .after(calculate_particle_forces)
                .before(bevy_rapier3d::plugin::PhysicsSet::StepSimulation),
        ))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera with orbit controls
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 15.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
        OrbitCamera {
            focus: Vec3::new(0.0, 3.0, 0.0),
            distance: 15.0,
            ..default()
        },
    ));
    
    // Directional light (sun)
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            -0.5,
            0.5,
            0.0,
        )),
    ));
    
    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        affects_lightmapped_meshes: false,
    });
    
    // Rapier ground plane (fixed rigid body)
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20.0, 20.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        })),
        RapierRigidBody::Fixed,
        Collider::cuboid(10.0, 0.1, 10.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
        SyncToRegolith,
    ));
    
    // Dynamic Rapier box - will be pushed by particles
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.5, 1.5, 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.3, 0.3),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RapierRigidBody::Dynamic,
        Collider::cuboid(0.75, 0.75, 0.75),
        ColliderMassProperties::Density(2.0),
        Restitution::coefficient(0.3),
        Friction::coefficient(0.5),
        ExternalForce::default(),
        Velocity::default(),
        Transform::from_xyz(-2.0, 3.0, 0.0),
        SyncToRegolith,
    ));
    
    // Dynamic Rapier sphere
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.8))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.82),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RapierRigidBody::Dynamic,
        Collider::ball(0.8),
        ColliderMassProperties::Density(2.0),
        Restitution::coefficient(0.5),
        Friction::coefficient(0.3),
        ExternalForce::default(),
        Velocity::default(),
        Transform::from_xyz(0.0, 1.0, -4.0),
        SyncToRegolith,
    ));
    
    // Static Rapier ramp
    let ramp_rotation = Quat::from_rotation_x(-0.3);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(4.0, 0.2, 3.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RapierRigidBody::Fixed,
        Collider::cuboid(2.0, 0.1, 1.5),
        Transform::from_xyz(0.0, 1.5, -1.5).with_rotation(ramp_rotation),
        SyncToRegolith,
    ));
    
    // Spawn initial particles above the scene
    spawn_particle_pile(&mut commands, Vec3::new(0.0, 7.0, 0.0), 0, 300);
    
    println!("=== Rapier Integration Example ===");
    println!("Red box and blue sphere are controlled by Rapier physics");
    println!("Particles are controlled by bevy_regolith PBD");
    println!("Watch them interact!");
    println!();
    println!("Controls:");
    println!("  Space: Spawn 100 particles");
    println!("  R: Spawn 500 particles");
    println!("  C: Clear all particles");
}

/// Marker component for Rapier-controlled rigid bodies that need syncing
#[derive(Component)]
struct SyncToRegolith;

/// Accumulates forces and torques from particle collisions before applying to Rapier
#[derive(Resource, Default)]
struct ParticleForceAccumulator {
    /// Forces to apply to rigid body centers of mass
    forces: HashMap<Entity, Vec3>,
    
    /// Torques to apply to rigid bodies
    torques: HashMap<Entity, Vec3>,
    
    /// Number of particle contacts per body (for debugging)
    contact_counts: HashMap<Entity, usize>,
}

impl ParticleForceAccumulator {
    fn clear(&mut self) {
        self.forces.clear();
        self.torques.clear();
        self.contact_counts.clear();
    }
    
    fn add_force(&mut self, entity: Entity, force: Vec3, contact_point: Vec3, body_center: Vec3) {
        *self.forces.entry(entity).or_default() += force;
        
        // Calculate torque from contact point
        let r = contact_point - body_center;
        let torque = r.cross(force);
        *self.torques.entry(entity).or_default() += torque;
        
        *self.contact_counts.entry(entity).or_default() += 1;
    }
}

/// Sync Rapier rigid body positions to regolith collision system
/// This allows regolith particles to collide with Rapier bodies
fn sync_rapier_to_regolith(
    mut commands: Commands,
    all_rapier_bodies: Query<(&Transform, &Collider), With<SyncToRegolith>>,
    changed_rapier_bodies: Query<&Transform, (With<SyncToRegolith>, Changed<Transform>)>,
    existing_rigid_bodies: Query<Entity, With<RigidBody>>,
    mut initialized: Local<bool>,
) {
    // Initialize on first run or when any transform changes
    let should_update = !*initialized || !changed_rapier_bodies.is_empty();
    
    if !should_update {
        return;
    }
    
    *initialized = true;
    
    // Remove old rigid body representations
    for entity in existing_rigid_bodies.iter() {
        commands.entity(entity).despawn();
    }
    
    // Create new rigid body representations for regolith collision
    for (transform, collider) in all_rapier_bodies.iter() {
        // Convert Rapier collider to regolith collision shape
        if let Some(cuboid) = collider.as_cuboid() {
            let half_extents = cuboid.half_extents();
            commands.spawn(RigidBodyBundle::static_box(
                transform.translation,
                transform.rotation,
                half_extents.x * 2.0,
                half_extents.y * 2.0,
                half_extents.z * 2.0,
            ));
        } else if let Some(ball) = collider.as_ball() {
            let radius = ball.radius();
            commands.spawn(RigidBodyBundle::static_sphere(
                transform.translation,
                radius,
            ));
        }
    }
}

/// Calculate forces from particle collisions with Rapier bodies
fn calculate_particle_forces(
    particles: Query<(&ParticlePosition, &ParticlePrevPosition, &ParticleRadius, &ParticleMass)>,
    rapier_bodies: Query<(Entity, &Transform, &Collider, &RapierRigidBody, &Velocity), With<SyncToRegolith>>,
    mut force_accumulator: ResMut<ParticleForceAccumulator>,
    time: Res<Time>,
) {
    force_accumulator.clear();
    
    let dt = time.delta_secs();
    if dt < 0.0001 {
        return;
    }
    
    for (particle_pos, particle_prev_pos, particle_radius, particle_mass) in particles.iter() {
        // Calculate particle velocity
        let particle_velocity = (particle_pos.0 - particle_prev_pos.0) / dt;
        
        for (rb_entity, rb_transform, collider, rb_type, rb_velocity) in rapier_bodies.iter() {
            // Only apply forces to dynamic bodies
            if !matches!(rb_type, RapierRigidBody::Dynamic) {
                continue;
            }
            
            // Simple collision detection - check if particle overlaps with collider
            let world_to_local = rb_transform.compute_affine().inverse();
            let local_pos = world_to_local.transform_point3(particle_pos.0);
            
            // Get closest point on collider
            let closest_point = if let Some(cuboid) = collider.as_cuboid() {
                let half_extents = cuboid.half_extents();
                Vec3::new(
                    local_pos.x.clamp(-half_extents.x, half_extents.x),
                    local_pos.y.clamp(-half_extents.y, half_extents.y),
                    local_pos.z.clamp(-half_extents.z, half_extents.z),
                )
            } else if let Some(ball) = collider.as_ball() {
                let radius = ball.radius();
                if local_pos.length() > 0.0001 {
                    local_pos.normalize() * radius
                } else {
                    Vec3::ZERO
                }
            } else {
                continue;
            };
            
            let closest_world = rb_transform.transform_point(closest_point);
            let delta = particle_pos.0 - closest_world;
            let dist = delta.length();
            
            // Check if particle is colliding
            if dist < particle_radius.0 && dist > 0.0001 {
                let penetration = particle_radius.0 - dist;
                let normal = delta / dist;
                
                // Calculate velocity of rigid body at contact point
                let r = closest_world - rb_transform.translation;
                let rb_velocity_at_contact = Vec3::from(rb_velocity.linvel) + Vec3::from(rb_velocity.angvel).cross(r);
                
                // Calculate relative velocity (particle relative to body at contact point)
                let relative_velocity = particle_velocity - rb_velocity_at_contact;
                
                // Project relative velocity onto normal
                let normal_velocity = relative_velocity.dot(normal);
                
                // Debug: print collision info occasionally
                static mut DEBUG_COUNTER: u32 = 0;
                unsafe {
                    DEBUG_COUNTER += 1;
                    if DEBUG_COUNTER % 100 == 0 {
                        println!("Collision detected: normal_vel={}, particle_vel={:?}, rb_vel={:?}",
                            normal_velocity, particle_velocity, rb_velocity_at_contact);
                    }
                }
                
                // Only apply force if particle and body are approaching each other
                if normal_velocity < 0.0 {
                    // Impulse-based force calculation
                    // normal points from body surface toward particle
                    // When particle hits body, body should be pushed AWAY from particle
                    // So force on body is in -normal direction (opposite to relative approach)
                    let impulse_magnitude = particle_mass.0 * normal_velocity.abs();
                    let force = -normal * impulse_magnitude / dt;
                    
                    // Penetration correction force also pushes body away from particle
                    // Tunable force scale parameters:
                    const PENETRATION_FORCE_SCALE: f32 = 10.0;  // Adjust this (10-1000)
                    const TOTAL_FORCE_SCALE: f32 = 5.0;         // Adjust this (1-100)
                    
                    let penetration_force = -normal * penetration * particle_mass.0 * PENETRATION_FORCE_SCALE;
                    let total_force = (force + penetration_force) * TOTAL_FORCE_SCALE;
                    
                    // Apply force at contact point
                    force_accumulator.add_force(
                        rb_entity,
                        total_force,
                        closest_world,
                        rb_transform.translation,
                    );
                }
            }
        }
    }
}

/// Apply accumulated particle forces to Rapier rigid bodies
fn apply_particle_forces(
    force_accumulator: Res<ParticleForceAccumulator>,
    mut external_forces: Query<&mut ExternalForce>,
    all_dynamic_bodies: Query<Entity, With<RapierRigidBody>>,
) {
    for (entity, force) in force_accumulator.forces.iter() {
        match external_forces.get_mut(*entity) {
            Ok(mut ext_force) => {
                // Set the force (don't add, to avoid accumulation issues)
                ext_force.force = *force;
                
                // Apply torque
                if let Some(torque) = force_accumulator.torques.get(entity) {
                    ext_force.torque = *torque;
                }
                
                println!("Applied force {:?} to entity {:?}", force, entity);
            }
            Err(_) => {
                println!("ERROR: Could not apply force to entity {:?} - ExternalForce component missing!", entity);
            }
        }
    }
    
    // Debug: List all dynamic bodies
    static mut PRINTED_BODIES: bool = false;
    unsafe {
        if !PRINTED_BODIES {
            println!("=== All Dynamic Bodies ===");
            for entity in all_dynamic_bodies.iter() {
                let has_ext_force = external_forces.get(entity).is_ok();
                println!("  Entity {:?}: has ExternalForce = {}", entity, has_ext_force);
            }
            PRINTED_BODIES = true;
        }
    }
    
    // Debug output
    if !force_accumulator.forces.is_empty() {
        static mut PRINTED: bool = false;
        unsafe {
            if !PRINTED {
                println!("=== Particle Forces Applied ===");
                for (entity, force) in force_accumulator.forces.iter() {
                    if let Some(count) = force_accumulator.contact_counts.get(entity) {
                        println!("  Entity {:?}: {} contacts, force: {:?}", entity, count, force);
                    }
                }
                PRINTED = true;
            }
        }
    }
}

/// Spawn a pile of particles at a given position
fn spawn_particle_pile(
    commands: &mut Commands,
    center: Vec3,
    material_id: usize,
    count: usize,
) {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    
    for _ in 0..count {
        let offset = Vec3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-0.5..0.5),
            rng.gen_range(-1.0..1.0),
        );
        
        let position = center + offset;
        
        commands.spawn(ParticleBundle::new(
            position,
            material_id,
            0.02,
            0.001,
        ));
    }
}

/// Handle particle spawning input
fn spawn_particles_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        spawn_particle_pile(&mut commands, Vec3::new(0.0, 7.0, 0.0), 0, 100);
        println!("Spawned 100 particles");
    }
    
    if keyboard.just_pressed(KeyCode::KeyR) {
        spawn_particle_pile(&mut commands, Vec3::new(0.0, 7.0, 0.0), 0, 500);
        println!("Spawned 500 particles");
    }
}

/// Handle clearing particles
fn clear_particles_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    particles: Query<Entity, With<ActiveParticle>>,
) {
    if keyboard.just_pressed(KeyCode::KeyC) {
        let count = particles.iter().count();
        for entity in particles.iter() {
            commands.entity(entity).despawn();
        }
        println!("Cleared {} particles", count);
    }
}

/// Despawn particles that fall outside the ground plane boundaries
fn despawn_out_of_bounds_particles(
    mut commands: Commands,
    particles: Query<(Entity, &ParticlePosition), With<ActiveParticle>>,
) {
    const GROUND_SIZE: f32 = 20.0;
    const HALF_GROUND: f32 = GROUND_SIZE / 2.0;
    const MIN_Y: f32 = -2.0; // Despawn if below ground
    
    for (entity, position) in particles.iter() {
        let pos = position.0;
        
        // Check if particle is outside bounds
        if pos.y < MIN_Y
            || pos.x < -HALF_GROUND || pos.x > HALF_GROUND
            || pos.z < -HALF_GROUND || pos.z > HALF_GROUND
        {
            commands.entity(entity).despawn();
        }
    }
}