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
    Collider, ColliderMassProperties, Friction, NoUserData, RapierDebugRenderPlugin,
    RapierPhysicsPlugin, Restitution, RigidBody as RapierRigidBody,
};

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
        .add_systems(Startup, setup)
        .add_systems(Update, (
            sync_rapier_to_regolith,
            spawn_particles_input,
            clear_particles_input,
            despawn_out_of_bounds_particles,
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
        Transform::from_xyz(-2.0, 3.0, 0.0),
        SyncToRegolith,
    ));
    
    // Dynamic Rapier sphere
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.8))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.8),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RapierRigidBody::Dynamic,
        Collider::ball(0.8),
        ColliderMassProperties::Density(2.0),
        Restitution::coefficient(0.5),
        Friction::coefficient(0.3),
        Transform::from_xyz(1.0, 1.0, 0.0),
        SyncToRegolith,
    ));
    
    // // Static Rapier ramp
    // let ramp_rotation = Quat::from_rotation_x(-0.3);
    // commands.spawn((
    //     Mesh3d(meshes.add(Cuboid::new(4.0, 0.2, 3.0))),
    //     MeshMaterial3d(materials.add(StandardMaterial {
    //         base_color: Color::srgb(0.5, 0.5, 0.5),
    //         perceptual_roughness: 0.8,
    //         metallic: 0.1,
    //         ..default()
    //     })),
    //     RapierRigidBody::Fixed,
    //     Collider::cuboid(2.0, 0.1, 1.5),
    //     Transform::from_xyz(0.0, 1.5, -1.5).with_rotation(ramp_rotation),
    //     SyncToRegolith,
    // ));
    
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