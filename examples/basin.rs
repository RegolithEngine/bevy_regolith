//! Basin example demonstrating particle stacking behavior
//!
//! This example creates a basin (bowl-like structure) where particles fall in
//! and stack on top of each other, allowing you to observe collision and
//! stacking interactions.
//!
//! Controls:
//! - Space: Spawn 50 particles above the basin
//! - R: Spawn 200 particles above the basin
//! - C: Clear all particles
//! - Left Mouse: Rotate camera
//! - Right Mouse: Pan camera
//! - Mouse Wheel: Zoom
//! - WASD/QE: Move camera

use bevy::prelude::*;
use bevy_regolith::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Regolith - Basin Stacking Demo".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RegolithPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (
            handle_spawn_input,
            handle_clear_input,
        ))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera positioned to view the basin from above and to the side
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(4.0, 6.0, 8.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
        OrbitCamera {
            focus: Vec3::new(0.0, 1.0, 0.0),
            distance: 10.0,
            yaw: 0.8,
            pitch: -0.6,
            sensitivity: 0.003,
            zoom_speed: 0.5,
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
    
    // Ambient light for better visibility
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 300.0,
        affects_lightmapped_meshes: false,
    });
    
    // Ground plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20.0, 20.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.3, 0.2),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));
    
    // Create basin walls using rigid bodies
    create_basin_walls(&mut commands, &mut meshes, &mut materials);
    
    // Spawn initial particles above the basin
    spawn_particle_drop(&mut commands, Vec3::new(0.0, 4.0, 0.0), 100);
    
    println!("=== Basin Stacking Demo ===");
    println!("Watch particles fall into the basin and stack on top of each other!");
    println!();
    println!("Controls:");
    println!("  Space: Spawn 50 particles");
    println!("  R: Spawn 200 particles");
    println!("  C: Clear all particles");
    println!();
    println!("Observe how particles:");
    println!("  - Fall into the basin");
    println!("  - Collide with walls");
    println!("  - Stack on top of each other");
    println!("  - Settle into stable configurations");
}

/// Create the basin walls using rigid body boxes
fn create_basin_walls(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let wall_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.6, 0.5, 0.4),
        perceptual_roughness: 0.8,
        metallic: 0.1,
        ..default()
    });
    
    // Basin dimensions - simple box with vertical walls
    let basin_size = 0.8;  // Interior size
    let wall_thickness = 0.15;
    let wall_height = 0.25;  // Short walls to see stacking behavior
    
    let half_size = basin_size / 2.0;
    let wall_offset = half_size + wall_thickness / 2.0;
    
    // North wall (vertical, extends full width including wall thickness)
    commands.spawn((
        RigidBodyBundle::static_box(
            Vec3::new(0.0, wall_height / 2.0, wall_offset),
            Quat::IDENTITY,
            basin_size + wall_thickness * 2.0,  // Full width including corners
            wall_height,
            wall_thickness,
        ),
        Mesh3d(meshes.add(Cuboid::new(
            basin_size + wall_thickness * 2.0,
            wall_height,
            wall_thickness,
        ))),
        MeshMaterial3d(wall_material.clone()),
    ));
    
    // South wall (vertical, extends full width including wall thickness)
    commands.spawn((
        RigidBodyBundle::static_box(
            Vec3::new(0.0, wall_height / 2.0, -wall_offset),
            Quat::IDENTITY,
            basin_size + wall_thickness * 2.0,
            wall_height,
            wall_thickness,
        ),
        Mesh3d(meshes.add(Cuboid::new(
            basin_size + wall_thickness * 2.0,
            wall_height,
            wall_thickness,
        ))),
        MeshMaterial3d(wall_material.clone()),
    ));
    
    // East wall (vertical, only interior length to avoid overlap)
    commands.spawn((
        RigidBodyBundle::static_box(
            Vec3::new(wall_offset, wall_height / 2.0, 0.0),
            Quat::IDENTITY,
            wall_thickness,
            wall_height,
            basin_size,  // Only interior length
        ),
        Mesh3d(meshes.add(Cuboid::new(
            wall_thickness,
            wall_height,
            basin_size,
        ))),
        MeshMaterial3d(wall_material.clone()),
    ));
    
    // West wall (vertical, only interior length to avoid overlap)
    commands.spawn((
        RigidBodyBundle::static_box(
            Vec3::new(-wall_offset, wall_height / 2.0, 0.0),
            Quat::IDENTITY,
            wall_thickness,
            wall_height,
            basin_size,
        ),
        Mesh3d(meshes.add(Cuboid::new(
            wall_thickness,
            wall_height,
            basin_size,
        ))),
        MeshMaterial3d(wall_material),
    ));
}

/// Spawn particles in a cluster above a position
fn spawn_particle_drop(
    commands: &mut Commands,
    center: Vec3,
    count: usize,
) {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    
    for _ in 0..count {
        // Smaller spawn area to match basin size
        let offset = Vec3::new(
            rng.gen_range(-0.25..0.25),
            rng.gen_range(-0.1..0.1),
            rng.gen_range(-0.25..0.25),
        );
        
        let position = center + offset;
        
        commands.spawn(ParticleBundle::new(
            position,
            0, // Lunar regolith material
            0.02,  // radius
            0.001, // mass
        ));
    }
}

/// Handle spawning particles with Space and R keys
fn handle_spawn_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        spawn_particle_drop(&mut commands, Vec3::new(0.0, 4.0, 0.0), 50);
        println!("Spawned 50 particles above basin");
    }
    
    if keyboard.just_pressed(KeyCode::KeyR) {
        spawn_particle_drop(&mut commands, Vec3::new(0.0, 5.0, 0.0), 200);
        println!("Spawned 200 particles above basin");
    }
}

/// Handle clearing all particles with C key
fn handle_clear_input(
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