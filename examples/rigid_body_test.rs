//! Rigid body interaction test example
//!
//! This example demonstrates particle interaction with static rigid bodies.
//! Currently shows particles falling onto static obstacles.
//!
//! Controls:
//! - Space: Spawn 100 particles
//! - R: Spawn 500 particles
//! - C: Reset all particles
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
                title: "Bevy Regolith - Rigid Body Test".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RegolithPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (
            debug_rigid_bodies,
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
    
    // Ground plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20.0, 20.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));
    
    // Static obstacle - Box
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 1.0, 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.3, 0.3),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RigidBodyBundle::static_box(
            Vec3::new(-1.0, 0.5, 0.0),
            Quat::IDENTITY,
            2.0,
            1.0,
            2.0,
        ),
        StaticObstacle,
    ));
    
    // Static obstacle - Cylinder
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.8, 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.7),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RigidBodyBundle::static_cylinder(
            Vec3::new(1.0, 1.0, 0.0),
            Quat::IDENTITY,
            0.8,
            2.0,
        ),
        StaticObstacle,
    ));
    
    // Static obstacle - Horizontal Cylinder (rotated 90 degrees)
    let horizontal_rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.5, 2.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.5, 0.3),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RigidBodyBundle::static_cylinder(
            Vec3::new(0.0, 3.0, 0.0),
            horizontal_rotation,
            0.5,
            2.5,
        ),
        StaticObstacle,
    ));
    
    // Static obstacle - Ramp
    let ramp_rotation = Quat::from_rotation_x(-0.3);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(3.0, 0.2, 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            perceptual_roughness: 0.8,
            metallic: 0.1,
            ..default()
        })),
        RigidBodyBundle::static_box(
            Vec3::new(0.0, 2.0, -1.0),
            ramp_rotation,
            3.0,
            0.2,
            2.0,
        ),
        StaticObstacle,
    ));
    
    // Spawn initial particles above the obstacles
    spawn_particle_pile(&mut commands, Vec3::new(0.0, 6.0, 0.0), 0, 500);
    
    println!("=== Rigid Body Test ===");
    println!("Rigid body collision detection is now active!");
    println!("Particles will collide with the box, vertical cylinder, horizontal cylinder, and ramp.");
    println!("Press Space to spawn 100 particles, R to spawn 500 particles.");
    println!("Press C to reset all particles.");
}

/// Marker component for static obstacles
#[derive(Component)]
struct StaticObstacle;

/// Spawn a pile of particles
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

/// Debug system to check rigid body count
fn debug_rigid_bodies(
    rigid_bodies: Query<(&Transform, &CollisionShape, &RigidBody)>,
) {
    static mut PRINTED: bool = false;
    unsafe {
        if !PRINTED {
            let count = rigid_bodies.iter().count();
            println!("DEBUG: Found {} rigid bodies in scene", count);
            for (transform, shape, _rb) in rigid_bodies.iter() {
                println!("  - Shape: {:?} at position: {:?}", shape, transform.translation);
            }
            PRINTED = true;
        }
    }
}

/// Despawn particles that fall outside the ground plane boundaries
fn despawn_out_of_bounds_particles(
    mut commands: Commands,
    particles: Query<(Entity, &ParticlePosition), With<ActiveParticle>>,
) {
    const GROUND_SIZE: f32 = 20.0;
    const HALF_GROUND: f32 = GROUND_SIZE / 2.0;
    const MIN_Y: f32 = -1.0; // Despawn if below ground
    
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