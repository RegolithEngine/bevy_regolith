//! Interactive sandbox example with material switching
//!
//! Controls:
//! - Space: Spawn 100 particles with current material
//! - R: Spawn 500 particles with current material
//! - C: Reset all particles
//! - 1/2/3: Switch material (Lunar Regolith/Sand/Snow)
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
                title: "Bevy Regolith - Sandbox".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RegolithPlugin)
        .insert_resource(CurrentMaterial(0)) // Start with lunar regolith
        .add_systems(Startup, setup)
        .add_systems(Update, (
            handle_material_switching,
            handle_sandbox_spawn_input,
        ))
        .run();
}

/// Resource to track currently selected material
#[derive(Resource)]
struct CurrentMaterial(usize);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera with orbit controls
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera::default(),
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
        brightness: 200.0,
        affects_lightmapped_meshes: false,
    });
    
    // Ground plane (larger for sandbox)
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(30.0, 30.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));
    
    // Spawn initial pile of particles (lunar regolith)
    spawn_initial_pile(&mut commands, 0);
}

/// Spawn an initial pile of particles
fn spawn_initial_pile(commands: &mut Commands, material_id: usize) {
    // Create a pyramid-like pile
    for layer in 0..8 {
        let y = layer as f32 * 0.04 + 0.5;
        let size = 8 - layer;
        
        for x in -size..=size {
            for z in -size..=size {
                let position = Vec3::new(
                    x as f32 * 0.045,
                    y,
                    z as f32 * 0.045,
                );
                
                commands.spawn(ParticleBundle::new(
                    position,
                    material_id,
                    0.02,
                    0.001,
                ));
            }
        }
    }
}

/// Handle material switching with number keys
fn handle_material_switching(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut current_material: ResMut<CurrentMaterial>,
) {
    if keyboard.just_pressed(KeyCode::Digit1) {
        current_material.0 = 0; // Lunar regolith
        println!("Switched to Lunar Regolith");
    } else if keyboard.just_pressed(KeyCode::Digit2) {
        current_material.0 = 1; // Sand
        println!("Switched to Sand");
    } else if keyboard.just_pressed(KeyCode::Digit3) {
        current_material.0 = 2; // Snow
        println!("Switched to Snow");
    }
}

/// Handle spawning particles with the currently selected material
fn handle_sandbox_spawn_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    current_material: Res<CurrentMaterial>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        spawn_particle_cluster(&mut commands, Vec3::new(0.0, 5.0, 0.0), current_material.0, 100);
        println!("Spawned 100 particles");
    }
    
    if keyboard.just_pressed(KeyCode::KeyR) {
        spawn_particle_cluster(&mut commands, Vec3::new(0.0, 8.0, 0.0), current_material.0, 500);
        println!("Spawned 500 particles");
    }
}

/// Helper: Spawn a cluster of particles
fn spawn_particle_cluster(
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