//! Interactive sandbox example with material switching
//!
//! This example demonstrates different particle materials with distinct colors and physics.
//!
//! Controls:
//! - 1/2/3: Switch material (Lunar Regolith/Sand/Snow) - affects NEW particles only
//! - Space: Spawn 100 particles with current material
//! - R: Spawn 500 particles with current material
//! - C: Clear all particles and reset
//! - Left Mouse: Rotate camera
//! - Right Mouse: Pan camera
//! - Mouse Wheel: Zoom
//! - WASD/QE: Move camera
//!
//! Material Properties:
//! - Lunar Regolith (1): Gray color, high friction, low bounce
//! - Sand (2): Tan color, medium friction, low bounce
//! - Snow (3): White color, low friction, very low bounce

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
            despawn_out_of_bounds_particles,
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
    
    println!("=== Sandbox Example ===");
    println!("Material switching demo - each material has different color and physics!");
    println!();
    println!("Current Material: Lunar Regolith (gray)");
    println!();
    println!("Controls:");
    println!("  1: Lunar Regolith (gray) - high friction");
    println!("  2: Sand (tan) - medium friction");
    println!("  3: Snow (white) - low friction");
    println!("  Space: Spawn 100 particles");
    println!("  R: Spawn 500 particles");
    println!("  C: Clear all particles");
    println!();
    println!("TIP: Switch materials (1/2/3) then spawn (Space/R) to see different colors!");
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
        println!("\n>>> Switched to Lunar Regolith (gray) - high friction, low bounce");
        println!("    Spawn particles with Space or R to see the gray color!");
    } else if keyboard.just_pressed(KeyCode::Digit2) {
        current_material.0 = 1; // Sand
        println!("\n>>> Switched to Sand (tan) - medium friction, low bounce");
        println!("    Spawn particles with Space or R to see the tan color!");
    } else if keyboard.just_pressed(KeyCode::Digit3) {
        current_material.0 = 2; // Snow
        println!("\n>>> Switched to Snow (white) - low friction, very low bounce");
        println!("    Spawn particles with Space or R to see the white color!");
    }
}

/// Handle spawning particles with the currently selected material
fn handle_sandbox_spawn_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    current_material: Res<CurrentMaterial>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        let material_name = match current_material.0 {
            0 => "Lunar Regolith (gray)",
            1 => "Sand (tan)",
            2 => "Snow (white)",
            _ => "Unknown",
        };
        spawn_particle_cluster(&mut commands, Vec3::new(0.0, 5.0, 0.0), current_material.0, 100);
        println!("Spawned 100 {} particles", material_name);
    }
    
    if keyboard.just_pressed(KeyCode::KeyR) {
        let material_name = match current_material.0 {
            0 => "Lunar Regolith (gray)",
            1 => "Sand (tan)",
            2 => "Snow (white)",
            _ => "Unknown",
        };
        spawn_particle_cluster(&mut commands, Vec3::new(0.0, 8.0, 0.0), current_material.0, 500);
        println!("Spawned 500 {} particles", material_name);
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

/// Despawn particles that fall outside the ground plane boundaries
fn despawn_out_of_bounds_particles(
    mut commands: Commands,
    particles: Query<(Entity, &ParticlePosition), With<ActiveParticle>>,
) {
    const GROUND_SIZE: f32 = 30.0;
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