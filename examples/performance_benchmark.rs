//! Performance benchmark example
//!
//! This example is designed to stress-test the physics system with large particle counts
//! and measure performance metrics. Use this with profiling tools like Tracy or cargo-flamegraph.
//!
//! Run with:
//! - cargo run --example performance_benchmark --release
//! - cargo flamegraph --example performance_benchmark
//!
//! Controls:
//! - 1-9: Set particle count (1000, 2000, 3000, ..., 9000)
//! - 0: Set particle count to 10000
//! - Space: Spawn additional 1000 particles
//! - C: Clear all particles
//! - P: Print performance stats
//! - Left Mouse: Rotate camera
//! - Mouse Wheel: Zoom

use bevy::prelude::*;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy_regolith::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Regolith - Performance Benchmark".to_string(),
                resolution: (1280.0, 720.0).into(),
                present_mode: bevy::window::PresentMode::AutoNoVsync, // Uncapped FPS
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RegolithPlugin)
        .insert_resource(BenchmarkStats::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (
            handle_benchmark_input,
            update_benchmark_stats,
            print_stats_on_demand,
        ))
        .run();
}

#[derive(Resource, Default)]
struct BenchmarkStats {
    particle_count: usize,
    frame_count: u64,
    total_frame_time: f32,
    min_fps: f64,
    max_fps: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera with orbit controls
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 20.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
        OrbitCamera {
            focus: Vec3::new(0.0, 3.0, 0.0),
            distance: 20.0,
            ..default()
        },
    ));
    
    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: false, // Disable shadows for better performance
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
        brightness: 300.0,
        affects_lightmapped_meshes: false,
    });
    
    // Ground plane
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
    
    // Spawn initial 3000 particles for baseline test
    spawn_particle_grid(&mut commands, 3000);
    
    println!("=== Performance Benchmark ===");
    println!("Starting with 3000 particles");
    println!();
    println!("Controls:");
    println!("  1-9: Set particle count (1000-9000)");
    println!("  0: Set particle count to 10000");
    println!("  Space: Add 1000 particles");
    println!("  C: Clear all particles");
    println!("  P: Print performance stats");
    println!();
    println!("Profiling Tips:");
    println!("  - Run with --release for accurate measurements");
    println!("  - Use 'cargo flamegraph --example performance_benchmark'");
    println!("  - Monitor FPS in window title");
}

/// Spawn particles in a grid pattern for consistent testing
fn spawn_particle_grid(commands: &mut Commands, count: usize) {
    let particles_per_side = (count as f32).cbrt().ceil() as i32;
    let spacing = 0.045;
    let offset = -(particles_per_side as f32 * spacing) / 2.0;
    
    let mut spawned = 0;
    'outer: for x in 0..particles_per_side {
        for y in 0..particles_per_side {
            for z in 0..particles_per_side {
                if spawned >= count {
                    break 'outer;
                }
                
                let position = Vec3::new(
                    offset + x as f32 * spacing,
                    2.0 + y as f32 * spacing,
                    offset + z as f32 * spacing,
                );
                
                commands.spawn(ParticleBundle::new(
                    position,
                    0, // material_id
                    0.02,
                    0.001,
                ));
                
                spawned += 1;
            }
        }
    }
    
    println!("Spawned {} particles in grid pattern", spawned);
}

/// Handle benchmark input controls
fn handle_benchmark_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    particles: Query<Entity, With<ActiveParticle>>,
    mut stats: ResMut<BenchmarkStats>,
) {
    // Clear particles
    if keyboard.just_pressed(KeyCode::KeyC) {
        let count = particles.iter().count();
        for entity in particles.iter() {
            commands.entity(entity).despawn();
        }
        stats.particle_count = 0;
        stats.frame_count = 0;
        stats.total_frame_time = 0.0;
        stats.min_fps = f64::MAX;
        stats.max_fps = 0.0;
        println!("Cleared {} particles", count);
        return;
    }
    
    // Set specific particle counts
    let target_count = if keyboard.just_pressed(KeyCode::Digit1) {
        Some(1000)
    } else if keyboard.just_pressed(KeyCode::Digit2) {
        Some(2000)
    } else if keyboard.just_pressed(KeyCode::Digit3) {
        Some(3000)
    } else if keyboard.just_pressed(KeyCode::Digit4) {
        Some(4000)
    } else if keyboard.just_pressed(KeyCode::Digit5) {
        Some(5000)
    } else if keyboard.just_pressed(KeyCode::Digit6) {
        Some(6000)
    } else if keyboard.just_pressed(KeyCode::Digit7) {
        Some(7000)
    } else if keyboard.just_pressed(KeyCode::Digit8) {
        Some(8000)
    } else if keyboard.just_pressed(KeyCode::Digit9) {
        Some(9000)
    } else if keyboard.just_pressed(KeyCode::Digit0) {
        Some(10000)
    } else {
        None
    };
    
    if let Some(target) = target_count {
        // Clear existing particles
        for entity in particles.iter() {
            commands.entity(entity).despawn();
        }
        
        // Spawn new particles
        spawn_particle_grid(&mut commands, target);
        
        // Reset stats
        stats.particle_count = target;
        stats.frame_count = 0;
        stats.total_frame_time = 0.0;
        stats.min_fps = f64::MAX;
        stats.max_fps = 0.0;
        
        println!("Set particle count to {}", target);
    }
    
    // Add 1000 particles
    if keyboard.just_pressed(KeyCode::Space) {
        spawn_particle_grid(&mut commands, 1000);
        println!("Added 1000 particles");
    }
}

/// Update benchmark statistics
fn update_benchmark_stats(
    mut stats: ResMut<BenchmarkStats>,
    particles: Query<Entity, With<ActiveParticle>>,
    diagnostics: Res<DiagnosticsStore>,
    time: Res<Time>,
) {
    stats.particle_count = particles.iter().count();
    stats.frame_count += 1;
    stats.total_frame_time += time.delta_secs();
    
    if let Some(fps_diagnostic) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
        if let Some(fps) = fps_diagnostic.smoothed() {
            if fps < stats.min_fps && fps > 0.0 {
                stats.min_fps = fps;
            }
            if fps > stats.max_fps {
                stats.max_fps = fps;
            }
        }
    }
}

/// Print performance statistics on demand
fn print_stats_on_demand(
    keyboard: Res<ButtonInput<KeyCode>>,
    stats: Res<BenchmarkStats>,
    diagnostics: Res<DiagnosticsStore>,
) {
    if keyboard.just_pressed(KeyCode::KeyP) {
        println!("\n=== Performance Statistics ===");
        println!("Particle Count: {}", stats.particle_count);
        println!("Frames Measured: {}", stats.frame_count);
        
        if stats.frame_count > 0 {
            let avg_frame_time = stats.total_frame_time / stats.frame_count as f32;
            let avg_fps = 1.0 / avg_frame_time;
            println!("Average FPS: {:.2}", avg_fps);
            println!("Average Frame Time: {:.2}ms", avg_frame_time * 1000.0);
        }
        
        if stats.min_fps < f64::MAX {
            println!("Min FPS: {:.2}", stats.min_fps);
        }
        if stats.max_fps > 0.0 {
            println!("Max FPS: {:.2}", stats.max_fps);
        }
        
        if let Some(fps_diagnostic) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(fps) = fps_diagnostic.smoothed() {
                println!("Current FPS: {:.2}", fps);
            }
        }
        
        println!("==============================\n");
    }
}