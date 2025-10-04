use bevy::prelude::*;
use crate::particle::*;
use crate::material::MaterialRegistry;
use crate::ui::ResetParticlesEvent;
use rand::Rng;

/// System: Spawn particles on key press
pub fn handle_spawn_input(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    _materials: Res<MaterialRegistry>,
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
    _materials: Res<MaterialRegistry>,
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
                    0, // material_id (lunar regolith)
                    0.02,
                    0.001,
                ));
            }
        }
    }
}

/// System: Handle keyboard input for reset
pub fn handle_reset_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut reset_events: EventWriter<ResetParticlesEvent>,
) {
    if keyboard.just_pressed(KeyCode::KeyC) {
        reset_events.write(ResetParticlesEvent);
    }
}

/// System: Reset all particles
pub fn reset_particles(
    mut commands: Commands,
    particles: Query<Entity, With<ActiveParticle>>,
    mut reset_events: EventReader<ResetParticlesEvent>,
) {
    for _event in reset_events.read() {
        let count = particles.iter().count();
        for entity in particles.iter() {
            commands.entity(entity).despawn();
        }
        println!("Reset: Removed {} particles", count);
    }
}