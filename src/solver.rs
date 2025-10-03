use bevy::prelude::*;
use crate::particle::*;
use crate::spatial::SpatialHash;

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

/// System: Rebuild spatial hash grid for efficient neighbor queries
pub fn rebuild_spatial_hash(
    mut spatial_hash: ResMut<SpatialHash>,
    particles: Query<(Entity, &ParticlePosition), With<ActiveParticle>>,
) {
    spatial_hash.rebuild(&particles);
}

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