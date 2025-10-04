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
    
    /// Particle sleeping configuration
    pub enable_sleeping: bool,
    /// Velocity threshold below which a particle is considered stationary (m/s)
    pub sleep_velocity_threshold: f32,
    /// Number of frames a particle must be stationary before sleeping
    pub sleep_frame_threshold: u32,
    /// Distance threshold for waking up nearby particles (multiplier of particle radius)
    pub wake_distance_multiplier: f32,
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
            enable_sleeping: true,
            sleep_velocity_threshold: 0.01,
            sleep_frame_threshold: 30,
            wake_distance_multiplier: 2.0,
        }
    }
}

/// System: Predict new positions based on velocity and external forces
pub fn predict_positions(
    mut particles: Query<(
        &mut ParticlePosition,
        &mut ParticlePrevPosition,
        &mut ParticleVelocity,
        &ParticleSleepState,
    )>,
    config: Res<SolverConfig>,
) {
    for (mut pos, mut prev_pos, mut vel, sleep_state) in particles.iter_mut() {
        // Skip sleeping particles
        if config.enable_sleeping && sleep_state.is_sleeping {
            continue;
        }
        
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
    particles: Query<(Entity, &ParticlePosition, &ParticleSleepState), With<ActiveParticle>>,
    config: Res<SolverConfig>,
) {
    // Only include awake particles in spatial hash if sleeping is enabled
    if config.enable_sleeping {
        spatial_hash.rebuild_with_filter(&particles, |sleep_state| !sleep_state.is_sleeping);
    } else {
        // When sleeping is disabled, include all particles
        spatial_hash.rebuild_with_filter(&particles, |_| true);
    }
}

/// System: Update velocities from position changes
pub fn update_velocities(
    mut particles: Query<(
        &ParticlePosition,
        &ParticlePrevPosition,
        &mut ParticleVelocity,
        &ParticleSleepState,
    )>,
    config: Res<SolverConfig>,
) {
    for (pos, prev_pos, mut vel, sleep_state) in particles.iter_mut() {
        // Skip sleeping particles
        if config.enable_sleeping && sleep_state.is_sleeping {
            continue;
        }
        
        // Velocity from position change
        vel.0 = (pos.0 - prev_pos.0) / config.dt;
    }
}

/// System: Apply velocity damping
pub fn apply_damping(
    mut particles: Query<(&mut ParticleVelocity, &ParticleSleepState)>,
    config: Res<SolverConfig>,
) {
    let damping_factor = 1.0 - config.damping;
    
    for (mut vel, sleep_state) in particles.iter_mut() {
        // Skip sleeping particles
        if config.enable_sleeping && sleep_state.is_sleeping {
            continue;
        }
        
        vel.0 *= damping_factor;
    }
}

/// System: Detect and put particles to sleep when they're stationary
pub fn detect_sleeping_particles(
    mut particles: Query<(
        &ParticleVelocity,
        &ParticlePosition,
        &mut ParticleSleepState,
    )>,
    config: Res<SolverConfig>,
) {
    if !config.enable_sleeping {
        return;
    }
    
    let velocity_threshold_sq = config.sleep_velocity_threshold * config.sleep_velocity_threshold;
    
    for (vel, _pos, mut sleep_state) in particles.iter_mut() {
        let velocity_sq = vel.0.length_squared();
        
        if velocity_sq < velocity_threshold_sq {
            // Particle is moving slowly
            sleep_state.stationary_frames += 1;
            
            if sleep_state.stationary_frames >= config.sleep_frame_threshold && !sleep_state.is_sleeping {
                sleep_state.is_sleeping = true;
            }
        } else {
            // Particle is moving fast, reset counter and wake up
            sleep_state.stationary_frames = 0;
            sleep_state.is_sleeping = false;
        }
    }
}

/// System: Wake up sleeping particles when nearby particles are active
pub fn wake_sleeping_particles(
    mut particles: Query<(
        Entity,
        &ParticlePosition,
        &ParticleRadius,
        &mut ParticleSleepState,
    )>,
    spatial_hash: Res<SpatialHash>,
    config: Res<SolverConfig>,
) {
    if !config.enable_sleeping {
        return;
    }
    
    // Collect particles that need to be woken up
    let mut particles_to_wake: Vec<Entity> = Vec::new();
    
    for (entity, pos, radius, sleep_state) in particles.iter() {
        // Skip if already awake
        if !sleep_state.is_sleeping {
            // Check if this active particle should wake up nearby sleeping particles
            let wake_radius = radius.0 * config.wake_distance_multiplier;
            let neighbors = spatial_hash.query_neighbors(pos.0, wake_radius);
            
            for neighbor_entity in neighbors {
                if neighbor_entity != entity {
                    if let Ok((_, _, _, neighbor_sleep)) = particles.get(neighbor_entity) {
                        if neighbor_sleep.is_sleeping {
                            particles_to_wake.push(neighbor_entity);
                        }
                    }
                }
            }
        }
    }
    
    // Wake up the collected particles
    for entity in particles_to_wake {
        if let Ok((_, _, _, mut sleep_state)) = particles.get_mut(entity) {
            sleep_state.is_sleeping = false;
            sleep_state.stationary_frames = 0;
        }
    }
}