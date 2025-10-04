use bevy::prelude::*;

/// Particle position in world space
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticlePosition(pub Vec3);

/// Particle velocity
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleVelocity(pub Vec3);

/// Previous position for PBD solver
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticlePrevPosition(pub Vec3);

/// Particle mass in kg
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleMass(pub f32);

/// Reference to material properties
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleMaterial(pub usize);

/// Particle radius for collision detection
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleRadius(pub f32);

/// Marker component for active particles
#[derive(Component, Debug, Clone, Copy)]
pub struct ActiveParticle;

/// Particle sleep state tracking
#[derive(Component, Debug, Clone, Copy)]
pub struct ParticleSleepState {
    /// Number of frames the particle has been nearly stationary
    pub stationary_frames: u32,
    /// Whether the particle is currently sleeping
    pub is_sleeping: bool,
}

impl Default for ParticleSleepState {
    fn default() -> Self {
        Self {
            stationary_frames: 0,
            is_sleeping: false,
        }
    }
}

/// Bundle for spawning particles
#[derive(Bundle)]
pub struct ParticleBundle {
    pub position: ParticlePosition,
    pub velocity: ParticleVelocity,
    pub prev_position: ParticlePrevPosition,
    pub mass: ParticleMass,
    pub material: ParticleMaterial,
    pub radius: ParticleRadius,
    pub active: ActiveParticle,
    pub sleep_state: ParticleSleepState,
}

impl ParticleBundle {
    pub fn new(position: Vec3, material_id: usize, radius: f32, mass: f32) -> Self {
        Self {
            position: ParticlePosition(position),
            velocity: ParticleVelocity(Vec3::ZERO),
            prev_position: ParticlePrevPosition(position),
            mass: ParticleMass(mass),
            material: ParticleMaterial(material_id),
            radius: ParticleRadius(radius),
            active: ActiveParticle,
            sleep_state: ParticleSleepState::default(),
        }
    }
}