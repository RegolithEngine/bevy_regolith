//! Rapier physics integration for bevy_regolith
//!
//! This module provides integration between bevy_regolith particles and Rapier rigid bodies,
//! allowing particles to apply forces to Rapier-controlled objects.
//!
//! # Example
//!
//! ```no_run
//! use bevy::prelude::*;
//! use bevy_regolith::prelude::*;
//! use bevy_rapier3d::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .add_plugins(RegolithPlugin)
//!         .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
//!         .add_plugins(RapierIntegrationPlugin::default())
//!         .run();
//! }
//! ```

use bevy::prelude::*;
use bevy_rapier3d::prelude::{
    Collider, ExternalForce, RigidBody as RapierRigidBody, Velocity,
};
use std::collections::HashMap;

use crate::particle::*;
use crate::rigid_body::{RigidBody, RigidBodyBundle};

/// Marker component for Rapier-controlled rigid bodies that should interact with particles
#[derive(Component, Debug, Clone, Copy)]
pub struct SyncToRegolith;

/// Configuration for Rapier integration force calculations
#[derive(Resource, Clone)]
pub struct RapierIntegrationConfig {
    /// Scale factor for penetration correction forces
    pub penetration_force_scale: f32,
    
    /// Overall force multiplier
    pub total_force_scale: f32,
    
    /// Coefficient of restitution (bounciness) for particle-body collisions
    pub restitution: f32,
}

impl Default for RapierIntegrationConfig {
    fn default() -> Self {
        Self {
            penetration_force_scale: 10.0,
            total_force_scale: 5.0,
            restitution: 0.0,
        }
    }
}

/// Accumulates forces and torques from particle collisions before applying to Rapier
#[derive(Resource, Default)]
pub struct ParticleForceAccumulator {
    /// Forces to apply to rigid body centers of mass
    forces: HashMap<Entity, Vec3>,
    
    /// Torques to apply to rigid bodies
    torques: HashMap<Entity, Vec3>,
    
    /// Number of particle contacts per body (for debugging)
    contact_counts: HashMap<Entity, usize>,
}

impl ParticleForceAccumulator {
    /// Clear all accumulated forces and torques
    pub fn clear(&mut self) {
        self.forces.clear();
        self.torques.clear();
        self.contact_counts.clear();
    }
    
    /// Add a force at a contact point, automatically calculating torque
    pub fn add_force(&mut self, entity: Entity, force: Vec3, contact_point: Vec3, body_center: Vec3) {
        *self.forces.entry(entity).or_default() += force;
        
        // Calculate torque from contact point
        let r = contact_point - body_center;
        let torque = r.cross(force);
        *self.torques.entry(entity).or_default() += torque;
        
        *self.contact_counts.entry(entity).or_default() += 1;
    }
    
    /// Get the accumulated force for an entity
    pub fn get_force(&self, entity: Entity) -> Option<Vec3> {
        self.forces.get(&entity).copied()
    }
    
    /// Get the accumulated torque for an entity
    pub fn get_torque(&self, entity: Entity) -> Option<Vec3> {
        self.torques.get(&entity).copied()
    }
    
    /// Get the number of contacts for an entity
    pub fn get_contact_count(&self, entity: Entity) -> usize {
        self.contact_counts.get(&entity).copied().unwrap_or(0)
    }
    
    /// Iterate over all entities with accumulated forces
    pub fn iter_forces(&self) -> impl Iterator<Item = (&Entity, &Vec3)> {
        self.forces.iter()
    }
}

/// Plugin for integrating Rapier physics with bevy_regolith particles
pub struct RapierIntegrationPlugin {
    /// Configuration for force calculations
    pub config: RapierIntegrationConfig,
}

impl Default for RapierIntegrationPlugin {
    fn default() -> Self {
        Self {
            config: RapierIntegrationConfig::default(),
        }
    }
}

impl Plugin for RapierIntegrationPlugin {
    fn build(&self, app: &mut App) {
        app
            .insert_resource(self.config.clone())
            .insert_resource(ParticleForceAccumulator::default())
            .add_systems(Update, sync_rapier_to_regolith)
            .add_systems(FixedUpdate, (
                calculate_particle_forces
                    .before(crate::solver::predict_positions),
                apply_particle_forces
                    .after(calculate_particle_forces)
                    .before(bevy_rapier3d::plugin::PhysicsSet::StepSimulation),
            ));
    }
}

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

/// Calculate forces from particle collisions with Rapier bodies
fn calculate_particle_forces(
    particles: Query<(&ParticlePosition, &ParticlePrevPosition, &ParticleRadius, &ParticleMass)>,
    rapier_bodies: Query<(Entity, &Transform, &Collider, &RapierRigidBody, &Velocity), With<SyncToRegolith>>,
    mut force_accumulator: ResMut<ParticleForceAccumulator>,
    config: Res<RapierIntegrationConfig>,
    time: Res<Time>,
) {
    force_accumulator.clear();
    
    let dt = time.delta_secs();
    if dt < 0.0001 {
        return;
    }
    
    for (particle_pos, particle_prev_pos, particle_radius, particle_mass) in particles.iter() {
        // Calculate particle velocity
        let particle_velocity = (particle_pos.0 - particle_prev_pos.0) / dt;
        
        for (rb_entity, rb_transform, collider, rb_type, rb_velocity) in rapier_bodies.iter() {
            // Only apply forces to dynamic bodies
            if !matches!(rb_type, RapierRigidBody::Dynamic) {
                continue;
            }
            
            // Simple collision detection - check if particle overlaps with collider
            let world_to_local = rb_transform.compute_affine().inverse();
            let local_pos = world_to_local.transform_point3(particle_pos.0);
            
            // Get closest point on collider
            let closest_point = if let Some(cuboid) = collider.as_cuboid() {
                let half_extents = cuboid.half_extents();
                Vec3::new(
                    local_pos.x.clamp(-half_extents.x, half_extents.x),
                    local_pos.y.clamp(-half_extents.y, half_extents.y),
                    local_pos.z.clamp(-half_extents.z, half_extents.z),
                )
            } else if let Some(ball) = collider.as_ball() {
                let radius = ball.radius();
                if local_pos.length() > 0.0001 {
                    local_pos.normalize() * radius
                } else {
                    Vec3::ZERO
                }
            } else {
                continue;
            };
            
            let closest_world = rb_transform.transform_point(closest_point);
            let delta = particle_pos.0 - closest_world;
            let dist = delta.length();
            
            // Check if particle is colliding
            if dist < particle_radius.0 && dist > 0.0001 {
                let penetration = particle_radius.0 - dist;
                let normal = delta / dist;
                
                // Calculate velocity of rigid body at contact point
                let r = closest_world - rb_transform.translation;
                let rb_velocity_at_contact = Vec3::from(rb_velocity.linvel) + Vec3::from(rb_velocity.angvel).cross(r);
                
                // Calculate relative velocity (particle relative to body at contact point)
                let relative_velocity = particle_velocity - rb_velocity_at_contact;
                
                // Project relative velocity onto normal
                let normal_velocity = relative_velocity.dot(normal);
                
                // Only apply force if particle and body are approaching each other
                if normal_velocity < 0.0 {
                    // Impulse-based force calculation
                    // normal points from body surface toward particle
                    // When particle hits body, body should be pushed AWAY from particle
                    // So force on body is in -normal direction (opposite to relative approach)
                    let impulse_magnitude = particle_mass.0 * normal_velocity.abs();
                    let force = -normal * impulse_magnitude / dt;
                    
                    // Penetration correction force also pushes body away from particle
                    let penetration_force = -normal * penetration * particle_mass.0 * config.penetration_force_scale;
                    let total_force = (force + penetration_force) * config.total_force_scale;
                    
                    // Apply force at contact point
                    force_accumulator.add_force(
                        rb_entity,
                        total_force,
                        closest_world,
                        rb_transform.translation,
                    );
                }
            }
        }
    }
}

/// Apply accumulated particle forces to Rapier rigid bodies
fn apply_particle_forces(
    force_accumulator: Res<ParticleForceAccumulator>,
    mut external_forces: Query<&mut ExternalForce>,
) {
    for (entity, force) in force_accumulator.iter_forces() {
        if let Ok(mut ext_force) = external_forces.get_mut(*entity) {
            // Set the force (don't add, to avoid accumulation issues)
            ext_force.force = *force;
            
            // Apply torque
            if let Some(torque) = force_accumulator.get_torque(*entity) {
                ext_force.torque = torque;
            }
        }
    }
}