use bevy::prelude::*;
use crate::particle::*;
use crate::spatial::SpatialHash;
use crate::material::MaterialRegistry;
use crate::solver::SolverConfig;
use crate::rigid_body::{RigidBody, CollisionShape};

/// System: Solve particle-particle and particle-ground collision constraints
pub fn solve_constraints(
    mut particles: Query<(
        Entity,
        &mut ParticlePosition,
        &ParticleRadius,
        &ParticleMass,
        &ParticleMaterial,
    )>,
    rigid_bodies: Query<(&Transform, &CollisionShape, &RigidBody)>,
    spatial_hash: Res<SpatialHash>,
    _materials: Res<MaterialRegistry>,
    config: Res<SolverConfig>,
) {
    // Iterate constraint solver
    for _ in 0..config.iterations {
        // Collect position updates to avoid borrow checker issues
        let mut position_updates: Vec<(Entity, Vec3)> = Vec::new();
        
        // Particle-particle collisions
        for (entity, pos, radius, mass, _mat_id) in particles.iter() {
            let neighbors = spatial_hash.query_neighbors(pos.0, radius.0 * 3.0);
            let mut correction = Vec3::ZERO;
            
            for neighbor_entity in neighbors {
                if neighbor_entity == entity {
                    continue;
                }
                
                if let Ok((_, neighbor_pos, neighbor_radius, neighbor_mass, _)) = 
                    particles.get(neighbor_entity) 
                {
                    let delta = neighbor_pos.0 - pos.0;
                    let dist = delta.length();
                    let min_dist = radius.0 + neighbor_radius.0;
                    
                    if dist < min_dist && dist > 0.0001 {
                        let penetration = min_dist - dist;
                        let dir = delta / dist;
                        
                        // Mass-weighted correction
                        let total_mass = mass.0 + neighbor_mass.0;
                        let weight = neighbor_mass.0 / total_mass;
                        correction -= dir * penetration * weight * 0.5;
                    }
                }
            }
            
            position_updates.push((entity, correction));
        }
        
        // Apply position updates
        for (entity, correction) in position_updates {
            if let Ok((_, mut pos, _, _, _)) = particles.get_mut(entity) {
                pos.0 += correction;
            }
        }
        
        // Ground plane collision
        for (_, mut pos, radius, _, _mat_id) in particles.iter_mut() {
            if pos.0.y < radius.0 {
                pos.0.y = radius.0;
            }
        }
        
        // Particle-rigid body collisions
        let rb_count = rigid_bodies.iter().count();
        static mut COLLISION_DEBUG_PRINTED: bool = false;
        
        for (_, mut pos, radius, _, _) in particles.iter_mut() {
            for (rb_transform, shape, _rb) in rigid_bodies.iter() {
                // Transform particle position to rigid body local space
                let world_to_local = rb_transform.compute_affine().inverse();
                let local_pos = world_to_local.transform_point3(pos.0);
                
                // Get closest point on shape surface (in local space)
                let closest_local = shape.closest_point(local_pos);
                
                // Transform back to world space
                let closest_world = rb_transform.transform_point(closest_local);
                
                // Calculate penetration
                let delta = pos.0 - closest_world;
                let dist = delta.length();
                
                // If particle is penetrating the rigid body
                if dist < radius.0 && dist > 0.0001 {
                    let penetration = radius.0 - dist;
                    let normal = delta / dist;
                    
                    unsafe {
                        if !COLLISION_DEBUG_PRINTED {
                            println!("COLLISION: Particle at {:?} colliding with rigid body", pos.0);
                            println!("  Local pos: {:?}, Closest: {:?}", local_pos, closest_local);
                            println!("  Dist: {}, Radius: {}, Penetration: {}", dist, radius.0, penetration);
                            COLLISION_DEBUG_PRINTED = true;
                        }
                    }
                    
                    // Push particle out of rigid body
                    pos.0 += normal * penetration;
                }
            }
        }
        
        unsafe {
            if !COLLISION_DEBUG_PRINTED && rb_count > 0 {
                static mut FRAME_COUNT: u32 = 0;
                FRAME_COUNT += 1;
                if FRAME_COUNT == 60 {
                    println!("DEBUG: No collisions detected after 60 frames with {} rigid bodies", rb_count);
                }
            }
        }
    }
}