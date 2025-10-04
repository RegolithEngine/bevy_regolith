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
        &ParticlePrevPosition,
        &ParticleRadius,
        &ParticleMass,
        &ParticleMaterial,
        &ParticleSleepState,
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
        for (entity, pos, _prev_pos, radius, mass, _mat_id, sleep_state) in particles.iter() {
            // Skip sleeping particles
            if config.enable_sleeping && sleep_state.is_sleeping {
                continue;
            }
            
            let neighbors = spatial_hash.query_neighbors(pos.0, radius.0 * 3.0);
            let mut correction = Vec3::ZERO;
            
            for neighbor_entity in neighbors {
                if neighbor_entity == entity {
                    continue;
                }
                
                if let Ok((_, neighbor_pos, _, neighbor_radius, neighbor_mass, _, _)) =
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
            if let Ok((_, mut pos, _, _, _, _, _)) = particles.get_mut(entity) {
                pos.0 += correction;
            }
        }
        
        // Ground plane collision
        for (_, mut pos, _prev_pos, radius, _, _mat_id, sleep_state) in particles.iter_mut() {
            // Skip sleeping particles
            if config.enable_sleeping && sleep_state.is_sleeping {
                continue;
            }
            
            if pos.0.y < radius.0 {
                pos.0.y = radius.0;
            }
        }
        
        // Particle-rigid body collisions with swept collision detection
        let rb_count = rigid_bodies.iter().count();
        static mut COLLISION_DEBUG_PRINTED: bool = false;
        
        for (_, mut pos, prev_pos, radius, _, _, sleep_state) in particles.iter_mut() {
            // Skip sleeping particles
            if config.enable_sleeping && sleep_state.is_sleeping {
                continue;
            }
            
            for (rb_transform, shape, _rb) in rigid_bodies.iter() {
                // Swept collision detection: check trajectory from previous to current position
                let ray_start = prev_pos.0;
                let ray_end = pos.0;
                let ray_dir = ray_end - ray_start;
                let ray_length = ray_dir.length();
                
                // Skip if particle didn't move much
                if ray_length < 0.0001 {
                    // Still do standard collision check for stationary particles
                    let world_to_local = rb_transform.compute_affine().inverse();
                    let local_pos = world_to_local.transform_point3(pos.0);
                    let closest_local = shape.closest_point(local_pos);
                    let closest_world = rb_transform.transform_point(closest_local);
                    
                    let delta = pos.0 - closest_world;
                    let dist = delta.length();
                    
                    if dist < radius.0 && dist > 0.0001 {
                        let penetration = radius.0 - dist;
                        let normal = delta / dist;
                        pos.0 += normal * penetration;
                    }
                    continue;
                }
                
                let _ray_dir_normalized = ray_dir / ray_length;
                
                // Sample along the ray to detect fast-moving collisions
                let num_samples = (ray_length / (radius.0 * 0.5)).ceil().max(2.0) as usize;
                let mut earliest_collision_t = f32::MAX;
                let mut collision_point = pos.0;
                let mut collision_normal = Vec3::Y;
                let mut found_collision = false;
                
                for i in 0..num_samples {
                    let t = i as f32 / (num_samples - 1) as f32;
                    let sample_pos = ray_start + ray_dir * t;
                    
                    // Transform sample position to rigid body local space
                    let world_to_local = rb_transform.compute_affine().inverse();
                    let local_sample = world_to_local.transform_point3(sample_pos);
                    
                    // Get closest point on shape surface
                    let closest_local = shape.closest_point(local_sample);
                    let closest_world = rb_transform.transform_point(closest_local);
                    
                    // Check distance
                    let delta = sample_pos - closest_world;
                    let dist = delta.length();
                    
                    if dist < radius.0 && t < earliest_collision_t {
                        earliest_collision_t = t;
                        collision_point = closest_world;
                        collision_normal = if dist > 0.0001 {
                            delta / dist
                        } else {
                            // Use local space normal if too close
                            let local_normal = (local_sample - closest_local).normalize_or_zero();
                            if local_normal.length_squared() > 0.0001 {
                                rb_transform.rotation * local_normal
                            } else {
                                Vec3::Y
                            }
                        };
                        found_collision = true;
                    }
                }
                
                // If collision found, reflect velocity and place particle appropriately
                if found_collision {
                    // Calculate the velocity vector from the movement
                    let velocity = ray_dir / config.dt;
                    
                    // Remove the normal component of velocity (bounce/slide)
                    let normal_velocity = velocity.dot(collision_normal);
                    
                    // Only apply correction if moving into the surface
                    if normal_velocity < 0.0 {
                        // Remove normal component and keep tangential (sliding)
                        let tangential_velocity = velocity - collision_normal * normal_velocity;
                        
                        // Place particle at surface with some restitution (bounciness)
                        let restitution = 0.0; // No bounce for now, pure sliding
                        let new_velocity = tangential_velocity - collision_normal * normal_velocity * restitution;
                        
                        // Update position based on corrected velocity
                        // Start from collision point and continue with tangential motion
                        let collision_pos = collision_point + collision_normal * radius.0;
                        let remaining_time = (1.0 - earliest_collision_t) * config.dt;
                        pos.0 = collision_pos + new_velocity * remaining_time;
                        
                        unsafe {
                            if !COLLISION_DEBUG_PRINTED {
                                println!("SWEPT COLLISION: Particle trajectory intersected rigid body");
                                println!("  Collision at t={}, normal={:?}", earliest_collision_t, collision_normal);
                                println!("  Normal vel: {}, Tangential preserved", normal_velocity);
                                COLLISION_DEBUG_PRINTED = true;
                            }
                        }
                    }
                    continue;
                }
                
                // Standard collision check for current position
                let world_to_local = rb_transform.compute_affine().inverse();
                let local_pos = world_to_local.transform_point3(pos.0);
                let closest_local = shape.closest_point(local_pos);
                let closest_world = rb_transform.transform_point(closest_local);
                
                let delta = pos.0 - closest_world;
                let dist = delta.length();
                
                // Check if particle is penetrating or too close to the rigid body
                // Particle should be at least radius.0 distance away from the surface
                if dist < radius.0 {
                    if dist > 0.0001 {
                        // Normal case: push particle away from surface
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
                        
                        // Push particle out of rigid body along normal
                        pos.0 += normal * penetration;
                    } else {
                        // Particle center is very close to or at the surface point
                        // Use the normal from local space to push it out
                        let local_normal = (local_pos - closest_local).normalize_or_zero();
                        if local_normal.length_squared() > 0.0001 {
                            let world_normal = rb_transform.rotation * local_normal;
                            pos.0 = closest_world + world_normal * radius.0;
                        } else {
                            // Fallback: push up
                            pos.0 = closest_world + Vec3::Y * radius.0;
                        }
                    }
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