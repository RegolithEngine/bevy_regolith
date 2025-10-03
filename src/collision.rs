use bevy::prelude::*;
use crate::particle::*;
use crate::spatial::SpatialHash;
use crate::material::MaterialRegistry;
use crate::solver::SolverConfig;

/// System: Solve particle-particle and particle-ground collision constraints
pub fn solve_constraints(
    mut particles: Query<(
        Entity,
        &mut ParticlePosition,
        &ParticleRadius,
        &ParticleMass,
        &ParticleMaterial,
    )>,
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
    }
}