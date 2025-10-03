use bevy::prelude::*;
use crate::spatial::SpatialHash;
use crate::solver::SolverConfig;
use crate::particle::*;

/// System: Debug visualization using gizmos
pub fn debug_draw_system(
    mut gizmos: Gizmos,
    spatial_hash: Res<SpatialHash>,
    config: Res<SolverConfig>,
    particles: Query<(&ParticlePosition, &ParticleVelocity), With<ActiveParticle>>,
) {
    // Draw spatial grid
    if config.debug_draw_grid {
        for (cell_coord, entities) in spatial_hash.occupied_cells() {
            if !entities.is_empty() {
                let cell_center = spatial_hash.cell_center(*cell_coord);
                let cell_size = spatial_hash.cell_size;
                
                // Draw cell as wireframe cube
                gizmos.cuboid(
                    Transform::from_translation(cell_center)
                        .with_scale(Vec3::splat(cell_size)),
                    Color::srgba(0.0, 1.0, 0.0, 0.2),
                );
            }
        }
    }
    
    // Draw velocity vectors
    if config.debug_draw_velocities {
        for (pos, vel) in particles.iter() {
            if vel.0.length() > 0.01 {
                gizmos.arrow(
                    pos.0,
                    pos.0 + vel.0 * 0.1,
                    Color::srgb(1.0, 0.0, 0.0),
                );
            }
        }
    }
}