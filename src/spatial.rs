use bevy::prelude::*;
use std::collections::HashMap;
use crate::particle::ParticlePosition;

/// Spatial hash grid for efficient neighbor queries
#[derive(Resource)]
pub struct SpatialHash {
    /// Grid cell size (typically 2-3x particle radius)
    pub cell_size: f32,
    
    /// Hash map from cell coordinates to particle entities
    grid: HashMap<IVec3, Vec<Entity>>,
    
    /// Bounds of the simulation domain
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}

impl SpatialHash {
    pub fn new(cell_size: f32, bounds_min: Vec3, bounds_max: Vec3) -> Self {
        Self {
            cell_size,
            grid: HashMap::new(),
            bounds_min,
            bounds_max,
        }
    }
    
    /// Convert world position to grid cell coordinates
    fn world_to_cell(&self, position: Vec3) -> IVec3 {
        ((position - self.bounds_min) / self.cell_size).as_ivec3()
    }
    
    /// Clear and rebuild the spatial hash grid
    pub fn rebuild<F>(&mut self, particles: &Query<(Entity, &ParticlePosition), F>)
    where
        F: bevy::ecs::query::QueryFilter,
    {
        self.grid.clear();
        
        for (entity, pos) in particles.iter() {
            let cell = self.world_to_cell(pos.0);
            self.grid.entry(cell).or_insert_with(Vec::new).push(entity);
        }
    }
    
    /// Query neighbors within a radius
    pub fn query_neighbors(&self, position: Vec3, radius: f32) -> Vec<Entity> {
        let mut neighbors = Vec::new();
        let cell = self.world_to_cell(position);
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        
        // Check neighboring cells
        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                for dz in -cell_radius..=cell_radius {
                    let neighbor_cell = cell + IVec3::new(dx, dy, dz);
                    
                    if let Some(entities) = self.grid.get(&neighbor_cell) {
                        neighbors.extend(entities.iter().copied());
                    }
                }
            }
        }
        
        neighbors
    }
    
    /// Get all occupied cells (for debug visualization)
    pub fn occupied_cells(&self) -> impl Iterator<Item = (&IVec3, &Vec<Entity>)> {
        self.grid.iter()
    }
    
    /// Get cell center position in world space
    pub fn cell_center(&self, cell: IVec3) -> Vec3 {
        self.bounds_min + cell.as_vec3() * self.cell_size + Vec3::splat(self.cell_size * 0.5)
    }
}

impl Default for SpatialHash {
    fn default() -> Self {
        Self::new(
            0.1,  // 10cm cells
            Vec3::new(-10.0, -2.0, -10.0),
            Vec3::new(10.0, 15.0, 10.0),
        )
    }
}