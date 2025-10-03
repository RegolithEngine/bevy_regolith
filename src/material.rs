use bevy::prelude::*;

/// Material properties for granular media
#[derive(Clone, Copy, Debug)]
pub struct Material {
    /// Density in kg/m³
    pub density: f32,
    
    /// Friction coefficient [0-1]
    pub friction: f32,
    
    /// Coefficient of restitution (bounciness) [0-1]
    pub restitution: f32,
    
    /// Particle radius in meters
    pub particle_radius: f32,
    
    /// Visual color
    pub color: Color,
}

impl Material {
    /// Lunar regolith preset
    pub fn lunar_regolith() -> Self {
        Self {
            density: 1500.0,
            friction: 0.8,
            restitution: 0.1,
            particle_radius: 0.02,
            color: Color::srgb(0.7, 0.65, 0.6),
        }
    }
    
    /// Dry sand preset
    pub fn sand() -> Self {
        Self {
            density: 1600.0,
            friction: 0.6,
            restitution: 0.2,
            particle_radius: 0.02,
            color: Color::srgb(0.9, 0.8, 0.6),
        }
    }
    
    /// Snow preset
    pub fn snow() -> Self {
        Self {
            density: 400.0,
            friction: 0.4,
            restitution: 0.1,
            particle_radius: 0.015,
            color: Color::srgb(0.95, 0.95, 1.0),
        }
    }
}

/// Resource storing all materials
#[derive(Resource, Default)]
pub struct MaterialRegistry {
    pub materials: Vec<Material>,
}

impl MaterialRegistry {
    pub fn new() -> Self {
        Self {
            materials: Vec::new(),
        }
    }
    
    pub fn add(&mut self, material: Material) -> usize {
        let id = self.materials.len();
        self.materials.push(material);
        id
    }
    
    pub fn get(&self, id: usize) -> Option<&Material> {
        self.materials.get(id)
    }
}