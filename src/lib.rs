//! Bevy Regolith - Granular physics simulation using Position-Based Dynamics
//!
//! This is a CPU-based prototype for testing PBD algorithms with rigid body interactions.
//! It serves as a development platform before migrating to GPU compute shaders.
//!
//! # Example
//!
//! ```no_run
//! use bevy::prelude::*;
//! use bevy_regolith::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .add_plugins(RegolithPlugin)
//!         .run();
//! }
//! ```

use bevy::prelude::*;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy_egui::EguiPlugin;

pub mod particle;
pub mod material;
pub mod spawner;
pub mod solver;
pub mod spatial;
pub mod collision;
pub mod rigid_body;
pub mod rendering;
pub mod camera;
pub mod ui;
pub mod debug;

#[cfg(feature = "rapier")]
pub mod rapier_integration;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::particle::*;
    pub use crate::material::*;
    pub use crate::spawner::*;
    pub use crate::solver::*;
    pub use crate::rigid_body::*;
    pub use crate::camera::*;
    pub use crate::RegolithPlugin;
    
    #[cfg(feature = "rapier")]
    pub use crate::rapier_integration::*;
}

/// Main plugin for the Regolith granular physics system
pub struct RegolithPlugin;

impl Plugin for RegolithPlugin {
    fn build(&self, app: &mut App) {
        app
            // Add diagnostics plugin for FPS counter
            .add_plugins(FrameTimeDiagnosticsPlugin::default())
            
            // Add egui plugin for UI
            .add_plugins(EguiPlugin {
                enable_multipass_for_primary_context: false,
            })
            
            // Events
            .add_event::<ui::ResetParticlesEvent>()
            
            // Resources
            .init_resource::<solver::SolverConfig>()
            .init_resource::<material::MaterialRegistry>()
            .init_resource::<spatial::SpatialHash>()
            
            // Startup systems
            .add_systems(Startup, (
                setup_materials,
                spawner::setup_initial_particles,
            ).chain())
            
            // Update systems (run every frame)
            .add_systems(Update, (
                spawner::handle_spawn_input,
                spawner::handle_reset_input,
                spawner::reset_particles,
                camera::orbit_camera_system,
                rendering::setup_particle_rendering,
                ui::update_ui,
            ))
            
            // Fixed update (physics - runs at fixed timestep)
            .add_systems(FixedUpdate, (
                solver::predict_positions,
                solver::rebuild_spatial_hash,
                collision::solve_constraints,
                solver::update_velocities,
                solver::apply_damping,
            ).chain())
            
            // Late update (after physics)
            .add_systems(Update, rendering::update_particle_transforms)
            .add_systems(Update, debug::debug_draw_system);
    }
}

/// Setup material registry with default materials
fn setup_materials(mut materials: ResMut<material::MaterialRegistry>) {
    materials.add(material::Material::lunar_regolith());
    materials.add(material::Material::sand());
    materials.add(material::Material::snow());
}