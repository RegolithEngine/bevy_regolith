use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use crate::solver::SolverConfig;
use crate::particle::ActiveParticle;
use crate::spawner::spawn_particle_cluster;

/// System: Update UI for parameter tuning
pub fn update_ui(
    mut contexts: EguiContexts,
    mut config: ResMut<SolverConfig>,
    particle_count: Query<&ActiveParticle>,
    mut commands: Commands,
) {
    egui::Window::new("Regolith Controls").show(contexts.ctx_mut(), |ui| {
        ui.heading("Solver Parameters");
        
        ui.add(egui::Slider::new(&mut config.gravity.y, -20.0..=0.0)
            .text("Gravity (Y)"));
        
        ui.add(egui::Slider::new(&mut config.iterations, 1..=20)
            .text("Iterations"));
        
        ui.add(egui::Slider::new(&mut config.damping, 0.0..=1.0)
            .text("Damping"));
        
        ui.separator();
        
        ui.checkbox(&mut config.enable_collisions, "Enable Collisions");
        ui.checkbox(&mut config.debug_draw_grid, "Debug: Draw Grid");
        ui.checkbox(&mut config.debug_draw_velocities, "Debug: Draw Velocities");
        
        ui.separator();
        
        ui.label(format!("Active Particles: {}", particle_count.iter().count()));
        
        ui.separator();
        
        if ui.button("Spawn 100 Particles (Space)").clicked() {
            spawn_particle_cluster(&mut commands, Vec3::new(0.0, 5.0, 0.0), 0, 100);
        }
        
        if ui.button("Spawn 500 Particles (R)").clicked() {
            spawn_particle_cluster(&mut commands, Vec3::new(0.0, 8.0, 0.0), 0, 500);
        }
        
        ui.separator();
        
        ui.label("Controls:");
        ui.label("• Space: Spawn 100 particles");
        ui.label("• R: Spawn 500 particles");
        ui.label("• Right Mouse: Rotate camera");
        ui.label("• Mouse Wheel: Zoom");
        ui.label("• WASD: Pan camera");
        ui.label("• Q/E: Move camera up/down");
    });
}