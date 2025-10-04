use bevy::prelude::*;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy_egui::{egui, EguiContexts};
use crate::solver::SolverConfig;
use crate::particle::ActiveParticle;
use crate::spawner::spawn_particle_cluster;

/// Event to trigger particle reset
#[derive(Event)]
pub struct ResetParticlesEvent;

/// Resource to store ground plane color (defined in examples)
#[derive(Resource)]
pub struct GroundPlaneColor(pub Color);

impl Default for GroundPlaneColor {
    fn default() -> Self {
        Self(Color::srgb(0.3, 0.3, 0.3))
    }
}

/// System: Update UI for parameter tuning
pub fn update_ui(
    mut contexts: EguiContexts,
    mut config: ResMut<SolverConfig>,
    particle_count: Query<&ActiveParticle>,
    mut commands: Commands,
    mut ground_plane_color: Option<ResMut<GroundPlaneColor>>,
    diagnostics: Res<DiagnosticsStore>,
    mut reset_events: EventWriter<ResetParticlesEvent>,
) {
    // Try to get ground plane color if it exists (for examples that have it)
    let mut ground_color_rgb: Option<[f32; 3]> = ground_plane_color.as_ref().map(|c| {
        // Convert to linear first, then get RGB values for egui
        let linear = c.0.to_linear();
        [linear.red, linear.green, linear.blue]
    });
    
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
        
        // Ground plane color picker (if available)
        if let Some(ref mut rgb) = ground_color_rgb {
            ui.heading("Ground Plane");
            if ui.color_edit_button_rgb(rgb).changed() {
                // Update the resource and trigger change detection
                if let Some(ref mut color_res) = ground_plane_color {
                    // egui gives us linear RGB, so create color from linear values
                    let new_color = Color::linear_rgb(rgb[0], rgb[1], rgb[2]);
                    color_res.0 = new_color;
                    // Explicitly mark as changed to trigger change detection
                    color_res.set_changed();
                }
            }
            ui.separator();
        }
        
        ui.label(format!("Active Particles: {}", particle_count.iter().count()));
        
        ui.separator();
        
        if ui.button("Spawn 100 Particles (Space)").clicked() {
            spawn_particle_cluster(&mut commands, Vec3::new(0.0, 5.0, 0.0), 0, 100);
        }
        
        if ui.button("Spawn 500 Particles (R)").clicked() {
            spawn_particle_cluster(&mut commands, Vec3::new(0.0, 8.0, 0.0), 0, 500);
        }
        
        ui.separator();
        
        if ui.button("Reset All Particles (C)").clicked() {
            reset_events.write(ResetParticlesEvent);
        }
        
        ui.separator();
        
        ui.label("Controls:");
        ui.label("• Space: Spawn 100 particles");
        ui.label("• R: Spawn 500 particles");
        ui.label("• C: Reset all particles");
        ui.label("• Right Mouse: Rotate camera");
        ui.label("• Mouse Wheel: Zoom");
        ui.label("• WASD: Pan camera");
        ui.label("• Q/E: Move camera up/down");
    });
    
    // FPS counter in upper right corner
    egui::Window::new("FPS")
        .fixed_pos(egui::pos2(
            contexts.ctx_mut().screen_rect().width() - 120.0,
            10.0
        ))
        .resizable(false)
        .collapsible(false)
        .title_bar(false)
        .show(contexts.ctx_mut(), |ui| {
            if let Some(fps_diagnostic) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
                if let Some(fps_value) = fps_diagnostic.smoothed() {
                    ui.label(
                        egui::RichText::new(format!("FPS: {:.0}", fps_value))
                            .size(20.0)
                            .color(egui::Color32::from_rgb(100, 255, 100))
                    );
                } else {
                    ui.label(
                        egui::RichText::new("FPS: --")
                            .size(20.0)
                            .color(egui::Color32::from_rgb(100, 255, 100))
                    );
                }
            } else {
                ui.label(
                    egui::RichText::new("FPS: --")
                        .size(20.0)
                        .color(egui::Color32::from_rgb(100, 255, 100))
                );
            }
        });
}