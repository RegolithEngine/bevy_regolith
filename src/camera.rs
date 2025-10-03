use bevy::prelude::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};

#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub sensitivity: f32,
    pub zoom_speed: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            focus: Vec3::ZERO,
            distance: 10.0,
            yaw: 0.0,
            pitch: -0.3,
            sensitivity: 0.003,
            zoom_speed: 0.5,
        }
    }
}

/// System: Handle orbit camera controls
pub fn orbit_camera_system(
    mut camera_query: Query<(&mut Transform, &mut OrbitCamera)>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_wheel: EventReader<MouseWheel>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    let Ok((mut transform, mut orbit)) = camera_query.single_mut() else {
        return;
    };
    
    // Left mouse button to rotate view (change angle)
    if mouse_button.pressed(MouseButton::Left) {
        for motion in mouse_motion.read() {
            orbit.yaw -= motion.delta.x * orbit.sensitivity;
            orbit.pitch -= motion.delta.y * orbit.sensitivity;
            orbit.pitch = orbit.pitch.clamp(-1.5, 1.5);
        }
    }
    // Right mouse button to pan (change focus point)
    else if mouse_button.pressed(MouseButton::Right) {
        for motion in mouse_motion.read() {
            let pan_speed = orbit.distance * 0.001;
            let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
            let right = rotation * Vec3::X;
            let up = rotation * Vec3::Y;
            orbit.focus -= right * motion.delta.x * pan_speed;
            orbit.focus += up * motion.delta.y * pan_speed;
        }
    } else {
        mouse_motion.clear();
    }
    
    // Mouse wheel to zoom
    for wheel in mouse_wheel.read() {
        orbit.distance -= wheel.y * orbit.zoom_speed;
        orbit.distance = orbit.distance.clamp(2.0, 50.0);
    }
    
    // WASD to pan focus
    let mut pan = Vec3::ZERO;
    let speed = 5.0 * time.delta_secs();
    
    if keyboard.pressed(KeyCode::KeyW) { pan.z -= speed; }
    if keyboard.pressed(KeyCode::KeyS) { pan.z += speed; }
    if keyboard.pressed(KeyCode::KeyA) { pan.x -= speed; }
    if keyboard.pressed(KeyCode::KeyD) { pan.x += speed; }
    if keyboard.pressed(KeyCode::KeyQ) { pan.y -= speed; }
    if keyboard.pressed(KeyCode::KeyE) { pan.y += speed; }
    
    // Apply pan in camera space
    let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
    orbit.focus += rotation * pan;
    
    // Update camera transform
    let offset = rotation * Vec3::new(0.0, 0.0, orbit.distance);
    transform.translation = orbit.focus + offset;
    transform.look_at(orbit.focus, Vec3::Y);
}