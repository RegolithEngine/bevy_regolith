use bevy::prelude::*;
use bevy_regolith::prelude::*;
use bevy_regolith::ui::GroundPlaneColor;

/// Marker component for the ground plane with its material handle
#[derive(Component)]
struct GroundPlane {
    material: Handle<StandardMaterial>,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RegolithPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, update_ground_plane_color)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera with orbit controls
    commands.spawn((
        Camera3d::default(),
        Camera {
            clear_color: ClearColorConfig::Custom(Color::srgb(0.1, 0.1, 0.15)),
            ..default()
        },
        Transform::from_xyz(0.0, 0.0, 0.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            focus: Vec3::new(0.0, 0.0, 0.0),  // Focus point above ground
            distance: 10.0,
            yaw: 0.0,
            pitch: -0.8,  // Higher pitch angle to look down from above
            sensitivity: 0.003,
            zoom_speed: 0.5,
        },
        bevy::core_pipeline::tonemapping::Tonemapping::TonyMcMapface,
    ));
    
    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            -0.5,
            0.5,
            0.0,
        )),
    ));
    
    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        affects_lightmapped_meshes: false,
    });
    
    // Initialize the ground plane color resource
    commands.insert_resource(GroundPlaneColor::default());
    
    // Ground plane with default gray color matching the resource
    let ground_color = GroundPlaneColor::default().0;
    println!("Creating ground plane with color: {:?}", ground_color);
    let ground_material = materials.add(StandardMaterial {
        base_color: ground_color,
        perceptual_roughness: 0.9,
        metallic: 0.0,
        reflectance: 0.5,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    
    let ground_mesh = meshes.add(Plane3d::default().mesh().size(20.0, 20.0));
    
    println!("Spawning ground plane entity with material handle: {:?}", ground_material);
    
    commands.spawn((
        GroundPlane {
            material: ground_material.clone(),
        },
        Mesh3d(ground_mesh),
        MeshMaterial3d(ground_material),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Visibility::default(),
    ));
}

/// System to update ground plane color when resource changes
fn update_ground_plane_color(
    ground_color: Res<GroundPlaneColor>,
    ground_plane: Query<&GroundPlane>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if ground_color.is_changed() {
        println!("Ground plane color changed to: {:?}", ground_color.0);
        for plane in ground_plane.iter() {
            if let Some(material) = materials.get_mut(&plane.material) {
                println!("Updating material base_color to: {:?}", ground_color.0);
                material.base_color = ground_color.0;
            }
        }
    }
}