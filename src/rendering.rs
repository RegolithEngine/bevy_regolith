use bevy::prelude::*;
use crate::particle::*;
use crate::material::MaterialRegistry;

/// System: Add visual representation to newly spawned particles
pub fn setup_particle_rendering(
    mut commands: Commands,
    particles: Query<(Entity, &ParticlePosition, &ParticleRadius, &ParticleMaterial), Added<ActiveParticle>>,
    materials: Res<MaterialRegistry>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create sphere mesh (reuse for all particles)
    let sphere_mesh = meshes.add(Sphere::new(1.0).mesh().ico(2).unwrap());
    
    for (entity, pos, radius, mat_id) in particles.iter() {
        let material = materials.get(mat_id.0).unwrap();
        
        let material_handle = standard_materials.add(StandardMaterial {
            base_color: material.color,
            perceptual_roughness: 0.8,
            metallic: 0.0,
            ..default()
        });
        
        commands.entity(entity).insert((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(material_handle),
            Transform::from_translation(pos.0)
                .with_scale(Vec3::splat(radius.0)),
        ));
    }
}

/// System: Update particle visual transforms
pub fn update_particle_transforms(
    mut particles: Query<(&ParticlePosition, &mut Transform), With<ActiveParticle>>,
) {
    for (pos, mut transform) in particles.iter_mut() {
        transform.translation = pos.0;
    }
}