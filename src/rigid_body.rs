//! Rigid body components and collision shapes for particle interaction

use bevy::prelude::*;

/// Marker component for rigid bodies that particles can collide with
#[derive(Component, Debug, Clone, Copy)]
pub struct RigidBody {
    /// Whether this rigid body is static (immovable) or dynamic
    pub is_static: bool,
}

impl RigidBody {
    /// Create a static rigid body (immovable)
    pub fn static_body() -> Self {
        Self { is_static: true }
    }
    
    /// Create a dynamic rigid body (can be moved by forces)
    pub fn dynamic() -> Self {
        Self { is_static: false }
    }
}

/// Collision shape types for rigid bodies
#[derive(Component, Debug, Clone)]
pub enum CollisionShape {
    /// Box shape with half-extents (width/2, height/2, depth/2)
    Box { half_extents: Vec3 },
    
    /// Sphere shape with radius
    Sphere { radius: f32 },
    
    /// Cylinder shape with radius and half-height
    Cylinder { radius: f32, half_height: f32 },
    
    /// Plane shape (infinite plane with normal pointing up)
    Plane { normal: Vec3 },
}

impl CollisionShape {
    /// Create a box collision shape from full dimensions
    pub fn box_shape(width: f32, height: f32, depth: f32) -> Self {
        Self::Box {
            half_extents: Vec3::new(width * 0.5, height * 0.5, depth * 0.5),
        }
    }
    
    /// Create a sphere collision shape
    pub fn sphere(radius: f32) -> Self {
        Self::Sphere { radius }
    }
    
    /// Create a cylinder collision shape from full height
    pub fn cylinder(radius: f32, height: f32) -> Self {
        Self::Cylinder {
            radius,
            half_height: height * 0.5,
        }
    }
    
    /// Create a plane collision shape
    pub fn plane(normal: Vec3) -> Self {
        Self::Plane {
            normal: normal.normalize(),
        }
    }
    
    /// Test if a point is inside this shape (in local space)
    pub fn contains_point(&self, point: Vec3) -> bool {
        match self {
            CollisionShape::Box { half_extents } => {
                point.x.abs() <= half_extents.x
                    && point.y.abs() <= half_extents.y
                    && point.z.abs() <= half_extents.z
            }
            CollisionShape::Sphere { radius } => point.length() <= *radius,
            CollisionShape::Cylinder { radius, half_height } => {
                let xz_dist = (point.x * point.x + point.z * point.z).sqrt();
                xz_dist <= *radius && point.y.abs() <= *half_height
            }
            CollisionShape::Plane { normal } => point.dot(*normal) <= 0.0,
        }
    }
    
    /// Get the closest point on the surface of this shape to a given point (in local space)
    pub fn closest_point(&self, point: Vec3) -> Vec3 {
        match self {
            CollisionShape::Box { half_extents } => {
                // Check if point is inside box
                let inside_x = point.x.abs() <= half_extents.x;
                let inside_y = point.y.abs() <= half_extents.y;
                let inside_z = point.z.abs() <= half_extents.z;
                
                if inside_x && inside_y && inside_z {
                    // Point is inside - find closest face
                    let dist_x = half_extents.x - point.x.abs();
                    let dist_y = half_extents.y - point.y.abs();
                    let dist_z = half_extents.z - point.z.abs();
                    
                    let min_dist = dist_x.min(dist_y).min(dist_z);
                    
                    if min_dist == dist_x {
                        Vec3::new(
                            if point.x > 0.0 { half_extents.x } else { -half_extents.x },
                            point.y,
                            point.z,
                        )
                    } else if min_dist == dist_y {
                        Vec3::new(
                            point.x,
                            if point.y > 0.0 { half_extents.y } else { -half_extents.y },
                            point.z,
                        )
                    } else {
                        Vec3::new(
                            point.x,
                            point.y,
                            if point.z > 0.0 { half_extents.z } else { -half_extents.z },
                        )
                    }
                } else {
                    // Point is outside - clamp to box bounds
                    Vec3::new(
                        point.x.clamp(-half_extents.x, half_extents.x),
                        point.y.clamp(-half_extents.y, half_extents.y),
                        point.z.clamp(-half_extents.z, half_extents.z),
                    )
                }
            }
            CollisionShape::Sphere { radius } => {
                let len = point.length();
                if len > 0.0001 {
                    // Project point onto sphere surface
                    point * (*radius / len)
                } else {
                    // Point is at center, return any point on surface
                    Vec3::new(*radius, 0.0, 0.0)
                }
            }
            CollisionShape::Cylinder { radius, half_height } => {
                let xz_len = (point.x * point.x + point.z * point.z).sqrt();
                
                // Check if point is inside cylinder
                let inside_radius = xz_len <= *radius;
                let inside_height = point.y.abs() <= *half_height;
                
                if inside_radius && inside_height {
                    // Point is inside cylinder - find closest surface
                    let dist_to_side = *radius - xz_len;
                    let dist_to_top = *half_height - point.y.abs();
                    
                    if dist_to_side < dist_to_top {
                        // Closer to side - project to cylindrical surface
                        if xz_len > 0.0001 {
                            let scale = *radius / xz_len;
                            Vec3::new(point.x * scale, point.y, point.z * scale)
                        } else {
                            Vec3::new(*radius, point.y, 0.0)
                        }
                    } else {
                        // Closer to top/bottom cap
                        let cap_y = if point.y > 0.0 { *half_height } else { -*half_height };
                        Vec3::new(point.x, cap_y, point.z)
                    }
                } else {
                    // Point is outside cylinder
                    let clamped_y = point.y.clamp(-*half_height, *half_height);
                    
                    if xz_len > 0.0001 {
                        let scale = *radius / xz_len;
                        Vec3::new(point.x * scale, clamped_y, point.z * scale)
                    } else {
                        Vec3::new(*radius, clamped_y, 0.0)
                    }
                }
            }
            CollisionShape::Plane { normal } => {
                let dist = point.dot(*normal);
                point - *normal * dist.max(0.0)
            }
        }
    }
}

/// Bundle for spawning rigid bodies with collision shapes
#[derive(Bundle)]
pub struct RigidBodyBundle {
    pub rigid_body: RigidBody,
    pub collision_shape: CollisionShape,
    pub transform: Transform,
}

impl RigidBodyBundle {
    /// Create a static box rigid body
    pub fn static_box(position: Vec3, rotation: Quat, width: f32, height: f32, depth: f32) -> Self {
        Self {
            rigid_body: RigidBody::static_body(),
            collision_shape: CollisionShape::box_shape(width, height, depth),
            transform: Transform::from_translation(position).with_rotation(rotation),
        }
    }
    
    /// Create a static sphere rigid body
    pub fn static_sphere(position: Vec3, radius: f32) -> Self {
        Self {
            rigid_body: RigidBody::static_body(),
            collision_shape: CollisionShape::sphere(radius),
            transform: Transform::from_translation(position),
        }
    }
    
    /// Create a static cylinder rigid body
    pub fn static_cylinder(position: Vec3, rotation: Quat, radius: f32, height: f32) -> Self {
        Self {
            rigid_body: RigidBody::static_body(),
            collision_shape: CollisionShape::cylinder(radius, height),
            transform: Transform::from_translation(position).with_rotation(rotation),
        }
    }
}