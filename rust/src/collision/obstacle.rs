use glam::Vec3;
use std::fmt::Debug;

use super::raycast::{Ray, RayHit};

#[derive(Debug, Clone, Copy)]
pub enum ObstacleShape {
    Sphere { center: Vec3, radius: f32 },
    Box { center: Vec3, half_extents: Vec3 },
}

pub trait Obstacle: Send + Sync + Debug {
    fn contains_point(&self, point: Vec3) -> bool;
    fn signed_distance(&self, point: Vec3) -> f32;
    fn closest_surface_point(&self, point: Vec3) -> Vec3;
    fn surface_normal(&self, point: Vec3) -> Vec3;
    fn ray_intersect(&self, ray: &Ray) -> Option<RayHit>;
    fn push_out(&self, point: Vec3, margin: f32) -> Vec3;
    fn clone_box(&self) -> Box<dyn Obstacle>;
    fn center(&self) -> Vec3;
    fn render_shape(&self) -> ObstacleShape;
}

impl Clone for Box<dyn Obstacle> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SphereObstacle {
    pub center: Vec3,
    pub radius: f32,
}

impl SphereObstacle {
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }
}

impl Obstacle for SphereObstacle {
    fn contains_point(&self, point: Vec3) -> bool {
        (point - self.center).length_squared() <= self.radius * self.radius
    }

    fn signed_distance(&self, point: Vec3) -> f32 {
        (point - self.center).length() - self.radius
    }

    fn closest_surface_point(&self, point: Vec3) -> Vec3 {
        let dir = (point - self.center).normalize_or_zero();
        if dir.length_squared() < 0.0001 {
            return self.center + Vec3::Y * self.radius;
        }
        self.center + dir * self.radius
    }

    fn surface_normal(&self, point: Vec3) -> Vec3 {
        (point - self.center).normalize_or_zero()
    }

    fn ray_intersect(&self, ray: &Ray) -> Option<RayHit> {
        let oc = ray.origin - self.center;
        let a = ray.direction.dot(ray.direction);
        let b = 2.0 * oc.dot(ray.direction);
        let c = oc.dot(oc) - self.radius * self.radius;
        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            return None;
        }

        let sqrt_d = discriminant.sqrt();
        let t = (-b - sqrt_d) / (2.0 * a);

        if t > ray.t_min && t < ray.t_max {
            let point = ray.at(t);
            let normal = self.surface_normal(point);
            return Some(RayHit { t, point, normal });
        }

        let t = (-b + sqrt_d) / (2.0 * a);
        if t > ray.t_min && t < ray.t_max {
            let point = ray.at(t);
            let normal = self.surface_normal(point);
            return Some(RayHit { t, point, normal });
        }

        None
    }

    fn push_out(&self, point: Vec3, margin: f32) -> Vec3 {
        let dir = (point - self.center).normalize_or_zero();
        if dir.length_squared() < 0.0001 {
            return self.center + Vec3::Y * (self.radius + margin);
        }
        self.center + dir * (self.radius + margin)
    }

    fn clone_box(&self) -> Box<dyn Obstacle> {
        Box::new(*self)
    }

    fn center(&self) -> Vec3 {
        self.center
    }

    fn render_shape(&self) -> ObstacleShape {
        ObstacleShape::Sphere {
            center: self.center,
            radius: self.radius,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AabbObstacle {
    pub min: Vec3,
    pub max: Vec3,
}

impl AabbObstacle {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn from_center_half_extents(center: Vec3, half_extents: Vec3) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    pub fn half_extents(&self) -> Vec3 {
        (self.max - self.min) * 0.5
    }
}

impl Obstacle for AabbObstacle {
    fn contains_point(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    fn signed_distance(&self, point: Vec3) -> f32 {
        let center = self.center();
        let half_extents = self.half_extents();
        let p = point - center;
        let q = p.abs() - half_extents;
        q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0)
    }

    fn closest_surface_point(&self, point: Vec3) -> Vec3 {
        let clamped = point.clamp(self.min, self.max);
        if clamped == point {
            let center = self.center();
            let half_extents = self.half_extents();
            let p = point - center;
            let distances = half_extents - p.abs();

            let min_dist = distances.x.min(distances.y.min(distances.z));
            let mut result = point;

            if (distances.x - min_dist).abs() < 0.0001 {
                result.x = if p.x > 0.0 { self.max.x } else { self.min.x };
            } else if (distances.y - min_dist).abs() < 0.0001 {
                result.y = if p.y > 0.0 { self.max.y } else { self.min.y };
            } else {
                result.z = if p.z > 0.0 { self.max.z } else { self.min.z };
            }
            result
        } else {
            clamped
        }
    }

    fn surface_normal(&self, point: Vec3) -> Vec3 {
        let center = self.center();
        let half_extents = self.half_extents();
        let p = (point - center) / half_extents.max(Vec3::splat(0.0001));

        let abs_p = p.abs();
        if abs_p.x > abs_p.y && abs_p.x > abs_p.z {
            Vec3::X * p.x.signum()
        } else if abs_p.y > abs_p.z {
            Vec3::Y * p.y.signum()
        } else {
            Vec3::Z * p.z.signum()
        }
    }

    fn ray_intersect(&self, ray: &Ray) -> Option<RayHit> {
        let inv_dir = Vec3::new(
            if ray.direction.x.abs() > 0.0001 {
                1.0 / ray.direction.x
            } else {
                f32::MAX
            },
            if ray.direction.y.abs() > 0.0001 {
                1.0 / ray.direction.y
            } else {
                f32::MAX
            },
            if ray.direction.z.abs() > 0.0001 {
                1.0 / ray.direction.z
            } else {
                f32::MAX
            },
        );

        let t1 = (self.min - ray.origin) * inv_dir;
        let t2 = (self.max - ray.origin) * inv_dir;

        let t_min_v = t1.min(t2);
        let t_max_v = t1.max(t2);

        let t_near = t_min_v.x.max(t_min_v.y.max(t_min_v.z));
        let t_far = t_max_v.x.min(t_max_v.y.min(t_max_v.z));

        if t_near > t_far || t_far < ray.t_min {
            return None;
        }

        let t = if t_near > ray.t_min { t_near } else { t_far };

        if t > ray.t_max {
            return None;
        }

        let point = ray.at(t);
        let normal = self.surface_normal(point);

        Some(RayHit { t, point, normal })
    }

    fn push_out(&self, point: Vec3, margin: f32) -> Vec3 {
        let surface_pt = self.closest_surface_point(point);
        let normal = self.surface_normal(surface_pt);
        surface_pt + normal * margin
    }

    fn clone_box(&self) -> Box<dyn Obstacle> {
        Box::new(*self)
    }

    fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    fn render_shape(&self) -> ObstacleShape {
        ObstacleShape::Box {
            center: self.center(),
            half_extents: self.half_extents(),
        }
    }
}