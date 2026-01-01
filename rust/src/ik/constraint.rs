use glam::{Quat, Vec3};
use std::fmt::Debug;

pub trait Constraint: Send + Sync + Debug {
    fn apply(&self, direction: Vec3, reference: Vec3) -> Vec3;
    fn clone_box(&self) -> Box<dyn Constraint>;
}

impl Clone for Box<dyn Constraint> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BallSocketConstraint {
    pub max_angle: f32,
}

impl BallSocketConstraint {
    pub fn new(max_angle_degrees: f32) -> Self {
        Self {
            max_angle: max_angle_degrees.to_radians(),
        }
    }

    pub fn from_radians(max_angle: f32) -> Self {
        Self { max_angle }
    }
}

impl Constraint for BallSocketConstraint {
    fn apply(&self, direction: Vec3, reference: Vec3) -> Vec3 {
        let dir = direction.normalize_or_zero();
        let ref_dir = reference.normalize_or_zero();

        if dir.length_squared() < 0.0001 || ref_dir.length_squared() < 0.0001 {
            return ref_dir;
        }

        let angle = dir.angle_between(ref_dir);

        if angle <= self.max_angle {
            dir
        } else {
            let axis = ref_dir.cross(dir);
            if axis.length_squared() < 0.0001 {
                ref_dir
            } else {
                let axis = axis.normalize();
                Quat::from_axis_angle(axis, self.max_angle) * ref_dir
            }
        }
    }

    fn clone_box(&self) -> Box<dyn Constraint> {
        Box::new(*self)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct NoConstraint;

impl Constraint for NoConstraint {
    fn apply(&self, direction: Vec3, _reference: Vec3) -> Vec3 {
        direction.normalize_or_zero()
    }

    fn clone_box(&self) -> Box<dyn Constraint> {
        Box::new(*self)
    }
}