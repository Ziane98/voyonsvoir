use super::constraint::Constraint;
use glam::Vec3;

#[derive(Debug, Clone)]
pub struct Joint {
    pub position: Vec3,
    pub constraint: Option<Box<dyn Constraint>>,
}

impl Joint {
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            constraint: None,
        }
    }

    pub fn with_constraint<C: Constraint + 'static>(mut self, constraint: C) -> Self {
        self.constraint = Some(Box::new(constraint));
        self
    }

    pub fn set_constraint<C: Constraint + 'static>(&mut self, constraint: C) {
        self.constraint = Some(Box::new(constraint));
    }

    pub fn clear_constraint(&mut self) {
        self.constraint = None;
    }

    pub fn apply_constraint(&self, direction: Vec3, reference: Vec3) -> Vec3 {
        match &self.constraint {
            Some(c) => c.apply(direction, reference),
            None => direction.normalize_or_zero(),
        }
    }
}