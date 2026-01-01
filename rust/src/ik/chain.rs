use super::constraint::Constraint;
use super::joint::Joint;
use glam::Vec3;

#[derive(Debug, Clone)]
pub struct Chain {
    pub(crate) joints: Vec<Joint>,
    pub(crate) bone_lengths: Vec<f32>,
    pub(crate) tolerance: f32,
    pub(crate) max_iterations: u32,
}

impl Chain {
    pub fn builder() -> ChainBuilder {
        ChainBuilder::new()
    }

    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    pub fn joints_mut(&mut self) -> &mut [Joint] {
        &mut self.joints
    }

    pub fn bone_lengths(&self) -> &[f32] {
        &self.bone_lengths
    }

    pub fn total_length(&self) -> f32 {
        self.bone_lengths.iter().sum()
    }

    pub fn tolerance(&self) -> f32 {
        self.tolerance
    }

    pub fn max_iterations(&self) -> u32 {
        self.max_iterations
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    pub fn end_effector(&self) -> Option<Vec3> {
        self.joints.last().map(|j| j.position)
    }

    pub fn base(&self) -> Option<Vec3> {
        self.joints.first().map(|j| j.position)
    }

    pub fn positions(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.joints.iter().map(|j| j.position)
    }
}

pub struct ChainBuilder {
    joints: Vec<Joint>,
    tolerance: f32,
    max_iterations: u32,
}

impl ChainBuilder {
    pub fn new() -> Self {
        Self {
            joints: Vec::new(),
            tolerance: 0.001,
            max_iterations: 10,
        }
    }

    pub fn add_joint(mut self, position: Vec3) -> Self {
        self.joints.push(Joint::new(position));
        self
    }

    pub fn add_joint_with_constraint<C: Constraint + 'static>(
        mut self,
        position: Vec3,
        constraint: C,
    ) -> Self {
        self.joints.push(Joint::new(position).with_constraint(constraint));
        self
    }

    pub fn tolerance(mut self, tolerance: f32) -> Self {
        self.tolerance = tolerance;
        self
    }

    pub fn max_iterations(mut self, max_iterations: u32) -> Self {
        self.max_iterations = max_iterations;
        self
    }

    pub fn build(self) -> Chain {
        let bone_lengths = self.calculate_bone_lengths();
        Chain {
            joints: self.joints,
            bone_lengths,
            tolerance: self.tolerance,
            max_iterations: self.max_iterations,
        }
    }

    fn calculate_bone_lengths(&self) -> Vec<f32> {
        if self.joints.len() < 2 {
            return Vec::new();
        }

        self.joints
            .windows(2)
            .map(|w| (w[1].position - w[0].position).length())
            .collect()
    }
}

impl Default for ChainBuilder {
    fn default() -> Self {
        Self::new()
    }
}