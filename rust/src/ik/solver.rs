use super::chain::Chain;
use crate::collision::{CollisionConfig, CollisionResponse, ObstacleWorld};
use glam::Vec3;

#[derive(Debug, Clone, Copy)]
pub struct SolveResult {
    pub converged: bool,
    pub iterations: u32,
    pub final_distance: f32,
}

pub struct FabrikSolver;

impl FabrikSolver {
    pub fn solve(chain: &mut Chain, target: Vec3) -> SolveResult {
        if chain.joints.is_empty() {
            return SolveResult {
                converged: true,
                iterations: 0,
                final_distance: 0.0,
            };
        }

        let base = chain.joints[0].position;
        Self::solve_anchored(chain, target, base)
    }

    pub fn solve_anchored(chain: &mut Chain, target: Vec3, base: Vec3) -> SolveResult {
        let joint_count = chain.joints.len();

        if joint_count < 2 {
            return SolveResult {
                converged: true,
                iterations: 0,
                final_distance: 0.0,
            };
        }

        let total_length = chain.total_length();
        let distance_to_target = (target - base).length();

        if distance_to_target > total_length {
            Self::stretch_towards_target(chain, base, target);
            return SolveResult {
                converged: false,
                iterations: 1,
                final_distance: distance_to_target - total_length,
            };
        }

        let tolerance = chain.tolerance;
        let max_iterations = chain.max_iterations;

        for iteration in 0..max_iterations {
            Self::forward_pass(chain, target);
            Self::backward_pass(chain, base);

            let end_effector = chain.joints.last().unwrap().position;
            let distance = (end_effector - target).length();

            if distance <= tolerance {
                return SolveResult {
                    converged: true,
                    iterations: iteration + 1,
                    final_distance: distance,
                };
            }
        }

        let final_distance = (chain.joints.last().unwrap().position - target).length();
        SolveResult {
            converged: final_distance <= tolerance,
            iterations: max_iterations,
            final_distance,
        }
    }

    fn forward_pass(chain: &mut Chain, target: Vec3) {
        let n = chain.joints.len();

        chain.joints[n - 1].position = target;

        for i in (0..n - 1).rev() {
            let next_pos = chain.joints[i + 1].position;
            let curr_pos = chain.joints[i].position;
            let bone_length = chain.bone_lengths[i];

            let dir = curr_pos - next_pos;
            let len = dir.length();

            let direction = if len > 0.0001 {
                dir / len
            } else {
                Vec3::Y
            };

            chain.joints[i].position = next_pos + direction * bone_length;
        }
    }

    fn backward_pass(chain: &mut Chain, base: Vec3) {
        let n = chain.joints.len();

        chain.joints[0].position = base;

        for i in 1..n {
            let prev_pos = chain.joints[i - 1].position;
            let curr_pos = chain.joints[i].position;
            let bone_length = chain.bone_lengths[i - 1];

            let dir = curr_pos - prev_pos;
            let len = dir.length();

            let direction = if len > 0.0001 {
                dir / len
            } else {
                Vec3::Y
            };

            chain.joints[i].position = prev_pos + direction * bone_length;
        }
    }

    fn stretch_towards_target(chain: &mut Chain, base: Vec3, target: Vec3) {
        let direction = (target - base).normalize_or_zero();

        if direction.length_squared() < 0.0001 {
            return;
        }

        chain.joints[0].position = base;

        for i in 1..chain.joints.len() {
            let prev = chain.joints[i - 1].position;
            let bone_length = chain.bone_lengths[i - 1];
            chain.joints[i].position = prev + direction * bone_length;
        }
    }

    pub fn solve_with_collision(
        chain: &mut Chain,
        target: Vec3,
        world: &ObstacleWorld,
        config: &CollisionConfig,
    ) -> SolveResult {
        if chain.joints.is_empty() {
            return SolveResult {
                converged: true,
                iterations: 0,
                final_distance: 0.0,
            };
        }

        let base = chain.joints[0].position;
        Self::solve_anchored_with_collision(chain, target, base, world, config)
    }

    pub fn solve_anchored_with_collision(
        chain: &mut Chain,
        target: Vec3,
        base: Vec3,
        world: &ObstacleWorld,
        config: &CollisionConfig,
    ) -> SolveResult {
        let joint_count = chain.joints.len();

        if joint_count < 2 {
            return SolveResult {
                converged: true,
                iterations: 0,
                final_distance: 0.0,
            };
        }

        let total_length = chain.total_length();
        let distance_to_target = (target - base).length();

        if distance_to_target > total_length {
            Self::stretch_towards_target(chain, base, target);
            CollisionResponse::resolve_chain(chain, world, config);
            return SolveResult {
                converged: false,
                iterations: 1,
                final_distance: distance_to_target - total_length,
            };
        }

        let tolerance = chain.tolerance;
        let max_iterations = chain.max_iterations;

        for iteration in 0..max_iterations {
            Self::forward_pass(chain, target);
            Self::backward_pass(chain, base);

            CollisionResponse::resolve_chain(chain, world, config);

            chain.joints[0].position = base;
            Self::fix_bone_lengths_forward(chain);

            let end_effector = chain.joints.last().unwrap().position;
            let distance = (end_effector - target).length();

            if distance <= tolerance {
                return SolveResult {
                    converged: true,
                    iterations: iteration + 1,
                    final_distance: distance,
                };
            }
        }

        let final_distance = (chain.joints.last().unwrap().position - target).length();
        SolveResult {
            converged: final_distance <= tolerance,
            iterations: max_iterations,
            final_distance,
        }
    }

    fn fix_bone_lengths_forward(chain: &mut Chain) {
        let n = chain.joints.len();
        for i in 1..n {
            let prev_pos = chain.joints[i - 1].position;
            let curr_pos = chain.joints[i].position;
            let bone_length = chain.bone_lengths[i - 1];

            let dir = curr_pos - prev_pos;
            let len = dir.length();

            let direction = if len > 0.0001 { dir / len } else { Vec3::Y };

            chain.joints[i].position = prev_pos + direction * bone_length;
        }
    }
}
