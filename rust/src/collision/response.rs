use glam::Vec3;

use super::world::ObstacleWorld;
use crate::ik::Chain;

#[derive(Debug, Clone, Copy)]
pub struct CollisionConfig {
    pub margin: f32,
    pub max_iterations: u32,
    pub preserve_bone_lengths: bool,
}

impl Default for CollisionConfig {
    fn default() -> Self {
        Self {
            margin: 0.05,
            max_iterations: 4,
            preserve_bone_lengths: true,
        }
    }
}

impl CollisionConfig {
    pub fn new(margin: f32) -> Self {
        Self {
            margin,
            ..Default::default()
        }
    }

    pub fn with_iterations(mut self, iterations: u32) -> Self {
        self.max_iterations = iterations;
        self
    }

    pub fn with_preserve_bone_lengths(mut self, preserve: bool) -> Self {
        self.preserve_bone_lengths = preserve;
        self
    }
}

pub struct CollisionResponse;

impl CollisionResponse {
    pub fn resolve_chain(chain: &mut Chain, world: &ObstacleWorld, config: &CollisionConfig) {
        if world.is_empty() {
            return;
        }

        let n = chain.joint_count();
        if n < 2 {
            return;
        }

        for _ in 0..config.max_iterations {
            let mut any_collision = false;

            {
                let joints = chain.joints_mut();
                for i in 1..n {
                    let old_pos = joints[i].position;
                    let new_pos = world.push_out_point(old_pos, config.margin);

                    if (new_pos - old_pos).length_squared() > 0.0001 {
                        joints[i].position = new_pos;
                        any_collision = true;
                    }
                }
            }

            if config.preserve_bone_lengths && any_collision {
                Self::fix_bone_lengths(chain);
            }

            if !any_collision {
                break;
            }
        }
    }

    fn fix_bone_lengths(chain: &mut Chain) {
        let bone_lengths: Vec<f32> = chain.bone_lengths().to_vec();
        let joints = chain.joints_mut();
        let n = joints.len();

        for i in 1..n {
            let prev_pos = joints[i - 1].position;
            let curr_pos = joints[i].position;
            let bone_length = bone_lengths[i - 1];

            let dir = curr_pos - prev_pos;
            let len = dir.length();

            let direction = if len > 0.0001 { dir / len } else { Vec3::Y };

            joints[i].position = prev_pos + direction * bone_length;
        }
    }

    pub fn has_collision(chain: &Chain, world: &ObstacleWorld) -> bool {
        chain
            .joints()
            .iter()
            .any(|j| world.point_inside_any(j.position))
    }

    pub fn colliding_joints(chain: &Chain, world: &ObstacleWorld) -> Vec<usize> {
        chain
            .joints()
            .iter()
            .enumerate()
            .filter(|(_, j)| world.point_inside_any(j.position))
            .map(|(i, _)| i)
            .collect()
    }

    pub fn get_collision_hits(
        chain: &Chain,
        world: &ObstacleWorld,
        margin: f32,
    ) -> Vec<CollisionHit> {
        let mut hits = Vec::new();

        for joint in chain.joints().iter().skip(1) {
            let pos = joint.position;
            let pushed = world.push_out_point(pos, margin);

            if (pushed - pos).length_squared() > 0.0001 {
                if let Some((idx, _)) = world.closest_obstacle(pos) {
                    let obstacles = world.obstacles();
                    let obstacle = &obstacles[idx];
                    let surface_point = obstacle.closest_surface_point(pos);
                    let normal = obstacle.surface_normal(surface_point);

                    hits.push(CollisionHit {
                        original: pos,
                        pushed: pushed,
                        surface_point,
                        normal,
                    });
                }
            }
        }

        hits
    }
}

/// Information about a collision hit point
#[derive(Debug, Clone, Copy)]
pub struct CollisionHit {
    /// Original position of the joint
    pub original: Vec3,
    /// Position after being pushed out
    pub pushed: Vec3,
    /// Closest point on obstacle surface
    pub surface_point: Vec3,
    /// Surface normal at hit point
    pub normal: Vec3,
}