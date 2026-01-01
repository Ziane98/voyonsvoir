use glam::Vec3;

use super::obstacle::{AabbObstacle, Obstacle, SphereObstacle};
use super::raycast::{Ray, RayHit};

#[derive(Default, Clone)]
pub struct ObstacleWorld {
    obstacles: Vec<Box<dyn Obstacle>>,
}

impl ObstacleWorld {
    pub fn new() -> Self {
        Self {
            obstacles: Vec::new(),
        }
    }

    pub fn add<T: Obstacle + 'static>(&mut self, obstacle: T) {
        self.obstacles.push(Box::new(obstacle));
    }

    pub fn add_sphere(&mut self, center: Vec3, radius: f32) {
        self.obstacles
            .push(Box::new(SphereObstacle::new(center, radius)));
    }

    pub fn add_box(&mut self, center: Vec3, half_extents: Vec3) {
        self.obstacles
            .push(Box::new(AabbObstacle::from_center_half_extents(
                center,
                half_extents,
            )));
    }

    pub fn add_aabb(&mut self, min: Vec3, max: Vec3) {
        self.obstacles.push(Box::new(AabbObstacle::new(min, max)));
    }

    pub fn clear(&mut self) {
        self.obstacles.clear();
    }

    pub fn obstacles(&self) -> &[Box<dyn Obstacle>] {
        &self.obstacles
    }

    pub fn obstacle_count(&self) -> usize {
        self.obstacles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.obstacles.is_empty()
    }

    pub fn point_inside_any(&self, point: Vec3) -> bool {
        self.obstacles.iter().any(|o| o.contains_point(point))
    }

    pub fn closest_obstacle(&self, point: Vec3) -> Option<(usize, f32)> {
        self.obstacles
            .iter()
            .enumerate()
            .map(|(i, o)| (i, o.signed_distance(point)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }

    pub fn raycast(&self, ray: &Ray) -> Option<(usize, RayHit)> {
        let mut closest: Option<(usize, RayHit)> = None;

        for (i, obstacle) in self.obstacles.iter().enumerate() {
            if let Some(hit) = obstacle.ray_intersect(ray) {
                match &closest {
                    None => closest = Some((i, hit)),
                    Some((_, prev_hit)) if hit.t < prev_hit.t => {
                        closest = Some((i, hit));
                    }
                    _ => {}
                }
            }
        }

        closest
    }

    pub fn push_out_point(&self, point: Vec3, margin: f32) -> Vec3 {
        let mut result = point;

        for _ in 0..4 {
            let mut pushed = false;
            for obstacle in &self.obstacles {
                if obstacle.signed_distance(result) < margin {
                    result = obstacle.push_out(result, margin);
                    pushed = true;
                }
            }
            if !pushed {
                break;
            }
        }

        result
    }
}

impl std::fmt::Debug for ObstacleWorld {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ObstacleWorld")
            .field("obstacle_count", &self.obstacles.len())
            .finish()
    }
}