use glam::Vec3;

#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub t_min: f32,
    pub t_max: f32,
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
            t_min: 0.0001,
            t_max: f32::MAX,
        }
    }

    pub fn with_range(origin: Vec3, direction: Vec3, t_min: f32, t_max: f32) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
            t_min,
            t_max,
        }
    }

    pub fn at(&self, t: f32) -> Vec3 {
        self.origin + self.direction * t
    }

    pub fn between_points(from: Vec3, to: Vec3) -> Self {
        let direction = to - from;
        let length = direction.length();
        if length < 0.0001 {
            return Self {
                origin: from,
                direction: Vec3::Y,
                t_min: 0.0,
                t_max: 0.0,
            };
        }
        Self {
            origin: from,
            direction: direction / length,
            t_min: 0.0001,
            t_max: length,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    pub t: f32,
    pub point: Vec3,
    pub normal: Vec3,
}