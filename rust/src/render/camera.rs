use glam::{Mat4, Vec3, Vec4};

pub struct Camera {
    pub position: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    pub fov: f32,
    pub aspect: f32,
    pub near: f32,
    pub far: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            position: Vec3::new(0.0, 2.0, 5.0),
            target: Vec3::ZERO,
            up: Vec3::Y,
            fov: 45.0_f32.to_radians(),
            aspect: 1.0,
            near: 0.1,
            far: 100.0,
        }
    }
}

impl Camera {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.position, self.target, self.up)
    }

    pub fn projection_matrix(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov, self.aspect, self.near, self.far)
    }

    pub fn view_projection(&self) -> Mat4 {
        self.projection_matrix() * self.view_matrix()
    }

    pub fn set_aspect(&mut self, aspect: f32) {
        self.aspect = aspect;
    }

    pub fn forward(&self) -> Vec3 {
        (self.target - self.position).normalize()
    }

    pub fn right(&self) -> Vec3 {
        self.forward().cross(self.up).normalize()
    }

    pub fn screen_to_ray(&self, ndc_x: f32, ndc_y: f32) -> (Vec3, Vec3) {
        let inv_view_proj = self.view_projection().inverse();

        let near_point = inv_view_proj * Vec4::new(ndc_x, ndc_y, 0.0, 1.0);
        let far_point = inv_view_proj * Vec4::new(ndc_x, ndc_y, 1.0, 1.0);

        let near = near_point.truncate() / near_point.w;
        let far = far_point.truncate() / far_point.w;

        let direction = (far - near).normalize();
        (near, direction)
    }

    pub fn screen_to_world_on_plane(&self, ndc_x: f32, ndc_y: f32, plane_normal: Vec3, plane_d: f32) -> Option<Vec3> {
        let (ray_origin, ray_dir) = self.screen_to_ray(ndc_x, ndc_y);

        let denom = ray_dir.dot(plane_normal);
        if denom.abs() > 0.0001 {
            let t = -(ray_origin.dot(plane_normal) + plane_d) / denom;
            if t >= 0.0 {
                return Some(ray_origin + ray_dir * t);
            }
        }
        None
    }
}

pub struct OrbitController {
    pub center: Vec3,
    pub radius: f32,
    pub theta: f32,
    pub phi: f32,
    pub min_radius: f32,
    pub max_radius: f32,
    pub min_phi: f32,
    pub max_phi: f32,
    pub rotate_speed: f32,
    pub pan_speed: f32,
    pub zoom_speed: f32,
    pub damping: f32,
    velocity_theta: f32,
    velocity_phi: f32,
    velocity_radius: f32,
    velocity_pan: Vec3,
}

impl Default for OrbitController {
    fn default() -> Self {
        Self {
            center: Vec3::ZERO,
            radius: 5.0,
            theta: 0.0,
            phi: std::f32::consts::FRAC_PI_4,
            min_radius: 0.5,
            max_radius: 100.0,
            min_phi: 0.05,
            max_phi: std::f32::consts::PI - 0.05,
            rotate_speed: 0.005,
            pan_speed: 0.005,
            zoom_speed: 0.1,
            damping: 0.85,
            velocity_theta: 0.0,
            velocity_phi: 0.0,
            velocity_radius: 0.0,
            velocity_pan: Vec3::ZERO,
        }
    }
}

impl OrbitController {
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self {
            center,
            radius,
            ..Default::default()
        }
    }

    pub fn rotate(&mut self, delta_x: f32, delta_y: f32) {
        self.velocity_theta -= delta_x * self.rotate_speed;
        self.velocity_phi -= delta_y * self.rotate_speed;
    }

    pub fn zoom(&mut self, delta: f32) {
        self.velocity_radius -= delta * self.zoom_speed * self.radius;
    }

    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let right = Vec3::new(self.theta.cos(), 0.0, -self.theta.sin());
        let up_dir = Vec3::new(
            -self.phi.cos() * self.theta.sin(),
            self.phi.sin(),
            -self.phi.cos() * self.theta.cos(),
        ).normalize();

        let pan_factor = self.pan_speed * self.radius;
        self.velocity_pan += right * delta_x * pan_factor + up_dir * delta_y * pan_factor;
    }

    pub fn move_forward(&mut self, delta: f32) {
        let forward = Vec3::new(
            self.phi.sin() * self.theta.cos(),
            self.phi.cos(),
            self.phi.sin() * self.theta.sin(),
        ).normalize();
        self.velocity_pan -= forward * delta * self.pan_speed * self.radius * 10.0;
    }

    pub fn move_right(&mut self, delta: f32) {
        let right = Vec3::new(self.theta.cos(), 0.0, -self.theta.sin());
        self.velocity_pan += right * delta * self.pan_speed * self.radius * 10.0;
    }

    pub fn move_up(&mut self, delta: f32) {
        self.velocity_pan += Vec3::Y * delta * self.pan_speed * self.radius * 10.0;
    }

    pub fn focus_on(&mut self, point: Vec3, distance: Option<f32>) {
        self.center = point;
        if let Some(d) = distance {
            self.radius = d.clamp(self.min_radius, self.max_radius);
        }
        self.velocity_theta = 0.0;
        self.velocity_phi = 0.0;
        self.velocity_radius = 0.0;
        self.velocity_pan = Vec3::ZERO;
    }

    pub fn reset(&mut self) {
        self.center = Vec3::ZERO;
        self.radius = 5.0;
        self.theta = 0.0;
        self.phi = std::f32::consts::FRAC_PI_4;
        self.velocity_theta = 0.0;
        self.velocity_phi = 0.0;
        self.velocity_radius = 0.0;
        self.velocity_pan = Vec3::ZERO;
    }

    pub fn update(&mut self) {
        self.theta += self.velocity_theta;
        self.phi = (self.phi + self.velocity_phi).clamp(self.min_phi, self.max_phi);
        self.radius = (self.radius + self.velocity_radius).clamp(self.min_radius, self.max_radius);
        self.center += self.velocity_pan;

        self.velocity_theta *= self.damping;
        self.velocity_phi *= self.damping;
        self.velocity_radius *= self.damping;
        self.velocity_pan *= self.damping;

        if self.velocity_theta.abs() < 0.0001 {
            self.velocity_theta = 0.0;
        }
        if self.velocity_phi.abs() < 0.0001 {
            self.velocity_phi = 0.0;
        }
        if self.velocity_radius.abs() < 0.0001 {
            self.velocity_radius = 0.0;
        }
        if self.velocity_pan.length_squared() < 0.000001 {
            self.velocity_pan = Vec3::ZERO;
        }
    }

    pub fn camera_position(&self) -> Vec3 {
        let x = self.radius * self.phi.sin() * self.theta.cos();
        let y = self.radius * self.phi.cos();
        let z = self.radius * self.phi.sin() * self.theta.sin();
        self.center + Vec3::new(x, y, z)
    }

    pub fn update_camera(&self, camera: &mut Camera) {
        camera.position = self.camera_position();
        camera.target = self.center;
    }
}

pub struct CameraController {
    pub orbit: OrbitController,
    pub left_mouse_action: MouseAction,
    pub right_mouse_action: MouseAction,
    pub middle_mouse_action: MouseAction,
    left_pressed: bool,
    right_pressed: bool,
    middle_pressed: bool,
    last_mouse_pos: (f32, f32),
    keys_pressed: [bool; 6],
}

#[derive(Clone, Copy, PartialEq)]
pub enum MouseAction {
    None,
    Orbit,
    Pan,
    Zoom,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            orbit: OrbitController::default(),
            left_mouse_action: MouseAction::Orbit,
            right_mouse_action: MouseAction::Pan,
            middle_mouse_action: MouseAction::Zoom,
            left_pressed: false,
            right_pressed: false,
            middle_pressed: false,
            last_mouse_pos: (0.0, 0.0),
            keys_pressed: [false; 6],
        }
    }
}

impl CameraController {
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self {
            orbit: OrbitController::new(center, radius),
            ..Default::default()
        }
    }

    pub fn on_mouse_button(&mut self, button: u8, pressed: bool) {
        match button {
            0 => self.left_pressed = pressed,
            1 => self.right_pressed = pressed,
            2 => self.middle_pressed = pressed,
            _ => {}
        }
    }

    pub fn on_mouse_move(&mut self, x: f32, y: f32) {
        let dx = x - self.last_mouse_pos.0;
        let dy = y - self.last_mouse_pos.1;
        self.last_mouse_pos = (x, y);

        let action = if self.left_pressed {
            self.left_mouse_action
        } else if self.right_pressed {
            self.right_mouse_action
        } else if self.middle_pressed {
            self.middle_mouse_action
        } else {
            MouseAction::None
        };

        match action {
            MouseAction::Orbit => self.orbit.rotate(dx, dy),
            MouseAction::Pan => self.orbit.pan(-dx, dy),
            MouseAction::Zoom => self.orbit.zoom(dy * 0.1),
            MouseAction::None => {}
        }
    }

    pub fn on_scroll(&mut self, delta: f32) {
        self.orbit.zoom(delta);
    }

    pub fn on_key(&mut self, key: Key, pressed: bool) {
        let idx = match key {
            Key::W => 0,
            Key::S => 1,
            Key::A => 2,
            Key::D => 3,
            Key::Q => 4,
            Key::E => 5,
        };
        self.keys_pressed[idx] = pressed;
    }

    pub fn update(&mut self, camera: &mut Camera) {
        if self.keys_pressed[0] { self.orbit.move_forward(1.0); }
        if self.keys_pressed[1] { self.orbit.move_forward(-1.0); }
        if self.keys_pressed[2] { self.orbit.move_right(-1.0); }
        if self.keys_pressed[3] { self.orbit.move_right(1.0); }
        if self.keys_pressed[4] { self.orbit.move_up(-1.0); }
        if self.keys_pressed[5] { self.orbit.move_up(1.0); }

        self.orbit.update();
        self.orbit.update_camera(camera);
    }

    pub fn focus_on(&mut self, point: Vec3, distance: Option<f32>) {
        self.orbit.focus_on(point, distance);
    }

    pub fn reset(&mut self) {
        self.orbit.reset();
    }
}

#[derive(Clone, Copy)]
pub enum Key {
    W, S, A, D, Q, E,
}
