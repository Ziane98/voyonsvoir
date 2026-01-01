use glam::Vec3;
use std::f32::consts::PI;

pub trait Interpolatable: Clone + Copy {
    fn zero() -> Self;
    fn add(self, other: Self) -> Self;
    fn sub(self, other: Self) -> Self;
    fn scale(self, factor: f32) -> Self;
}

impl Interpolatable for f32 {
    fn zero() -> Self { 0.0 }
    fn add(self, other: Self) -> Self { self + other }
    fn sub(self, other: Self) -> Self { self - other }
    fn scale(self, factor: f32) -> Self { self * factor }
}

impl Interpolatable for Vec3 {
    fn zero() -> Self { Vec3::ZERO }
    fn add(self, other: Self) -> Self { self + other }
    fn sub(self, other: Self) -> Self { self - other }
    fn scale(self, factor: f32) -> Self { self * factor }
}

#[derive(Clone, Copy)]
pub enum SpringPreset {
    Snappy,
    Smooth,
    Bouncy,
    Sluggish,
    Anticipate,
}

impl SpringPreset {
    pub fn params(self) -> (f32, f32, f32) {
        match self {
            SpringPreset::Snappy => (4.0, 0.5, 2.0),
            SpringPreset::Smooth => (2.0, 1.0, 0.0),
            SpringPreset::Bouncy => (3.0, 0.3, 1.0),
            SpringPreset::Sluggish => (1.0, 1.5, 0.0),
            SpringPreset::Anticipate => (3.0, 0.8, -0.5),
        }
    }
}

pub struct SecondOrderDynamics<T: Interpolatable> {
    y: T,
    yd: T,
    xp: T,
    k1: f32,
    k2: f32,
    k3: f32,
}

impl<T: Interpolatable> SecondOrderDynamics<T> {
    pub fn new(f: f32, z: f32, r: f32, initial: T) -> Self {
        let (k1, k2, k3) = Self::compute_constants(f, z, r);
        Self {
            y: initial,
            yd: T::zero(),
            xp: initial,
            k1,
            k2,
            k3,
        }
    }

    pub fn from_preset(preset: SpringPreset, initial: T) -> Self {
        let (f, z, r) = preset.params();
        Self::new(f, z, r, initial)
    }

    fn compute_constants(f: f32, z: f32, r: f32) -> (f32, f32, f32) {
        let w = 2.0 * PI * f;
        let k1 = z / (PI * f);
        let k2 = 1.0 / (w * w);
        let k3 = r * z / (PI * f);
        (k1, k2, k3)
    }

    pub fn set_parameters(&mut self, f: f32, z: f32, r: f32) {
        let (k1, k2, k3) = Self::compute_constants(f, z, r);
        self.k1 = k1;
        self.k2 = k2;
        self.k3 = k3;
    }

    pub fn reset(&mut self, value: T) {
        self.y = value;
        self.yd = T::zero();
        self.xp = value;
    }

    pub fn update(&mut self, x: T, dt: f32) -> T {
        if dt <= 0.0 {
            return self.y;
        }

        let xd = x.sub(self.xp).scale(1.0 / dt);
        self.xp = x;

        let k2_stable = self.k2.max(
            (dt * dt / 2.0 + dt * self.k1 / 2.0).max(dt * self.k1)
        );

        self.y = self.y.add(self.yd.scale(dt));

        let accel = x.add(xd.scale(self.k3))
            .sub(self.y)
            .sub(self.yd.scale(self.k1))
            .scale(1.0 / k2_stable);

        self.yd = self.yd.add(accel.scale(dt));

        self.y
    }

    pub fn current(&self) -> T {
        self.y
    }

    pub fn velocity(&self) -> T {
        self.yd
    }
}
