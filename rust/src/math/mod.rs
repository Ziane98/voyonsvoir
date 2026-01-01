//! Math utilities module
//!
//! Provides convenient re-exports from glam and additional transform utilities.

mod transform;

pub use transform::Transform;

// Re-export commonly used glam types
pub use glam::{Mat4, Quat, Vec3, Vec4};