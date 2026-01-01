//! # ik-webgpu
//!
//! A clean, extensible 3D inverse kinematics library based on the FABRIK algorithm
//! with WebGPU rendering support.
//!
//! ## Features
//! - FABRIK (Forward And Backward Reaching Inverse Kinematics) solver
//! - Constraint system (ball-socket, extensible via traits)
//! - WebGPU-based debug visualization
//! - Cross-platform: Native + WASM support
//!
//! ## Example
//! ```rust,ignore
//! use ik_webgpu::ik::{Chain, FabrikSolver, BallSocketConstraint};
//! use glam::Vec3;
//!
//! // Build an IK chain
//! let mut chain = Chain::builder()
//!     .add_joint(Vec3::ZERO)
//!     .add_joint_with_constraint(Vec3::Y, BallSocketConstraint::new(45.0))
//!     .add_joint(Vec3::new(0.0, 2.0, 0.0))
//!     .tolerance(0.001)
//!     .max_iterations(10)
//!     .build();
//!
//! // Solve for target
//! let target = Vec3::new(1.0, 1.5, 0.0);
//! let result = FabrikSolver::solve(&mut chain, target);
//! println!("Converged: {}, iterations: {}", result.converged, result.iterations);
//! ```

pub mod collision;
pub mod dynamics;
pub mod ik;
pub mod math;
pub mod render;

#[cfg(target_arch = "wasm32")]
pub mod web;

pub use collision::{
    AabbObstacle, CollisionConfig, CollisionHit, CollisionResponse, Obstacle, ObstacleShape,
    ObstacleWorld, Ray, RayHit, SphereObstacle,
};
pub use dynamics::{Interpolatable, SecondOrderDynamics, SpringPreset};
pub use ik::constraint::{BallSocketConstraint, Constraint};
pub use ik::{Chain, ChainBuilder, FabrikSolver, Joint, SolveResult};
pub use math::Transform;
