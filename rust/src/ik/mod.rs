//! Inverse Kinematics module
//!
//! This module contains the core IK types and FABRIK solver implementation.

pub mod chain;
pub mod constraint;
pub mod joint;
pub mod solver;

pub use chain::{Chain, ChainBuilder};
pub use constraint::{BallSocketConstraint, Constraint};
pub use joint::Joint;
pub use solver::{FabrikSolver, SolveResult};