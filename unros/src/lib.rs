//! Unros is an experimental alternative to the ROS 1 & 2 frameworks.
//!
//! It is written from the ground up in Rust and seeks to replicate most
//! of the common functionality in ROS while adding some extra features
//! that exploit Rust's abilities.
//!
//! This crate contains the core functionality which defines what this
//! framework offers:
//!
//! 1. The Node trait
//! 2. A complete logging system
//! 3. An asynchronous Node runtime
//! 4. Publisher and Subscribers (analagous to ROS publisher and subscribers)
//! 5. The Service framework (analagous to ROS actions and services)

pub use unros_core::*;
pub use unros_macros::*;
