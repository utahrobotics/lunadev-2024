//! This crate provides a node that can connect to RealSense cameras and interpret
//! depth and color images.
//!
//! Unfortunately, this crate depends on the RealSense SDK. If you do not have this
//! SDK, remove this crate from the workspace.

#[cfg(unix)]
mod implementation;
#[cfg(unix)]
pub use implementation::*;

pub mod iter;
