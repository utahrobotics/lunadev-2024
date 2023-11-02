#![no_std]

use core::{fmt::Debug, ops::Deref};

use ordered_float::NotNan;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Steering {
    pub drive: NotNan<f32>,
    pub steering: NotNan<f32>,
}

impl Debug for Steering {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Steering")
            .field("drive", self.drive.deref())
            .field("steering", self.steering.deref())
            .finish()
    }
}
