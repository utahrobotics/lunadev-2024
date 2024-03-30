use core::{fmt::Debug, ops::Deref};

use ordered_float::NotNan;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Steering {
    pub left: NotNan<f32>,
    pub right: NotNan<f32>,
}

impl Debug for Steering {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Steering")
            .field("left", self.left.deref())
            .field("right", self.right.deref())
            .finish()
    }
}

impl Steering {
    /// Shorthand to make this struct if you know the given values are not `NaN`.
    ///
    /// # Panics
    /// Panics if either left or right are `NaN`. To handle this possibility gracefully,
    /// you should just construct this struct normally as the fields are public.
    pub fn new(left: f32, right: f32) -> Self {
        Self {
            left: NotNan::new(left).unwrap(),
            right: NotNan::new(right).unwrap(),
        }
    }

    pub fn from_drive_and_steering(drive: NotNan<f32>, steering: NotNan<f32>) -> Self {
        let mut left = drive;
        let mut right = drive;

        if steering.into_inner() > 0.0 {
            right *= (0.5 - steering.into_inner()) * 2.0;
        } else {
            left *= (0.5 + steering.into_inner()) * 2.0;
        }

        Self { left, right }
    }
}