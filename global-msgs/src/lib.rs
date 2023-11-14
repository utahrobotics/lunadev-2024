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


impl Steering {
    pub fn to_left_right_drive(self) -> (f32, f32) {
        let mut left_drive = self.drive.into_inner();
        let mut right_drive = self.drive.into_inner();
    
        if self.steering.into_inner() > 0.0 {
            right_drive *= (0.5 - self.steering.into_inner()) * 2.0;
        } else {
            left_drive *= (0.5 + self.steering.into_inner()) * 2.0;
    }
        (left_drive, right_drive)
    }
}
