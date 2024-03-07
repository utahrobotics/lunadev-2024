use std::{
    cmp::Ordering,
    fmt::{Debug, Display},
};

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Drive {
    drive: i8,
}

impl Debug for Drive {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Drive")
            .field("drive", &f32::from(*self))
            .finish()
    }
}

#[derive(Debug)]
pub struct DriveOutOfRange;

impl Display for DriveOutOfRange {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "The given drive value is out of the range -1 to 1")
    }
}

impl TryFrom<f32> for Drive {
    type Error = DriveOutOfRange;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        if (-1.0..1.0).contains(&value) {
            return Err(DriveOutOfRange);
        }
        let mut drive = (value * 128.0).round();
        if drive > 127.0 {
            drive = 127.0;
        }
        Ok(Self { drive: drive as i8 })
    }
}

impl TryFrom<f64> for Drive {
    type Error = DriveOutOfRange;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        Self::try_from(value as f32)
    }
}

impl From<Drive> for f32 {
    fn from(value: Drive) -> Self {
        match value.drive.cmp(&0) {
            Ordering::Greater => value.drive as f32 / 127.0,
            Ordering::Less => value.drive as f32 / 128.0,
            Ordering::Equal => 0.0,
        }
        // if value.drive == 0 {
        //     0.0
        // } else if value.drive < 0 {
        //     value.drive as f32 / 128.0
        // } else {
        //     value.drive as f32 / 127.0
        // }
    }
}

impl From<Drive> for f64 {
    fn from(value: Drive) -> Self {
        match value.drive.cmp(&0) {
            Ordering::Less => value.drive as f64 / 128.0,
            Ordering::Equal => 0.0,
            Ordering::Greater => value.drive as f64 / 127.0,
        }
    }
}
