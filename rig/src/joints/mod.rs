use std::sync::atomic::Ordering;

use atomic_float::AtomicF64;
use nalgebra::{UnitVector3, Isometry3, UnitQuaternion};
use serde::{Deserialize, Serialize};


/// A joint that may be moved at runtime
#[derive(Deserialize, Serialize)]
pub enum Joint {
    Fixed,
    Hinge(HingeJoint),
}


impl Default for Joint {
    fn default() -> Self {
        Self::Fixed
    }
}


impl Joint {
    pub(super) fn init(&mut self) {
        match self {
            Joint::Fixed => {}
            Joint::Hinge(joint) => joint.set_angle(joint.starting_angle),
        }
    }

    pub fn get_ref(&self) -> JointRef {
        match self {
            Joint::Fixed => JointRef::Fixed,
            Joint::Hinge(x) => JointRef::Hinge(HingeJointRef(x)),
        }
    }

    pub fn get_isometry(&self) -> Isometry3<f64> {
        match self {
            Joint::Fixed => Isometry3::default(),
            Joint::Hinge(x) => Isometry3::from_parts(Default::default(), UnitQuaternion::from_axis_angle(&x.axis, x.angle.load(Ordering::Acquire))),
        }
    }
}


/// An immutable reference to a joint.
/// 
/// Unlike `RobotElementRef`, this is bounded to the lifetime of
/// the original joint.
pub enum JointRef<'a> {
    Fixed,
    Hinge(HingeJointRef<'a>)
}


#[derive(Deserialize, Serialize)]
pub struct HingeJoint {
    pub axis: UnitVector3<f64>,
    pub starting_angle: f64,
    #[serde(skip)]
    angle: AtomicF64,
}


impl HingeJoint {
    pub fn set_angle(&self, angle: f64) {
        self.angle.store(angle, Ordering::Release);
    }

    pub fn get_angle(&self) -> f64 {
        self.angle.load(Ordering::Acquire)
    }

    pub fn add_angle(&self, angle: f64) {
        self.angle.fetch_add(angle, Ordering::Release);
    }
}


pub struct HingeJointRef<'a>(&'a HingeJoint);


impl<'a> HingeJointRef<'a> {
    pub fn get_angle(&self) -> f64 {
        self.0.get_angle()
    }
}
