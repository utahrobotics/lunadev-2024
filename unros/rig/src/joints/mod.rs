use std::sync::atomic::Ordering;

use nalgebra::{Isometry3, UnitQuaternion, UnitVector3};
use serde::{Deserialize, Serialize};

use crate::{AtomicFloat, Float};

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
            Joint::Hinge(joint) => *joint.angle.get_mut() = joint.starting_angle,
        }
    }

    pub(super) fn get_joint_mut(&self) -> JointMut {
        match self {
            Joint::Fixed => JointMut::Fixed,
            Joint::Hinge(joint) => JointMut::Hinge(HingeJointMut(joint)),
        }
    }

    pub fn get_isometry(&self) -> Isometry3<Float> {
        match self {
            Joint::Fixed => Isometry3::default(),
            Joint::Hinge(x) => Isometry3::from_parts(
                Default::default(),
                UnitQuaternion::from_axis_angle(&x.axis, x.angle.load(Ordering::Acquire)),
            ),
        }
    }

    // pub(super) fn add_subscriber(&self, sender: mpsc::Sender<()>) {
    //     match self {
    //         Joint::Fixed => {}
    //         Joint::Hinge(x) => x.add_subscriber(sender),
    //     }
    // }
}

/// A mutable reference to a joint.
///
/// Unlike `RobotElementRef`, this is bounded to the lifetime of
/// the original joint.
pub enum JointMut<'a> {
    Fixed,
    Hinge(HingeJointMut<'a>),
}

/// A joint that rotates about a single axis.
#[derive(Deserialize, Serialize)]
pub struct HingeJoint {
    pub axis: UnitVector3<Float>,
    #[serde(default)]
    pub starting_angle: Float,
    #[serde(skip)]
    angle: AtomicFloat,
    // #[serde(skip)]
    // senders: Mutex<Vec<mpsc::Sender<()>>>,
}

impl HingeJoint {
    // fn update(&self) {
    //     for sender in self.senders.lock().unwrap().iter() {
    //         let _ = sender.try_send(());
    //     }
    // }

    /// Gets the signed angle of the hinge from the starting point.
    pub fn get_angle(&self) -> Float {
        self.angle.load(Ordering::Acquire)
    }

    // pub(super) fn add_subscriber(&self, sender: mpsc::Sender<()>) {
    //     self.senders.lock().unwrap().push(sender);
    // }
}

/// A mutable reference to a `HingeJoint`.
///
/// This is the only way to rotate a `HingeJoint`.
pub struct HingeJointMut<'a>(&'a HingeJoint);

impl<'a> HingeJointMut<'a> {
    pub fn get_angle(&self) -> Float {
        self.0.get_angle()
    }

    pub fn add_angle(&mut self, angle: Float) {
        self.0.angle.fetch_add(angle, Ordering::Release);
        // self.0.update();
    }

    pub fn set_angle(&mut self, angle: Float) {
        self.0.angle.store(angle, Ordering::Release);
        // self.0.update();
    }
}
