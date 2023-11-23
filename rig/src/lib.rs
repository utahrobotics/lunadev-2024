//! The rig library serves as the analogue to the `tf2` library of `ROS`.
//!
//! This library is being actively developed based on current needs, so it
//! will never be a complete match to `tf2`.

use std::{
    collections::HashMap,
    hash::BuildHasher,
    sync::Arc,
};

use crossbeam::atomic::AtomicCell;
use fxhash::FxHashMap;
use joints::{Joint, JointRef};
use nalgebra::{Isometry3, Point3, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub mod joints;

#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum RotationSequence {
    // Proper (z-x-z, x-y-x, y-z-y, z-y-z, x-z-x, y-x-y)
    ZXZ,
    XYX,
    YZY,
    ZYZ,
    XZX,
    YXY,
    // Tait–Bryan (x-y-z, y-z-x, z-x-y, x-z-y, z-y-x, y-x-z)
    XYZ,
    YZX,
    ZXY,
    XZY,
    ZYX,
    YXZ,
}

impl Default for RotationSequence {
    fn default() -> Self {
        Self::YXZ
    }
}


impl Into<quaternion_core::RotationSequence> for RotationSequence {
    fn into(self) -> quaternion_core::RotationSequence {
        match self {
            RotationSequence::ZXZ => quaternion_core::RotationSequence::ZXZ,
            RotationSequence::XYX => quaternion_core::RotationSequence::XYX,
            RotationSequence::YZY => quaternion_core::RotationSequence::YZY,
            RotationSequence::ZYZ => quaternion_core::RotationSequence::ZYZ,
            RotationSequence::XZX => quaternion_core::RotationSequence::XZX,
            RotationSequence::YXY => quaternion_core::RotationSequence::YXY,
            RotationSequence::XYZ => quaternion_core::RotationSequence::XYZ,
            RotationSequence::YZX => quaternion_core::RotationSequence::YZX,
            RotationSequence::ZXY => quaternion_core::RotationSequence::ZXY,
            RotationSequence::XZY => quaternion_core::RotationSequence::XZY,
            RotationSequence::ZYX => quaternion_core::RotationSequence::ZYX,
            RotationSequence::YXZ => quaternion_core::RotationSequence::YXZ,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum RotationType {
    Intrinsic,
    Extrinsic,
}


impl Into<quaternion_core::RotationType> for RotationType {
    fn into(self) -> quaternion_core::RotationType {
        match self {
            RotationType::Intrinsic => quaternion_core::RotationType::Intrinsic,
            RotationType::Extrinsic => quaternion_core::RotationType::Extrinsic,
        }
    }
}


impl Default for RotationType {
    fn default() -> Self {
        Self::Intrinsic
    }
}

#[derive(Deserialize, Serialize)]
pub struct PendingRobotElement {
    #[serde(default)]
    pub position: Point3<f64>,
    #[serde(default)]
    pub orientation: Vector3<f64>,
    #[serde(default)]
    pub rotation_order: RotationSequence,
    #[serde(default)]
    pub rotation_type: RotationType,
    // pub isometry: Isometry3<f64>,
    #[serde(default)]
    pub joint: Joint,
    #[serde(default)]
    pub children: FxHashMap<Box<str>, Self>,
}

#[derive(Deserialize, Serialize)]
pub struct Robot {
    // #[serde(skip)]
    // isometry: AtomicCell<Isometry3<f64>>,
    pub fixed_translation: bool,
    pub fixed_rotation: bool,

    pub children: FxHashMap<Box<str>, PendingRobotElement>,
}

impl Robot {
    pub fn destructure<'a, S: BuildHasher + Default>(
        mut self,
        element_paths: impl IntoIterator<Item = &'a str>,
    ) -> anyhow::Result<(HashMap<&'a str, RobotElement, S>, RobotBase)> {
        let mut out = HashMap::default();
        let base_element = RobotBase(Arc::new(RobotBaseInner {
            isometry: Default::default(),
        }));
        let mut existing_robot_elements: FxHashMap<_, Arc<RobotElementInner>> =
            FxHashMap::default();
        let mut chain = Vec::new();

        for element_path in element_paths {
            chain.clear();
            let mut slices_iter = element_path.split('/');
            let mut slices_vec = Vec::new();

            let first_slice = slices_iter.next().ok_or_else(|| anyhow::anyhow!("Empty element path encountered"))?;

            let mut current_element = self
                .children
                .get_mut(first_slice)
                .ok_or_else(|| anyhow::anyhow!("Missing element: {element_path}"))?;

            slices_vec.push(first_slice);

            loop {
                current_element.joint.init();
                let (w, [i, j, k]) = quaternion_core::from_euler_angles(
                    current_element.rotation_type.into(),
                    current_element.rotation_order.into(),
                    [
                        current_element.orientation.x,
                        current_element.orientation.y,
                        current_element.orientation.z,
                    ],
                );

                let robot_element = Arc::new(RobotElementInner {
                    isometry: Isometry3::from_parts(
                        current_element.position.into(),
                        UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k)),
                    ),
                    joint: std::mem::take(&mut current_element.joint),
                });

                existing_robot_elements.insert(slices_vec.clone(), robot_element.clone());
                chain.push(robot_element);

                let Some(path_slice) = slices_iter.next() else { break; };
                slices_vec.push(path_slice);

                if let Some(robot_element) = existing_robot_elements.get(&slices_vec) {
                    chain.push(robot_element.clone());
                    continue;
                }
                
                current_element = current_element
                    .children
                    .get_mut(path_slice)
                    .ok_or_else(|| anyhow::anyhow!("Missing element: {element_path}"))?;
            }

            out.insert(
                element_path,
                RobotElement(chain.drain(..).collect(), base_element.get_ref()),
            )
            .ok_or_else(|| anyhow::anyhow!("Duplicate element: {element_path}"))?;
        }

        Ok((out, base_element))
    }
}

struct RobotElementInner {
    isometry: Isometry3<f64>,
    joint: Joint,
}

/// An element of a robot, which can be moved if it is attached by a non-fixed
/// joint to its parent.
pub struct RobotElement(Arc<[Arc<RobotElementInner>]>, RobotBaseRef);

/// An immutable reference to an element of a robot.
///
/// Changes may only be observed through this reference,
/// never written. If the original `RobotElement` has been dropped,
/// changes will never be made.
pub struct RobotElementRef(RobotElement);


impl Clone for RobotElementRef {
    fn clone(&self) -> Self {
        Self(RobotElement(self.0.0.clone(), self.0.1.clone()))
    }
}


impl RobotElement {
    /// Gets an immutable reference to this element.
    /// 
    /// This is as cheap as cloning an existing reference.
    pub fn get_ref(&self) -> RobotElementRef {
        RobotElementRef(Self(self.0.clone(), self.1.clone()))
    }

    fn get_local_element(&self) -> &RobotElementInner {
        self.0.last().unwrap()
    }

    pub fn get_local_joint(&mut self) -> &Joint {
        &self.get_local_element().joint
    }

    pub fn get_isometry_from_base(&self) -> Isometry3<f64> {
        let mut out = Isometry3::default();

        for element in self.0.iter().rev() {
            out = element.isometry * element.joint.get_isometry() * out;
        }

        out
    }

    pub fn get_isometry_of_base(&self) -> Isometry3<f64> {
        self.1.get_isometry()
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        self.get_isometry_of_base() * self.get_isometry_from_base()
    }
}


impl RobotElementRef {
    pub fn get_local_joint(&mut self) -> JointRef {
        self.0.get_local_element().joint.get_ref()
    }

    pub fn get_isometry_from_base(&self) -> Isometry3<f64> {
        self.0.get_isometry_from_base()
    }

    pub fn get_isometry_of_base(&self) -> Isometry3<f64> {
        self.0.get_isometry_of_base()
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        self.0.get_isometry_of_base() * self.0.get_isometry_from_base()
    }
}

struct RobotBaseInner {
    isometry: AtomicCell<Isometry3<f64>>,
}

/// The base of the robot.
///
/// When we say the robot has moved 4 meters to its left, what that
/// really means is the local position of this element has moved 4 units
/// in the `-x` direction. Thus, the global space of the robot is just
/// the coordinate space that this element is in.
pub struct RobotBase(Arc<RobotBaseInner>);

/// An immutable reference to the base of the robot.
///
/// Changes may only be observed through this reference,
/// never written. If the original `RobotBase` has been dropped,
/// changes will never be made.
pub struct RobotBaseRef(RobotBase);


impl Clone for RobotBaseRef {
    fn clone(&self) -> Self {
        Self(RobotBase(self.0.0.clone()))
    }
}


impl RobotBase {
    /// Gets an immutable reference to this base.
    /// 
    /// This is as cheap as cloning an existing reference.
    pub fn get_ref(&self) -> RobotBaseRef {
        RobotBaseRef(Self(self.0.clone()))
    }

    pub fn get_isometry(&self) -> Isometry3<f64> {
        self.0.isometry.load()
    }

    pub fn set_isometry(&self, isometry: Isometry3<f64>) {
        self.0.isometry.store(isometry);
    }

    pub fn set_position(&self, position: Point3<f64>) {
        let mut isometry = self.get_isometry();
        isometry.translation = position.into();
        self.set_isometry(isometry);
    }

    pub fn set_orientation(&self, orientation: UnitQuaternion<f64>) {
        let mut isometry = self.get_isometry();
        isometry.rotation = orientation.into();
        self.set_isometry(isometry);
    }
}


impl RobotBaseRef {
    pub fn get_isometry(&self) -> Isometry3<f64> {
        self.0.get_isometry()
    }
}