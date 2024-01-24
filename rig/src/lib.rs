//! The rig library serves as the analogue to the `tf2` library of `ROS`.
//!
//! This library is being actively developed based on current needs, so it
//! will never be a complete match to `tf2`.

use std::{
    collections::HashMap,
    hash::{BuildHasher, Hash},
    sync::Arc,
};

use portable_atomic::AtomicF32;
use crossbeam::atomic::AtomicCell;
use fxhash::FxHashMap;
use joints::{Joint, JointMut};
use nalgebra::{Isometry3, Point3, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use tokio::sync::{mpsc, watch};

pub mod joints;

type Float = f32;
type AtomicFloat = AtomicF32;

/// The order with which a set of euler angles should be applied.
///
/// This is usually provided alongside the euler angles themselves.
///
/// The default rotation sequence for this crate is YXZ. When used with
/// intrinsic rotation, it most accurately depicts human head rotation
/// and is used in many places, especially turrets. In words, this order
/// means yaw-pitch-roll.
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

/// The type of rotation that a set of euler angles are describing.
///
/// This is usually provided alongside the euler angles themselves.
///
/// Intrinsic rotations mean that the axes of rotation - x, y, z -
/// are rotated by the previous rotation in the sequence. For example,
/// if we are following YXZ sequence, we first rotate along the global
/// y-axis. However, the x and z axes should be rotated as well. We then
/// rotate by the rotated x-axis. The z-axis would be rotated as well. We
/// then finally rotate by the z-axis, which was rotated by the y and x axis.
/// This is usually the most intuitive and common form of rotation and thus
/// is the default in this crate.
///
/// Extrinsic rotations simply mean that the axes of rotation - x, y, z -
/// themselves do not rotate. If we follow YXZ sequence, we first rotate along
/// the global y, then global x, then global z.
///
/// Properly distinguishing between these two is critical for implementing
/// rotations correctly, and this property must not be overlooked.
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
    pub position: Point3<Float>,
    #[serde(default)]
    pub orientation: Vector3<Float>,
    #[serde(default)]
    pub rotation_order: RotationSequence,
    #[serde(default)]
    pub rotation_type: RotationType,
    #[serde(default)]
    pub joint: Joint,
    #[serde(default)]
    pub children: FxHashMap<Box<str>, Self>,
}

#[derive(Deserialize, Serialize)]
pub struct Robot {
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
            linear_velocity: Default::default(),
            sender: watch::channel(()).0,
        }));
        let mut existing_robot_elements: FxHashMap<_, Arc<IsometryAndJoint>> = FxHashMap::default();
        let mut chain = Vec::new();

        for element_path in element_paths {
            chain.clear();
            let mut slices_iter = element_path.split('/');
            let mut slices_vec = Vec::new();

            let first_slice = slices_iter
                .next()
                .ok_or_else(|| anyhow::anyhow!("Empty element path encountered"))?;

            let mut current_element = self
                .children
                .get_mut(first_slice)
                .ok_or_else(|| anyhow::anyhow!("Missing element: {element_path}"))?;

            slices_vec.push(first_slice);
            let (sender, receiver) = mpsc::channel(1);

            loop {
                current_element.joint.init();
                current_element.joint.add_subscriber(sender.clone());
                let (w, [i, j, k]) = quaternion_core::from_euler_angles(
                    current_element.rotation_type.into(),
                    current_element.rotation_order.into(),
                    [
                        current_element.orientation.x,
                        current_element.orientation.y,
                        current_element.orientation.z,
                    ],
                );

                let robot_element = Arc::new(IsometryAndJoint {
                    isometry: Isometry3::from_parts(
                        current_element.position.into(),
                        UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k)),
                    ),
                    joint: std::mem::take(&mut current_element.joint),
                });

                existing_robot_elements.insert(slices_vec.clone(), robot_element.clone());
                chain.push(robot_element);

                let Some(path_slice) = slices_iter.next() else {
                    break;
                };
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

            if out
                .insert(
                    element_path,
                    RobotElement(Arc::new(RobotElementInner {
                        chain: chain.drain(..).collect(),
                        reference: base_element.get_ref(),
                        receiver: receiver.into(),
                    })),
                )
                .is_some()
            {
                return Err(anyhow::anyhow!("Duplicate element: {element_path}"));
            }
        }

        Ok((out, base_element))
    }
}

struct IsometryAndJoint {
    isometry: Isometry3<Float>,
    joint: Joint,
}

struct RobotElementInner {
    chain: Box<[Arc<IsometryAndJoint>]>,
    reference: RobotBaseRef,
    receiver: tokio::sync::Mutex<mpsc::Receiver<()>>,
}

/// An element of a robot, which can be moved if it is attached by a non-fixed
/// joint to its parent.
// pub struct RobotElement(Arc<[Arc<RobotElementInner>]>, RobotBaseRef);
pub struct RobotElement(Arc<RobotElementInner>);

impl Hash for RobotElement {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        Arc::as_ptr(&self.0).hash(state)
    }
}

impl PartialEq for RobotElement {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl PartialEq<RobotElementRef> for RobotElement {
    fn eq(&self, other: &RobotElementRef) -> bool {
        Arc::ptr_eq(&self.0, &other.0 .0)
    }
}

impl Eq for RobotElement {}

/// An immutable reference to an element of a robot.
///
/// Changes may only be observed through this reference,
/// never written. If the original `RobotElement` has been dropped,
/// changes will never be made.
pub struct RobotElementRef(RobotElement);

impl Hash for RobotElementRef {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        Arc::as_ptr(&self.0 .0).hash(state)
    }
}

impl PartialEq for RobotElementRef {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0 .0, &other.0 .0)
    }
}

impl PartialEq<RobotElement> for RobotElementRef {
    fn eq(&self, other: &RobotElement) -> bool {
        Arc::ptr_eq(&self.0 .0, &other.0)
    }
}

impl Eq for RobotElementRef {}

impl Clone for RobotElementRef {
    fn clone(&self) -> Self {
        Self(RobotElement(self.0 .0.clone()))
    }
}

impl RobotElement {
    /// Gets an immutable reference to this element.
    ///
    /// This is as cheap as cloning an existing reference.
    pub fn get_ref(&self) -> RobotElementRef {
        RobotElementRef(RobotElement(self.0.clone()))
    }

    fn get_local_element(&self) -> &IsometryAndJoint {
        self.0.chain.last().unwrap()
    }

    pub fn get_local_joint(&mut self) -> JointMut {
        self.get_local_element().joint.get_joint_mut()
    }

    pub fn get_isometry_from_base(&self) -> Isometry3<Float> {
        let mut out = Isometry3::default();

        for element in self.0.chain.iter().rev() {
            out = element.isometry * element.joint.get_isometry() * out;
        }

        out
    }

    pub fn get_isometry_of_base(&self) -> Isometry3<Float> {
        self.0.reference.get_isometry()
    }

    pub fn get_global_isometry(&self) -> Isometry3<Float> {
        self.get_isometry_of_base() * self.get_isometry_from_base()
    }
}

impl RobotElementRef {
    pub fn get_local_joint(&self) -> &Joint {
        &self.0.get_local_element().joint
    }

    pub fn get_isometry_from_base(&self) -> Isometry3<Float> {
        self.0.get_isometry_from_base()
    }

    pub fn get_isometry_of_base(&self) -> Isometry3<Float> {
        self.0.get_isometry_of_base()
    }

    pub fn get_global_isometry(&self) -> Isometry3<Float> {
        self.0.get_isometry_of_base() * self.0.get_isometry_from_base()
    }

    pub async fn wait_for_change(&self) {
        self.0
             .0
            .receiver
            .lock()
            .await
            .recv()
            .await
            .expect("Sender should still be alive");
    }
}

struct RobotBaseInner {
    isometry: AtomicCell<Isometry3<Float>>,
    linear_velocity: AtomicCell<Vector3<Float>>,
    sender: watch::Sender<()>,
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
pub struct RobotBaseRef(RobotBase, watch::Receiver<()>);

impl Clone for RobotBaseRef {
    fn clone(&self) -> Self {
        Self(RobotBase(self.0 .0.clone()), self.1.clone())
    }
}

impl RobotBase {
    /// Gets an immutable reference to this base.
    ///
    /// This is as cheap as cloning an existing reference.
    pub fn get_ref(&self) -> RobotBaseRef {
        RobotBaseRef(Self(self.0.clone()), self.0.sender.subscribe())
    }

    pub fn get_isometry(&self) -> Isometry3<Float> {
        self.0.isometry.load()
    }

    pub fn set_linear_velocity(&self, linear_velocity: Vector3<Float>) {
        self.0.linear_velocity.store(linear_velocity);
        self.0.sender.send_replace(());
    }

    pub fn get_linear_velocity(&self) -> Vector3<Float> {
        self.0.linear_velocity.load()
    }

    pub fn set_isometry(&self, isometry: Isometry3<Float>) {
        self.0.isometry.store(isometry);
        self.0.sender.send_replace(());
    }

    pub fn set_position(&self, position: Point3<Float>) {
        let mut isometry = self.get_isometry();
        isometry.translation = position.into();
        self.set_isometry(isometry);
    }

    pub fn set_orientation(&self, orientation: UnitQuaternion<Float>) {
        let mut isometry = self.get_isometry();
        isometry.rotation = orientation.into();
        self.set_isometry(isometry);
    }
}

impl RobotBaseRef {
    pub fn get_isometry(&self) -> Isometry3<Float> {
        self.0.get_isometry()
    }

    pub fn get_linear_velocity(&self) -> Vector3<Float> {
        self.0.get_linear_velocity()
    }

    pub async fn wait_for_change(&mut self) {
        self.1
            .changed()
            .await
            .expect("Sender should not be dropped");
    }
}
