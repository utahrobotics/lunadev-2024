//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::time::{Duration, Instant};

pub use eskf;
use nalgebra::{Isometry3, Matrix3, Point3, Quaternion, UnitQuaternion, Vector3};
use rig::{RobotBase, RobotElementRef, RotationSequence, RotationType};
use unros_core::{
    anyhow, async_trait, setup_logging, signal::unbounded::UnboundedSubscription, tokio, Node,
    RuntimeContext,
};

/// A position and variance measurement.
#[derive(Clone)]
pub struct PositionFrame {
    pub position: Point3<f64>,
    pub variance: Matrix3<f64>,
    pub robot_element: RobotElementRef,
}

/// An orientation and variance measurement.
#[derive(Clone)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion<f64>,
    pub variance: Matrix3<f64>,
    pub robot_element: RobotElementRef,
}

/// A measurement from an IMU.
#[derive(Clone)]
pub struct IMUFrame {
    pub acceleration: Vector3<f64>,

    pub angular_velocity: Vector3<f64>,
    pub rotation_sequence: RotationSequence,
    pub rotation_type: RotationType,

    pub acceleration_variance: Vector3<f64>,
    pub angular_velocity_variance: Vector3<f64>,

    pub robot_element: RobotElementRef,
}

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
pub struct Localizer {
    /// The builder for the Error-State Kalman Filter
    pub builder: eskf::Builder,
    imu_sub: UnboundedSubscription<IMUFrame>,
    position_sub: UnboundedSubscription<PositionFrame>,
    orientation_sub: UnboundedSubscription<OrientationFrame>,

    robot_base: RobotBase,
}

impl Localizer {
    pub fn new(robot_base: RobotBase) -> Self {
        Self {
            builder: Default::default(),
            imu_sub: UnboundedSubscription::none(),
            position_sub: UnboundedSubscription::none(),
            orientation_sub: UnboundedSubscription::none(),
            robot_base,
        }
    }

    /// Provide an imu subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn add_imu_sub(&mut self, sub: UnboundedSubscription<IMUFrame>) {
        self.imu_sub += sub;
    }

    /// Provide a position subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn add_position_sub(&mut self, sub: UnboundedSubscription<PositionFrame>) {
        self.position_sub += sub;
    }

    /// Provide an orientation subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn add_orientation_sub(&mut self, sub: UnboundedSubscription<OrientationFrame>) {
        self.orientation_sub += sub;
    }
}

#[async_trait]
impl Node for Localizer {
    const DEFAULT_NAME: &'static str = "positioning";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let mut eskf = self.builder.build();
        let start = Instant::now();
        let mut last_elapsed = Duration::ZERO;

        let mut imu_sub_counter = 0u64;
        let mut imu_sub = self.imu_sub.to_watched().await.map(move |x| {
            let out = (x, imu_sub_counter);
            imu_sub_counter += 1;
            out
        });
        let mut position_sub_counter = 0u64;
        let mut position_sub = self.position_sub.to_watched().await.map(move |x| {
            let out = (x, position_sub_counter);
            position_sub_counter += 1;
            out
        });
        let mut orientation_sub_counter = 0u64;
        let mut orientation_sub = self.orientation_sub.to_watched().await.map(move |x| {
            let out = (x, orientation_sub_counter);
            orientation_sub_counter += 1;
            out
        });

        let mut imu_recv_counter = 0u64;
        let mut position_recv_counter = 0u64;
        let mut orientation_recv_counter = 0u64;

        loop {
            tokio::select! {
                (mut frame, imu_sub_counter) = imu_sub.wait_for_change() => {
                    if imu_sub_counter != imu_recv_counter {
                        warn!("Lagged behind by {} imu frames", imu_recv_counter - imu_sub_counter);
                        imu_recv_counter = imu_sub_counter + 1;
                    } else {
                        imu_recv_counter += 1;
                    }

                    let isometry = frame.robot_element.get_isometry_from_base();
                    frame.acceleration_variance = isometry * frame.acceleration_variance;
                    frame.acceleration = isometry * frame.acceleration;

                    let (w, [i, j, k]) = quaternion_core::from_euler_angles(frame.rotation_type.into(), frame.rotation_sequence.into(), [frame.angular_velocity_variance.x, frame.angular_velocity_variance.y, frame.angular_velocity_variance.z]);
                    let angular_velocity_variance_quat = isometry.rotation * UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k));
                    let angular_velocity_variance_quat = (angular_velocity_variance_quat.w, [angular_velocity_variance_quat.i, angular_velocity_variance_quat.j, angular_velocity_variance_quat.k]);

                    eskf.set_acceleration_variance(frame.acceleration_variance);
                    eskf.set_rotational_variance(quaternion_core::to_euler_angles(quaternion_core::RotationType::Intrinsic, quaternion_core::RotationSequence::XYZ, angular_velocity_variance_quat).into());

                    eskf.gravity = eskf.orientation * Vector3::y_axis().into_inner() * -9.81;
                    let (w, [i, j, k]) = quaternion_core::from_euler_angles(frame.rotation_type.into(), frame.rotation_sequence.into(), [frame.angular_velocity.x, frame.angular_velocity.y, frame.angular_velocity.z]);
                    let vel_quat = isometry.rotation * UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k));
                    let vel_quat = (vel_quat.w, [vel_quat.i, vel_quat.j, vel_quat.k]);
                    frame.angular_velocity = quaternion_core::to_euler_angles(quaternion_core::RotationType::Intrinsic, quaternion_core::RotationSequence::XYZ, vel_quat).into();

                    let now = start.elapsed();
                    let delta = now - last_elapsed;
                    eskf.predict(
                        frame.acceleration,
                        frame.angular_velocity,
                        delta
                    );
                    last_elapsed = now;

                    self.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    self.robot_base.set_linear_velocity(eskf.velocity);
                }
                (mut frame, position_sub_counter) = position_sub.wait_for_change() => {
                    if position_sub_counter != position_recv_counter {
                        warn!("Lagged behind by {} position frames", position_sub_counter - position_recv_counter);
                        position_recv_counter = position_sub_counter + 1;
                    } else {
                        position_recv_counter += 1;
                    }

                    let isometry = frame.robot_element.get_isometry_from_base();
                    frame.position = isometry * frame.position;
                    frame.variance = isometry.rotation.to_rotation_matrix() * frame.variance;

                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        error!("Failed to observe position: {e:#?}");
                        continue;
                    }

                    self.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    self.robot_base.set_linear_velocity(eskf.velocity);
                }
                (mut frame, orientation_sub_counter) = orientation_sub.wait_for_change() => {
                    if orientation_sub_counter != orientation_recv_counter {
                        warn!("Lagged behind by {} orientation frames", imu_recv_counter - orientation_sub_counter);
                        orientation_recv_counter = orientation_sub_counter + 1;
                    } else {
                        orientation_recv_counter += 1;
                    }

                    let isometry = frame.robot_element.get_isometry_from_base();
                    frame.orientation = isometry.rotation * frame.orientation;
                    frame.variance = isometry.rotation.to_rotation_matrix() * frame.variance;

                    if let Err(e) = eskf.observe_orientation(frame.orientation, frame.variance) {
                        error!("Failed to observe orientation: {e:#?}");
                        continue;
                    }

                    self.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    self.robot_base.set_linear_velocity(eskf.velocity);
                }
            }
        }
    }
}
