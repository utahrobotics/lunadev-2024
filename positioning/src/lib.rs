//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::time::{Duration, Instant};

pub use eskf;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3, Isometry3};
use rig::RobotBase;
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::unbounded::UnboundedSubscription,
    tokio, Node, RuntimeContext,
};

/// A position and variance measurement.
#[derive(Clone, Copy)]
pub struct PositionFrame {
    pub position: Point3<f64>,
    pub variance: Matrix3<f64>,
}

/// An orientation and variance measurement.
#[derive(Clone, Copy)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion<f64>,
    pub variance: Matrix3<f64>,
}

/// A measurement from an IMU.
#[derive(Clone, Copy)]
pub struct IMUFrame {
    pub acceleration: Vector3<f64>,
    /// XYZ rotation order
    pub angular_velocity: Vector3<f64>,

    pub acceleration_variance: Option<Vector3<f64>>,
    pub angular_velocity_variance: Option<Vector3<f64>>,
}

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
pub struct Positioner {
    /// The builder for the Error-State Kalman Filter
    pub builder: eskf::Builder,
    imu_sub: UnboundedSubscription<IMUFrame>,
    position_sub: UnboundedSubscription<PositionFrame>,
    orientation_sub: UnboundedSubscription<OrientationFrame>,

    robot_base: RobotBase,
}

impl Positioner {
    pub fn new(robot_base: RobotBase) -> Self {
        Self {
            builder: Default::default(),
            imu_sub: UnboundedSubscription::none(),
            position_sub: UnboundedSubscription::none(),
            orientation_sub: UnboundedSubscription::none(),
            robot_base
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
impl Node for Positioner {
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

                    if let Some(acceleration_variance) = frame.acceleration_variance {
                        eskf.set_acceleration_variance(acceleration_variance);
                    }

                    let now = start.elapsed();
                    let delta = now - last_elapsed;

                    if let Some(angular_velocity_variance) = frame.angular_velocity_variance {
                        eskf.set_rotational_variance(eskf.orientation_uncertainty() + angular_velocity_variance * delta.as_secs_f64());
                    } else {
                        eskf.set_rotational_variance(eskf.orientation_uncertainty());
                    }

                    frame.angular_velocity *= delta.as_secs_f64();

                    eskf.gravity = eskf.orientation * Vector3::y_axis().into_inner() * -9.81;

                    eskf.predict(
                        frame.acceleration,
                        (UnitQuaternion::from_axis_angle(&Vector3::x_axis(), frame.angular_velocity.x) *
                        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), frame.angular_velocity.y) *
                        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), frame.angular_velocity.z) *
                        eskf.orientation).scaled_axis(),
                        delta
                    );
                    last_elapsed = now;

                    self.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    self.robot_base.set_linear_velocity(eskf.velocity);
                }
                (frame, position_sub_counter) = position_sub.wait_for_change() => {
                    if position_sub_counter != position_recv_counter {
                        warn!("Lagged behind by {} position frames", position_sub_counter - position_recv_counter);
                        position_recv_counter = position_sub_counter + 1;
                    } else {
                        position_recv_counter += 1;
                    }

                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        error!("Failed to observe position: {e:#?}");
                        continue;
                    }

                    self.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    self.robot_base.set_linear_velocity(eskf.velocity);
                }
                (frame, orientation_sub_counter) = orientation_sub.wait_for_change() => {
                    if orientation_sub_counter != orientation_recv_counter {
                        warn!("Lagged behind by {} orientation frames", imu_recv_counter - orientation_sub_counter);
                        orientation_recv_counter = orientation_sub_counter + 1;
                    } else {
                        orientation_recv_counter += 1;
                    }

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
