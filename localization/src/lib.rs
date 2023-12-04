//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::{
    collections::hash_map::Entry,
    f64::consts::PI,
    time::{Duration, Instant},
};

pub use eskf;
use eskf::ESKF;
use fxhash::FxHashMap;
use nalgebra::{Isometry3, Matrix3, Point3, Quaternion, UnitQuaternion, UnitVector3, Vector3};
use rig::{RobotBase, RobotElementRef, RotationSequence, RotationType};
use smach::{start_machine, Transition};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{unbounded::UnboundedSubscription, watched::WatchedSubscription},
    tokio, Node, RuntimeContext,
};

/// A position and variance measurement.
#[derive(Clone)]
pub struct PositionFrame {
    pub position: Point3<f64>,
    pub variance: Matrix3<f64>,

    pub velocity: Option<Vector3<f64>>,

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

    pub calibration_duration: Duration,

    recalibrate_sub: WatchedSubscription<()>,

    imu_sub: UnboundedSubscription<IMUFrame>,
    position_sub: UnboundedSubscription<PositionFrame>,
    orientation_sub: UnboundedSubscription<OrientationFrame>,

    robot_base: RobotBase,
}

impl Localizer {
    pub fn new(robot_base: RobotBase) -> Self {
        Self {
            builder: Default::default(),
            calibration_duration: Duration::from_secs(5),
            recalibrate_sub: WatchedSubscription::none(),
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

struct CalibratingImu {
    count: usize,
    accel: Vector3<f64>,
    angular_velocity: Vector3<f64>,
}

#[derive(Debug)]
struct CalibratedImu {
    accel_scale: f64,
    accel_correction: UnitQuaternion<f64>,
    angular_velocity_bias: Vector3<f64>,
}

struct LocalizerBlackboard {
    builder: eskf::Builder,

    calibration_duration: Duration,

    recalibrate_sub: WatchedSubscription<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu>,

    imu_sub: WatchedSubscription<(IMUFrame, u64)>,
    position_sub: WatchedSubscription<(PositionFrame, u64)>,
    orientation_sub: WatchedSubscription<(OrientationFrame, u64)>,

    imu_recv_counter: u64,
    position_recv_counter: u64,
    orientation_recv_counter: u64,

    robot_base: RobotBase,
}

async fn calibrate_localizer(
    (mut bb, context, _): (LocalizerBlackboard, RuntimeContext, ESKF),
) -> ((LocalizerBlackboard, RuntimeContext, ESKF), ()) {
    setup_logging!(context);
    info!("Calibrating localizer");

    let mut imu_map = FxHashMap::<RobotElementRef, CalibratingImu>::default();
    let mut total_gravity = Vector3::default();

    tokio::select! {
        _ = tokio::time::sleep(bb.calibration_duration) => {}
        _ = async { loop {
            let (imu, n) = bb.imu_sub.wait_for_change().await;
            bb.imu_recv_counter = n + 1;
            total_gravity += imu.acceleration;
            match imu_map.entry(imu.robot_element) {
                Entry::Occupied(mut x) => {
                    let x = x.get_mut();
                    x.count += 1;
                    x.accel += imu.acceleration;
                    x.angular_velocity += imu.angular_velocity;
                }
                Entry::Vacant(x) => {
                    x.insert(CalibratingImu { count: 1, accel: imu.acceleration, angular_velocity: imu.angular_velocity });
                }
            }
        }} => {}
    }
    let total_gravity = UnitVector3::new_normalize(total_gravity);

    bb.calibrations = imu_map
        .into_iter()
        .map(|(robot_element, calibrating)| {
            let accel_norm = UnitVector3::new_normalize(calibrating.accel);

            let accel_correction = {
                let angle = accel_norm.angle(&total_gravity);
                if angle < 1.0 / 180.0 * PI {
                    UnitQuaternion::default()
                } else {
                    UnitQuaternion::from_axis_angle(
                        &UnitVector3::new_normalize(accel_norm.cross(&total_gravity)),
                        angle,
                    )
                }
            };

            let calibrated = CalibratedImu {
                accel_scale: 9.81 / calibrating.accel.magnitude() * calibrating.count as f64,
                accel_correction,
                angular_velocity_bias: calibrating.angular_velocity / calibrating.count as f64,
            };
            (robot_element, calibrated)
        })
        .collect();

    let mut eskf = bb.builder.clone().build();
    if let Err(e) = eskf.observe_orientation(
        UnitQuaternion::from_axis_angle(
            &UnitVector3::new_normalize(total_gravity.cross(&-Vector3::y_axis())),
            total_gravity.angle(&-Vector3::y_axis()),
        ),
        ESKF::variance_from_element(0.01),
    ) {
        error!("Failed to initialize orientation: {e:?}");
    }
    eskf.gravity = Vector3::y_axis().into_inner() * -9.81;
    bb.recalibrate_sub.get_or_empty();
    ((bb, context, eskf), ())
}

async fn run_localizer(
    (mut bb, context, mut eskf): (LocalizerBlackboard, RuntimeContext, ESKF),
) -> ((LocalizerBlackboard, RuntimeContext, ESKF), ()) {
    setup_logging!(context);
    info!("Localizer calibrated");

    let start = Instant::now();
    let mut last_elapsed = Duration::ZERO;

    tokio::select! {
        () = bb.recalibrate_sub.wait_for_change() => {}
        _ = async { loop {
            tokio::select! {
                (mut frame, imu_sub_counter) = bb.imu_sub.wait_for_change() => {
                    if imu_sub_counter != bb.imu_recv_counter {
                        warn!("Lagged behind by {} imu frames", bb.imu_recv_counter as i64 - imu_sub_counter as i64);
                        bb.imu_recv_counter = imu_sub_counter + 1;
                    } else {
                        bb.imu_recv_counter += 1;
                    }

                    let Some(calibration) = bb.calibrations.get(&frame.robot_element) else {
                        error!("Unrecognized IMU message");
                        continue;
                    };

                    // frame.acceleration_variance = isometry * frame.acceleration_variance;
                    eskf.set_acceleration_variance(frame.acceleration_variance * calibration.accel_scale);
                    // println!("{}", eskf.orientation * frame.acceleration);

                    frame.angular_velocity_variance -= calibration.angular_velocity_bias;

                    let isometry = frame.robot_element.get_isometry_from_base();

                    frame.acceleration = isometry * calibration.accel_correction * frame.acceleration * calibration.accel_scale;

                    let (w, [i, j, k]) = quaternion_core::from_euler_angles(frame.rotation_type.into(), frame.rotation_sequence.into(), [frame.angular_velocity_variance.x, frame.angular_velocity_variance.y, frame.angular_velocity_variance.z]);
                    let angular_velocity_variance_quat = isometry.rotation * UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k));
                    let angular_velocity_variance_quat = (angular_velocity_variance_quat.w, [angular_velocity_variance_quat.i, angular_velocity_variance_quat.j, angular_velocity_variance_quat.k]);

                    eskf.set_rotational_variance(quaternion_core::to_euler_angles(quaternion_core::RotationType::Intrinsic, quaternion_core::RotationSequence::XYZ, angular_velocity_variance_quat).into());

                    let (w, [i, j, k]) = quaternion_core::from_euler_angles(frame.rotation_type.into(), frame.rotation_sequence.into(), [frame.angular_velocity.x, frame.angular_velocity.y, frame.angular_velocity.z]);
                    let vel_quat = isometry.rotation * UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k));
                    let vel_quat = (vel_quat.w, [vel_quat.i, vel_quat.j, vel_quat.k]);
                    frame.angular_velocity = quaternion_core::to_euler_angles(quaternion_core::RotationType::Intrinsic, quaternion_core::RotationSequence::XYZ, vel_quat).into();

                    let now = start.elapsed();
                    let delta = now - last_elapsed;
                    // info!("accel: {:?}", frame.acceleration);
                    // info!("ang_vel: {:?}", frame.angular_velocity);
                    // info!("delta: {:.4}", delta.as_secs_f32());
                    // info!("{}", eskf.orientation);
                    eskf.predict(
                        frame.acceleration,
                        frame.angular_velocity,
                        delta
                    );
                    // println!("{:.2}", (frame.acceleration - Vector3::y_axis().into_inner() * 9.81).y);
                    last_elapsed = now;

                    if eskf.velocity.x.is_nan() {
                        break;
                    }

                    bb.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    bb.robot_base.set_linear_velocity(eskf.velocity);
                }
                (mut frame, position_sub_counter) = bb.position_sub.wait_for_change() => {
                    if position_sub_counter != bb.position_recv_counter {
                        warn!("Lagged behind by {} position frames", position_sub_counter - bb.position_recv_counter);
                        bb.position_recv_counter = position_sub_counter + 1;
                    } else {
                        bb.position_recv_counter += 1;
                    }

                    let isometry = frame.robot_element.get_isometry_from_base().inverse();
                    frame.position = isometry * frame.position;
                    frame.variance = isometry.rotation.to_rotation_matrix() * frame.variance;

                    if let Some(mut velocity) = frame.velocity {
                        velocity = isometry * velocity;
                        // info!("Tag vel: {:?}", velocity);
                        if let Err(e) = eskf.observe_velocity(velocity, frame.variance) {
                            error!("Failed to observe velocity: {e:#?}");
                            continue;
                        }
                    } else {
                        // info!("Tag pos: {:?}", frame.position);
                        if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                            error!("Failed to observe position: {e:#?}");
                            continue;
                        }
                    }

                    if eskf.velocity.x.is_nan() {
                        break;
                    }

                    bb.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    bb.robot_base.set_linear_velocity(eskf.velocity);
                }
                (mut frame, orientation_sub_counter) = bb.orientation_sub.wait_for_change() => {
                    if orientation_sub_counter != bb.orientation_recv_counter {
                        // warn!("Lagged behind by {} orientation frames", imu_recv_counter - orientation_sub_counter);
                        bb.orientation_recv_counter = orientation_sub_counter + 1;
                    } else {
                        bb.orientation_recv_counter += 1;
                    }

                    let isometry = frame.robot_element.get_isometry_from_base().inverse();
                    frame.orientation = isometry.rotation * frame.orientation;
                    frame.variance = isometry.rotation.to_rotation_matrix() * frame.variance;

                    // info!("Tag orientation: {}", frame.orientation);
                    if let Err(e) = eskf.observe_orientation(frame.orientation, frame.variance) {
                        error!("Failed to observe orientation: {e:#?}");
                        continue;
                    }

                    bb.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                }
            }
        }}=> {}
    }
    ((bb, context, eskf), ())
}

struct CalibrateTransition;

impl Transition<(), (LocalizerBlackboard, RuntimeContext, ESKF)> for CalibrateTransition {
    fn transition(
        _: (),
        blackboard: (LocalizerBlackboard, RuntimeContext, ESKF),
        init: smach::TransitionInit<(LocalizerBlackboard, RuntimeContext, ESKF)>,
    ) -> smach::Transitioned {
        init.next_state::<RunTransition, _, _>(blackboard, run_localizer)
    }
}

struct RunTransition;

impl Transition<(), (LocalizerBlackboard, RuntimeContext, ESKF)> for RunTransition {
    fn transition(
        _: (),
        blackboard: (LocalizerBlackboard, RuntimeContext, ESKF),
        init: smach::TransitionInit<(LocalizerBlackboard, RuntimeContext, ESKF)>,
    ) -> smach::Transitioned {
        init.next_state::<CalibrateTransition, _, _>(blackboard, calibrate_localizer)
    }
}

#[async_trait]
impl Node for Localizer {
    const DEFAULT_NAME: &'static str = "positioning";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let mut imu_sub_counter = 0u64;
        let imu_sub = self.imu_sub.to_watched().await.map(move |x| {
            let out = (x, imu_sub_counter);
            imu_sub_counter += 1;
            out
        });
        let mut position_sub_counter = 0u64;
        let position_sub = self.position_sub.to_watched().await.map(move |x| {
            let out = (x, position_sub_counter);
            position_sub_counter += 1;
            out
        });
        let mut orientation_sub_counter = 0u64;
        let orientation_sub = self.orientation_sub.to_watched().await.map(move |x| {
            let out = (x, orientation_sub_counter);
            orientation_sub_counter += 1;
            out
        });

        let bb = LocalizerBlackboard {
            builder: self.builder.clone(),
            calibration_duration: self.calibration_duration,
            recalibrate_sub: self.recalibrate_sub,
            calibrations: Default::default(),
            imu_sub,
            position_sub,
            orientation_sub,
            imu_recv_counter: 0,
            position_recv_counter: 0,
            orientation_recv_counter: 0,
            robot_base: self.robot_base,
        };
        start_machine::<CalibrateTransition, _, _, _>(
            (bb, context, self.builder.build()),
            calibrate_localizer,
        )
        .await;

        unreachable!()
    }
}
