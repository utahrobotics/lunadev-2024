//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::{
    collections::hash_map::Entry,
    time::{Duration, Instant},
};

use fxhash::FxHashMap;
use nalgebra::{Isometry3, Matrix3, Point3, Quaternion, UnitQuaternion, UnitVector3, Vector3 as V3};
use rand_distr::{Distribution, Normal};
use rig::{RobotBase, RobotElementRef, RotationSequence, RotationType};
use smach::{start_machine, Transition};
use unros_core::{
    anyhow, async_trait, pubsub::{Subscriber, Subscription}, rng::QuickRng, setup_logging, tokio, Node, RuntimeContext
};

type Float = f32;
type Vector3 = V3<Float>;

/// A position and variance measurement.
#[derive(Clone)]
pub struct PositionFrame {
    /// Position in meters
    pub position: Point3<Float>,
    /// Variance centered around position in meters
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

/// An orientation and variance measurement.
#[derive(Clone)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion<Float>,
    /// Variance of orientation in radians
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

/// A measurement from an IMU.
#[derive(Clone)]
pub struct IMUFrame {
    pub acceleration: Vector3,
    /// Variance centered around acceleration in meters
    pub acceleration_variance: Float,

    pub angular_velocity: UnitQuaternion<Float>,
    /// Variance of angular_velocity in radians
    pub angular_velocity_variance: Float,

    pub robot_element: RobotElementRef,
}

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
pub struct Localizer {
    pub point_count: usize,
    pub calibration_duration: Duration,
    pub start_position: Point3<Float>,
    pub start_variance: Float,

    recalibrate_sub: Subscriber<()>,

    imu_sub: Subscriber<IMUFrame>,
    position_sub: Subscriber<PositionFrame>,
    orientation_sub: Subscriber<OrientationFrame>,

    robot_base: RobotBase,
}

impl Localizer {
    pub fn new(robot_base: RobotBase, start_variance: Float) -> Self {
        Self {
            point_count: 30,
            start_position: Default::default(),
            start_variance,
            calibration_duration: Duration::from_secs(3),
            recalibrate_sub: Subscriber::default(),
            imu_sub: Subscriber::default(),
            position_sub: Subscriber::default(),
            orientation_sub: Subscriber::default(),
            robot_base,
        }
    }

    /// Provide an imu subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_imu_sub(&mut self) -> Subscription<IMUFrame> {
        self.imu_sub.create_subscription(1)
    }

    /// Provide a position subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_position_sub(&mut self) -> Subscription<PositionFrame> {
        self.position_sub.create_subscription(1)
    }

    /// Provide an orientation subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_orientation_sub(&mut self) -> Subscription<OrientationFrame> {
        self.orientation_sub.create_subscription(1)
    }
}

struct CalibratingImu {
    count: usize,
    accel: Vector3,
    angular_velocity: UnitQuaternion<Float>,
}

#[derive(Debug)]
struct CalibratedImu {
    accel_scale: Float,
    accel_correction: UnitQuaternion<Float>,
    angular_velocity_bias: UnitQuaternion<Float>,
}

struct LocalizerBlackboard {
    point_count: usize,
    start_position: Point3<Float>,
    start_std_dev: Float,

    calibration_duration: Duration,

    recalibrate_sub: Subscriber<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu>,

    imu_sub: Subscriber<IMUFrame>,
    position_sub: Subscriber<PositionFrame>,
    orientation_sub: Subscriber<OrientationFrame>,

    robot_base: RobotBase,

    context: RuntimeContext,
}

/// The calibration stage of the localizer.
///
/// This stage runs for `calibration_duration` before applying the calibrations and exiting.
async fn calibrate_localizer(mut bb: LocalizerBlackboard) -> (LocalizerBlackboard, ()) {
    let context = bb.context;
    setup_logging!(context);
    info!("Calibrating localizer");

    let mut imu_map = FxHashMap::<RobotElementRef, CalibratingImu>::default();
    let mut total_gravity = Vector3::default();

    tokio::select! {
        _ = tokio::time::sleep(bb.calibration_duration) => {}
        _ = async { loop {
            let imu = bb.imu_sub.recv().await;
            total_gravity += imu.acceleration;
            let isometry = imu.robot_element.get_isometry_from_base();

            match imu_map.entry(imu.robot_element) {
                Entry::Occupied(mut x) => {
                    let x = x.get_mut();
                    x.count += 1;
                    x.accel += isometry * imu.acceleration;
                    x.angular_velocity = imu.angular_velocity * x.angular_velocity;
                }
                Entry::Vacant(x) => {
                    x.insert(CalibratingImu { count: 1, accel: imu.acceleration, angular_velocity: imu.angular_velocity });
                }
            }
        }} => {}
    }

    bb.calibrations = imu_map
        .into_iter()
        .map(|(robot_element, calibrating)| {
            let mut accel_correction = UnitQuaternion::from_axis_angle(
                &UnitVector3::new_normalize(calibrating.accel.cross(&total_gravity)),
                calibrating.accel.angle(&total_gravity),
            );

            if accel_correction.w.is_nan()
                || accel_correction.i.is_nan()
                || accel_correction.j.is_nan()
                || accel_correction.k.is_nan()
            {
                accel_correction = Default::default();
            }

            let calibrated = CalibratedImu {
                accel_scale: 9.81 / calibrating.accel.magnitude() * calibrating.count as Float,
                accel_correction,
                angular_velocity_bias: UnitQuaternion::default().try_slerp(&calibrating.angular_velocity, 1.0 / calibrating.count as Float, 0.01).unwrap_or_default(),
            };

            (robot_element, calibrated)
        })
        .collect();

    bb.recalibrate_sub.try_recv();
    info!("Localizer calibrated");
    bb.context = context;
    (bb, ())
}


const E: Float = std::f64::consts::E as Float;
const TAU: Float = std::f64::consts::TAU as Float;
const SQRT_TAU: Float = TAU.sqrt();


fn normal(mean: Float, std_dev: Float, x: Float) -> Float {
    E.powf(((x - mean) / std_dev).powi(2) / - 2.0) / std_dev / SQRT_TAU
}


fn c_normal_sum(mean_a: Float, mean_b: Float, mean_c: Float, x: Float) -> Float {
    normal(mean_c, x, mean_a) + normal(mean_c, x, mean_b)
}


fn c_normal_sum_deriv(mean_a: Float, mean_b: Float, mean_c: Float, x: Float) -> Float {
    let e_a = E.powf(-(mean_a - mean_c).powi(2) / 2.0 / x.powi(2));
    let e_b = E.powf(-(mean_b - mean_c).powi(2) / 2.0 / x.powi(2));
    let x2 = x.powi(2);

    - e_a / SQRT_TAU / x2 +
    (mean_a - mean_c).powi(2) * e_a / SQRT_TAU / x2 / x2 -
    e_b / SQRT_TAU / x2 +
    (mean_b - mean_c).powi(2) * e_b / SQRT_TAU / x2 / x2
}


fn c_normal_sol(mean_a: Float, mean_b: Float, mean_c: Float, n: usize, start_x: Float) -> Float {
    let mut x = start_x;
    for _ in 0..n {
        x = x - c_normal_sum(mean_a, mean_b, mean_c, x) / c_normal_sum_deriv(mean_a, mean_b, mean_c, x);
    }
    x
}


fn mean_means(mean_a: Float, std_dev_a: Float, mean_b: Float, std_dev_b: Float) -> Float {
    (std_dev_a * mean_b + std_dev_b * mean_a) / (std_dev_a + std_dev_b)
}


fn lerp_std_dev(std_dev_a: Float, std_dev_b: Float, target_std_dev: Float) -> Float {
    std_dev_a / std_dev_b * (target_std_dev - std_dev_a) + std_dev_a
}


/// The active stage of the localizer.  
/// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
///
/// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
async fn run_localizer(
    mut bb: LocalizerBlackboard,
) -> (LocalizerBlackboard, ()) {
    let context = bb.context;
    setup_logging!(context);

    let mut position = bb.start_position;
    let mut position_std_dev = bb.start_std_dev;

    let start = Instant::now();

    // Check for recalibration while simultaneously feeding observations into the Kalman Filter
    tokio::select! {
        () = bb.recalibrate_sub.recv() => {}
        _ = async { loop {
            // Simultaneously watch three different subscriptions at once.
            // 1. IMU observations
            // 2. Position observations
            // 3. Orientation observations
            tokio::select! {
                mut frame = bb.imu_sub.recv() => {
                    let Some(calibration) = bb.calibrations.get(&frame.robot_element) else {
                        error!("Unrecognized IMU message");
                        continue;
                    };

                    frame.angular_velocity -= calibration.angular_velocity_bias;

                    frame.acceleration = frame.robot_element.get_global_isometry() * calibration.accel_correction * frame.acceleration * calibration.accel_scale;

                    let isometry = frame.robot_element.get_isometry_from_base();

                    let delta = start.elapsed();
                    // PREDICT
                    start += delta;

                    // Apply the predicted position, orientation, and velocity onto the robot base such that
                    // other nodes can observe it.
                    bb.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    bb.robot_base.set_linear_velocity(eskf.velocity);
                }
                mut frame = bb.position_sub.recv() => {
                    // Find the position of the robot base based on the observation of the position of an element
                    // attached to the robot base.
                    let isometry = frame.robot_element.get_isometry_from_base().inverse();
                    frame.position = isometry * frame.position;

                    let mean_c_x = mean_means(position.x, position_std_dev, frame.position.x, frame.variance);
                    let mean_c_y = mean_means(position.y, position_std_dev, frame.position.y, frame.variance);
                    let mean_c_z = mean_means(position.z, position_std_dev, frame.position.z, frame.variance);

                    let length

                    // Apply the predicted position, orientation, and velocity onto the robot base such that
                    // other nodes can observe it.
                    bb.robot_base.set_isometry(Isometry3::from_parts(position.into(), eskf.orientation));
                    bb.robot_base.set_linear_velocity(eskf.velocity);
                }
                mut frame = bb.orientation_sub.recv() => {
                    // Find the orientation of the robot base based on the observation of the orientation of an element
                    // attached to the robot base.
                    let isometry = frame.robot_element.get_isometry_from_base().inverse();
                    frame.orientation = isometry.rotation * frame.orientation;
                    frame.variance = isometry.rotation.to_rotation_matrix() * frame.variance;

                    if let Err(e) = eskf.observe_orientation(frame.orientation, frame.variance) {
                        error!("Failed to observe orientation: {e:#?}");
                        continue;
                    }

                    // Apply the predicted position, orientation, and velocity onto the robot base such that
                    // other nodes can observe it.
                    bb.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
                    bb.robot_base.set_linear_velocity(eskf.velocity);
                }
            }
        }}=> {}
    }
    bb.context = context;
    (bb, ())
}

struct CalibrateTransition;

impl Transition<(), LocalizerBlackboard> for CalibrateTransition {
    fn transition(
        _: (),
        blackboard: LocalizerBlackboard,
        init: smach::TransitionInit<LocalizerBlackboard>,
    ) -> smach::Transitioned {
        init.next_state::<RunTransition, _, _>(blackboard, run_localizer)
    }
}

struct RunTransition;

impl Transition<(), LocalizerBlackboard> for RunTransition {
    fn transition(
        _: (),
        blackboard: LocalizerBlackboard,
        init: smach::TransitionInit<LocalizerBlackboard>,
    ) -> smach::Transitioned {
        init.next_state::<CalibrateTransition, _, _>(blackboard, calibrate_localizer)
    }
}

#[async_trait]
impl Node for Localizer {
    const DEFAULT_NAME: &'static str = "positioning";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let bb = LocalizerBlackboard {
            point_count: self.point_count,
            start_position: self.start_position,
            start_std_dev: self.start_variance.sqrt(),
            calibration_duration: self.calibration_duration,
            recalibrate_sub: self.recalibrate_sub,
            calibrations: Default::default(),
            imu_sub: self.imu_sub,
            position_sub: self.position_sub,
            orientation_sub: self.orientation_sub,
            context,
            robot_base: self.robot_base,
        };

        start_machine::<CalibrateTransition, _, _, _>(
            bb,
            calibrate_localizer,
        )
        .await;
        unreachable!()
    }
}
