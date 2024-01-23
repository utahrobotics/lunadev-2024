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

/// The active stage of the localizer.  
/// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
///
/// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
async fn run_localizer(
    mut bb: LocalizerBlackboard,
) -> (LocalizerBlackboard, ()) {
    let context = bb.context;
    setup_logging!(context);

    let rand_points = |origin: Vector3, std_dev: Float| {
        let mut rng = QuickRng::default();
        let x_distr = Normal::new(origin.x, std_dev).unwrap();
        let y_distr = Normal::new(origin.y, std_dev).unwrap();
        let z_distr = Normal::new(origin.z, std_dev).unwrap();

        (0..bb.point_count).map(|_| Vector3::new(x_distr.sample(&mut rng), y_distr.sample(&mut rng), z_distr.sample(&mut rng))).collect()
    };

    let mut position_points: Box<[Vector3]> = rand_points(bb.start_position.coords, bb.start_std_dev);
    let mut velocity_points: Box<[Vector3]> = vec![Default::default(); bb.point_count].into_boxed_slice();
    let mut accel_points: Box<[Vector3]> = vec![Vector3::new(0.0, -9.81, 0.0); bb.point_count].into_boxed_slice();

    let mut ang_vel_points: Box<[UnitQuaternion<Float>]> = vec![Default::default(); bb.point_count].into_boxed_slice();
    let mut orientation_points: Box<[UnitQuaternion<Float>]> = (0..bb.point_count).map(move |i| UnitQuaternion::from_axis_angle(&UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)), std::f64::consts::TAU as Float * i as Float / bb.point_count as Float)).collect();
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
                    frame.variance = isometry.rotation.to_rotation_matrix() * frame.variance;

                    // Apply the predicted position, orientation, and velocity onto the robot base such that
                    // other nodes can observe it.
                    bb.robot_base.set_isometry(Isometry3::from_parts(eskf.position.into(), eskf.orientation));
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
