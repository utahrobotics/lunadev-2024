//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::{
    collections::hash_map::Entry,
    time::{Duration, Instant},
};

use eigenvalues::{Davidson, DavidsonCorrection, SpectrumTarget};
use fxhash::FxHashMap;
use nalgebra::{
    DMatrix, Isometry, Isometry3 as I3, Matrix4, Point3 as P3, Quaternion, Translation3,
    UnitQuaternion as UQ, UnitVector3, Vector3 as V3,
};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use rig::{RobotBase, RobotElementRef};
use smach::{start_machine, Transition};
use unros_core::{
    anyhow, async_trait,
    pubsub::{Subscriber, Subscription},
    rayon::iter::{
        IndexedParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator,
        ParallelIterator,
    },
    rng::QuickRng,
    setup_logging, tokio, Node, RuntimeContext,
};

type Float = f32;
type Vector3 = V3<Float>;
type Isometry3 = I3<Float>;
type UnitQuaternion = UQ<Float>;
type Point3 = P3<Float>;

/// A position and variance measurement.
#[derive(Clone)]
pub struct PositionFrame {
    /// Position in meters
    pub position: Point3,
    /// Variance centered around position in meters
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

/// A position and variance measurement.
#[derive(Clone)]
pub struct VelocityFrame {
    /// Velocity in meters
    pub velocity: Vector3,
    /// Variance centered around velocity in meters per second
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

/// An orientation and variance measurement.
#[derive(Clone)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion,
    /// Variance of orientation in radians
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

/// A measurement from an IMU.
#[derive(Clone)]
pub struct IMUFrame {
    pub acceleration: Vector3,
    /// Variance centered around acceleration in meters per second^2
    pub acceleration_variance: Float,

    pub angular_velocity: UnitQuaternion,
    /// Variance of angular_velocity in radians per second
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
    pub start_position: Point3,
    pub start_variance: Float,
    pub max_delta: Duration,

    recalibrate_sub: Subscriber<()>,

    imu_sub: Subscriber<IMUFrame>,
    position_sub: Subscriber<PositionFrame>,
    velocity_sub: Subscriber<VelocityFrame>,
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
            velocity_sub: Subscriber::default(),
            robot_base,
            max_delta: Duration::from_millis(50),
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

    /// Provide a velocity subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_velocity_sub(&mut self) -> Subscription<VelocityFrame> {
        self.velocity_sub.create_subscription(1)
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
    angular_velocity: UnitQuaternion,
}

#[derive(Debug)]
struct CalibratedImu {
    accel_scale: Float,
    accel_correction: UnitQuaternion,
    angular_velocity_bias: UnitQuaternion,
}

struct LocalizerBlackboard {
    point_count: usize,
    start_position: Point3,
    start_std_dev: Float,
    max_delta: Duration,

    calibration_duration: Duration,

    recalibrate_sub: Subscriber<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu>,

    imu_sub: Subscriber<IMUFrame>,
    position_sub: Subscriber<PositionFrame>,
    velocity_sub: Subscriber<VelocityFrame>,
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
                angular_velocity_bias: UnitQuaternion::default()
                    .try_slerp(
                        &calibrating.angular_velocity,
                        1.0 / calibrating.count as Float,
                        0.01,
                    )
                    .unwrap_or_default(),
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

fn normal(mean: Float, std_dev: Float, x: Float) -> Float {
    E.powf(((x - mean) / std_dev).powi(2) / -2.0) / std_dev / TAU.sqrt()
}

fn rand_quat(rng: &mut QuickRng) -> UnitQuaternion {
    let u: Float = rng.gen_range(0.0..1.0);
    let v: Float = rng.gen_range(0.0..1.0);
    let w: Float = rng.gen_range(0.0..1.0);
    // h = ( sqrt(1-u) sin(2πv), sqrt(1-u) cos(2πv), sqrt(u) sin(2πw), sqrt(u) cos(2πw))
    UnitQuaternion::new_unchecked(Quaternion::new(
        (1.0 - u).sqrt() * (TAU * v).sin(),
        (1.0 - u).sqrt() * (TAU * v).cos(),
        u.sqrt() * (TAU * w).sin(),
        u.sqrt() * (TAU * w).cos(),
    ))
}

#[derive(Clone, Copy, Default)]
struct Particle {
    isometry: Isometry3,
    linear_velocity: Vector3,
    angular_velocity: UnitQuaternion,
    acceleration: Vector3,
}

/// The active stage of the localizer.  
/// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
///
/// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
async fn run_localizer(mut bb: LocalizerBlackboard) -> (LocalizerBlackboard, ()) {
    let context = bb.context;
    setup_logging!(context);

    let mut particles: Box<[_]> = (0..bb.point_count)
        .map(|_| {
            let mut rng = QuickRng::default();

            let rotation = UnitQuaternion::from_axis_angle(
                &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
                rng.gen_range(0.0..TAU),
            );

            let x_distr = Normal::new(bb.start_position.x, bb.start_std_dev).unwrap();
            let y_distr = Normal::new(bb.start_position.x, bb.start_std_dev).unwrap();
            let z_distr = Normal::new(bb.start_position.x, bb.start_std_dev).unwrap();
            let translation = Translation3::new(
                x_distr.sample(&mut rng),
                y_distr.sample(&mut rng),
                z_distr.sample(&mut rng),
            );

            Particle {
                isometry: Isometry::from_parts(translation, rotation),
                linear_velocity: Default::default(),
                angular_velocity: Default::default(),
                acceleration: Default::default(),
            }
        })
        .collect();

    let mut start = Instant::now();
    let mut normals = vec![0.0 as Float; bb.point_count];
    let mut accelerations = vec![Vector3::default(); bb.point_count];
    let mut angular_velocities = vec![UnitQuaternion::default(); bb.point_count];

    loop {
        // Simultaneously watch three different subscriptions at once.
        // 1. IMU observations
        // 2. Position observations
        // 3. Orientation observations
        tokio::select! {
            // Check for recalibration while simultaneously feeding observations into the Kalman Filter
            () = bb.recalibrate_sub.recv() => {
                break;
            }
            // Process system if max_delta time has passed and no observations were received
            () = tokio::time::sleep(bb.max_delta) => {}
            mut frame = bb.imu_sub.recv() => {
                let Some(calibration) = bb.calibrations.get(&frame.robot_element) else {
                    error!("Unrecognized IMU message");
                    continue;
                };

                frame.angular_velocity = calibration.angular_velocity_bias.inverse() * frame.angular_velocity;

                {
                    let std_dev = frame.acceleration_variance.sqrt();

                    let normals_sum: Float = particles.par_iter().zip(&mut normals).zip(&mut accelerations).map(|((p, out_normal), acceleration)| {
                        *acceleration = p.isometry.rotation * frame.robot_element.get_isometry_from_base().rotation * calibration.accel_correction * frame.acceleration * calibration.accel_scale;
                        let distance = (*acceleration - p.acceleration).magnitude();
                        *out_normal = normal(0.0, std_dev, distance);
                        *out_normal
                    }).sum();

                    particles.par_iter_mut().enumerate().for_each(|(i, p)| {
                        let mut rng = QuickRng::default();

                        // Probability of resampling point
                        if rng.gen_bool(1.0 - (normals[i] / normals_sum) as f64) {
                            let acceleration = accelerations[i];

                            let x_distr = Normal::new(acceleration.x, std_dev).unwrap();
                            let y_distr = Normal::new(acceleration.y, std_dev).unwrap();
                            let z_distr = Normal::new(acceleration.z, std_dev).unwrap();

                            p.acceleration = Vector3::new(x_distr.sample(&mut rng), y_distr.sample(&mut rng), z_distr.sample(&mut rng));
                        }
                    });
                }

                {
                    let std_dev = frame.angular_velocity_variance.sqrt();

                    let normals_sum: Float = particles.par_iter().zip(&mut normals).zip(&mut angular_velocities).map(|((p, out_normal), angular_velocity)| {
                        *angular_velocity = p.isometry.rotation * frame.robot_element.get_isometry_from_base().rotation * frame.angular_velocity;
                        let distance = p.angular_velocity.angle_to(angular_velocity);
                        *out_normal = normal(0.0, std_dev, distance);
                        *out_normal
                    }).sum();

                    let ang_distr = Normal::new(0.0, std_dev).unwrap();

                    particles.par_iter_mut().enumerate().for_each(|(i, p)| {
                        let mut rng = QuickRng::default();

                        // Probability of resampling point
                        if rng.gen_bool(1.0 - (normals[i] / normals_sum) as f64) {
                            let angular_velocity = angular_velocities[i];

                            let target_ang = ang_distr.sample(&mut rng);
                            let rand_quat = rand_quat(&mut rng);
                            let rand_ang = angular_velocity.angle_to(&rand_quat);
                            let t = target_ang / rand_ang;
                            p.angular_velocity = angular_velocity.slerp(&rand_quat, t);
                        }
                    });
                }
            }
            mut frame = bb.position_sub.recv() => {
                // Find the position of the robot base based on the observation of the position of an element
                // attached to the robot base.
                let isometry = frame.robot_element.get_isometry_from_base().inverse();
                frame.position = isometry * frame.position;
                let std_dev = frame.variance.sqrt();

                let x_distr = Normal::new(frame.position.x, std_dev).unwrap();
                let y_distr = Normal::new(frame.position.y, std_dev).unwrap();
                let z_distr = Normal::new(frame.position.z, std_dev).unwrap();

                let normals_sum: Float = particles.par_iter().zip(&mut normals).map(|(p, out)| {
                    let distance = (frame.position.coords - p.isometry.translation.vector).magnitude();
                    let normal = normal(0.0, std_dev, distance);
                    *out = normal;
                    normal
                }).sum();

                particles.par_iter_mut().enumerate().for_each(|(i, p)| {
                    let mut rng = QuickRng::default();

                    // Probability of resampling point
                    if rng.gen_bool(1.0 - (normals[i] / normals_sum) as f64) {
                        p.isometry.translation = Translation3::new(x_distr.sample(&mut rng), y_distr.sample(&mut rng), z_distr.sample(&mut rng));
                    }
                });
            }
            mut frame = bb.velocity_sub.recv() => {
                // Find the velocity of the robot base based on the observation of the position of an element
                // attached to the robot base.
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                frame.velocity = inv_rotation * frame.velocity;
                let std_dev = frame.variance.sqrt();

                let x_distr = Normal::new(frame.velocity.x, std_dev).unwrap();
                let y_distr = Normal::new(frame.velocity.y, std_dev).unwrap();
                let z_distr = Normal::new(frame.velocity.z, std_dev).unwrap();

                let normals_sum: Float = particles.par_iter().zip(&mut normals).map(|(p, out)| {
                    let distance = (frame.velocity - p.linear_velocity).magnitude();
                    let normal = normal(0.0, std_dev, distance);
                    *out = normal;
                    normal
                }).sum();

                particles.par_iter_mut().enumerate().for_each(|(i, p)| {
                    let mut rng = QuickRng::default();

                    // Probability of resampling point
                    if rng.gen_bool(1.0 - (normals[i] / normals_sum) as f64) {
                        p.linear_velocity = Vector3::new(x_distr.sample(&mut rng), y_distr.sample(&mut rng), z_distr.sample(&mut rng));
                    }
                });
            }
            mut frame = bb.orientation_sub.recv() => {
                // Find the orientation of the robot base based on the observation of the orientation of an element
                // attached to the robot base.
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                frame.orientation = inv_rotation * frame.orientation;

                let std_dev = frame.variance.sqrt();

                let normals_sum: Float = particles.par_iter().zip(&mut normals).map(|(p, out_normal)| {
                    let distance = p.isometry.rotation.angle_to(&frame.orientation);
                    *out_normal = normal(0.0, std_dev, distance);
                    *out_normal
                }).sum();

                let ang_distr = Normal::new(0.0, std_dev).unwrap();

                particles.par_iter_mut().enumerate().for_each(|(i, p)| {
                    let mut rng = QuickRng::default();

                    // Probability of resampling point
                    if rng.gen_bool(1.0 - (normals[i] / normals_sum) as f64) {
                        let target_ang = ang_distr.sample(&mut rng);
                        let rand_quat = rand_quat(&mut rng);
                        let rand_ang = p.isometry.rotation.angle_to(&rand_quat);
                        let t = target_ang / rand_ang;
                        p.isometry.rotation = p.isometry.rotation.slerp(&rand_quat, t);
                    }
                });
            }
        }

        // Apply the predicted position, orientation, and velocity onto the robot base such that
        // other nodes can observe it.
        let delta_duration = start.elapsed();
        let delta = delta_duration.as_secs_f64() as Float;
        particles.par_iter_mut().for_each(|p| {
            p.linear_velocity += (p.acceleration + Vector3::new(0.0, 9.81, 0.0)) * delta;
            p.isometry.translation.vector += p.linear_velocity * delta;
            p.isometry.rotation = UnitQuaternion::default()
                .try_slerp(&p.angular_velocity, delta, 0.001)
                .unwrap_or_default()
                * p.isometry.rotation;
        });
        start += delta_duration;

        let mut rotation_matrix = Matrix4::<Float>::default();
        let mut position = Vector3::default();
        let mut velocity = Vector3::default();

        particles.iter().for_each(|p| {
            let q_vec = p.isometry.rotation.as_vector();
            rotation_matrix += q_vec * q_vec.transpose() / bb.point_count as Float;
            position += p.isometry.translation.vector;
            velocity += p.linear_velocity;
        });

        position /= bb.point_count as Float;
        velocity /= bb.point_count as Float;

        let mut isometry = bb.robot_base.get_isometry();
        isometry.translation.vector = position;

        // https://math.stackexchange.com/questions/61146/averaging-quaternions
        match Davidson::new::<DMatrix<f64>>(
            nalgebra::convert(rotation_matrix),
            1,
            DavidsonCorrection::DPR,
            SpectrumTarget::Highest,
            0.0001,
        ) {
            Ok(x) => {
                let ev = x.eigenvectors.column(0);
                isometry.rotation = UnitQuaternion::new_normalize(Quaternion::new(
                    ev[3] as Float,
                    ev[0] as Float,
                    ev[1] as Float,
                    ev[2] as Float,
                ))
            }
            Err(e) => {
                error!("{e}");
            }
        }

        bb.robot_base.set_isometry(isometry);
        bb.robot_base.set_linear_velocity(velocity);
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
            max_delta: self.max_delta,
            velocity_sub: self.velocity_sub,
        };

        start_machine::<CalibrateTransition, _, _, _>(bb, calibrate_localizer).await;
        unreachable!()
    }
}
