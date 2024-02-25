//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::{
    collections::hash_map::Entry,
    num::NonZeroUsize,
    time::{Duration, Instant},
};

use eigenvalues::{
    algorithms::davidson::DavidsonError, Davidson, DavidsonCorrection, SpectrumTarget,
};
use fxhash::FxHashMap;
use nalgebra::{
    DMatrix, Isometry, Isometry3 as I3, Matrix4, Point3 as P3, Quaternion, Translation3,
    UnitQuaternion as UQ, UnitVector3, Vector3 as V3,
};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use rig::{RobotBase, RobotElementRef};
use smach::{start_machine, Transition};
use unros::{
    anyhow, async_trait,
    pubsub::{Subscriber, Subscription},
    rayon::{
        iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator},
        join,
    },
    rng::QuickRng,
    setup_logging, tokio, Node, NodeIntrinsics, RuntimeContext,
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

impl PositionFrame {
    pub fn rand(position: Point3, variance: Float, robot_element: RobotElementRef) -> Self {
        let mut rng = QuickRng::default();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev).unwrap();

        Self {
            position: position + random_unit_vector(&mut rng).scale(distr.sample(&mut rng)),
            variance,
            robot_element,
        }
    }
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

impl VelocityFrame {
    pub fn rand(velocity: Vector3, variance: Float, robot_element: RobotElementRef) -> Self {
        let mut rng = QuickRng::default();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev).unwrap();

        Self {
            velocity: velocity + random_unit_vector(&mut rng).scale(distr.sample(&mut rng)),
            variance,
            robot_element,
        }
    }
}

/// An orientation and variance measurement.
#[derive(Clone)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion,
    /// Variance of orientation in radians
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

impl OrientationFrame {
    pub fn rand(
        orientation: UnitQuaternion,
        variance: Float,
        robot_element: RobotElementRef,
    ) -> Self {
        let mut rng = QuickRng::default();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev).unwrap();

        Self {
            orientation: UnitQuaternion::from_axis_angle(
                &random_unit_vector(&mut rng),
                distr.sample(&mut rng),
            ) * orientation,
            variance,
            robot_element,
        }
    }
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

impl IMUFrame {
    pub fn rand(
        acceleration: Vector3,
        acceleration_variance: Float,
        angular_velocity: UnitQuaternion,
        angular_velocity_variance: Float,
        robot_element: RobotElementRef,
    ) -> Self {
        let mut rng = QuickRng::default();
        let accel_std_dev = acceleration_variance.sqrt();
        let accel_distr = Normal::new(0.0, accel_std_dev).unwrap();
        let angular_velocity_std_dev = angular_velocity_variance.sqrt();
        let ang_vel_distr = Normal::new(0.0, angular_velocity_std_dev).unwrap();

        IMUFrame {
            acceleration: acceleration
                + random_unit_vector(&mut rng).scale(accel_distr.sample(&mut rng)),
            angular_velocity: UnitQuaternion::from_axis_angle(
                &random_unit_vector(&mut rng),
                ang_vel_distr.sample(&mut rng),
            ) * angular_velocity,
            acceleration_variance,
            angular_velocity_variance,
            robot_element,
        }
    }
}

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
pub struct Localizer {
    pub point_count: NonZeroUsize,
    pub calibration_duration: Duration,
    pub start_position: Point3,
    pub start_variance: Float,
    pub max_delta: Duration,
    pub resistance_modifier: Float,

    recalibrate_sub: Subscriber<()>,

    imu_sub: Subscriber<IMUFrame>,
    position_sub: Subscriber<PositionFrame>,
    velocity_sub: Subscriber<VelocityFrame>,
    orientation_sub: Subscriber<OrientationFrame>,

    robot_base: RobotBase,
    intrinsics: NodeIntrinsics<Self>,
}

impl Localizer {
    pub fn new(robot_base: RobotBase, start_variance: Float) -> Self {
        Self {
            point_count: NonZeroUsize::new(500).unwrap(),
            start_position: Default::default(),
            start_variance,
            calibration_duration: Duration::from_secs(3),
            recalibrate_sub: Subscriber::new(1),
            imu_sub: Subscriber::new(1),
            position_sub: Subscriber::new(1),
            orientation_sub: Subscriber::new(1),
            velocity_sub: Subscriber::new(1),
            robot_base,
            max_delta: Duration::from_millis(50),
            resistance_modifier: 0.2,
            intrinsics: Default::default(),
        }
    }

    /// Provide an imu subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_imu_sub(&self) -> Subscription<IMUFrame> {
        self.imu_sub.create_subscription()
    }

    /// Provide a position subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_position_sub(&self) -> Subscription<PositionFrame> {
        self.position_sub.create_subscription()
    }

    /// Provide a velocity subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_velocity_sub(&self) -> Subscription<VelocityFrame> {
        self.velocity_sub.create_subscription()
    }

    /// Provide an orientation subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_orientation_sub(&self) -> Subscription<OrientationFrame> {
        self.orientation_sub.create_subscription()
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
    resistance_modifier: Float,

    calibration_duration: Duration,

    recalibrate_sub: Subscriber<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu>,
    start_orientation: UnitQuaternion,

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
    bb.start_orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(total_gravity.cross(&-Vector3::y_axis())),
        total_gravity.angle(&-Vector3::y_axis()),
    );
    if !bb.start_orientation.w.is_finite()
        || !bb.start_orientation.i.is_finite()
        || !bb.start_orientation.j.is_finite()
        || !bb.start_orientation.k.is_finite()
    {
        bb.start_orientation = Default::default();
    }
    info!("Localizer calibrated");
    bb.context = context;
    (bb, ())
}

const E: Float = std::f64::consts::E as Float;
const TAU: Float = std::f64::consts::TAU as Float;

#[inline]
fn normal(mean: Float, std_dev: Float, x: Float) -> Float {
    E.powf(((x - mean) / std_dev).powi(2) / -2.0) / std_dev / TAU.sqrt()
}

// #[inline]
// fn rand_quat(rng: &mut QuickRng) -> UnitQuaternion {
//     let u: Float = rng.gen_range(0.0..1.0);
//     let v: Float = rng.gen_range(0.0..1.0);
//     let w: Float = rng.gen_range(0.0..1.0);
//     // h = ( sqrt(1-u) sin(2πv), sqrt(1-u) cos(2πv), sqrt(u) sin(2πw), sqrt(u) cos(2πw))
//     UnitQuaternion::new_unchecked(Quaternion::new(
//         (1.0 - u).sqrt() * (TAU * v).sin(),
//         (1.0 - u).sqrt() * (TAU * v).cos(),
//         u.sqrt() * (TAU * w).sin(),
//         u.sqrt() * (TAU * w).cos(),
//     ))
// }

#[inline]
fn random_unit_vector(rng: &mut QuickRng) -> UnitVector3<Float> {
    loop {
        let x = rng.gen_range(-1.0..1.0);
        let y = rng.gen_range(-1.0..1.0);
        let z = rng.gen_range(-1.0..1.0);
        let vec = Vector3::new(x, y, z);
        let length = vec.magnitude();
        if length <= 1.0 {
            break UnitVector3::new_unchecked(vec.unscale(length));
        }
    }
}

#[derive(Clone, Copy, Default)]
pub struct Particle {
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

            // let rotation = UnitQuaternion::from_axis_angle(
            //     &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            //     rng.gen_range(0.0..TAU),
            // );
            // let rotation = UnitQuaternion::default();

            let trans_distr = Normal::new(0.0, bb.start_std_dev).unwrap();
            let translation = Translation3::from(
                bb.start_position
                    + random_unit_vector(&mut rng).scale(trans_distr.sample(&mut rng)),
            );

            Particle {
                isometry: Isometry::from_parts(translation, bb.start_orientation),
                linear_velocity: Default::default(),
                angular_velocity: Default::default(),
                acceleration: Vector3::new(0.0, -9.81, 0.0),
            }
        })
        .collect();

    let mut start = Instant::now();

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
                    let distr = Normal::new(0.0, std_dev).unwrap();
                    let max_normal = normal(0.0, std_dev, 0.0);

                    if max_normal.is_finite() {
                        particles.par_iter_mut().for_each(|p| {
                            let mut rng = QuickRng::default();
                            let new_accel = p.isometry.rotation * frame.robot_element.get_isometry_from_base().rotation * calibration.accel_correction * frame.acceleration * calibration.accel_scale;
                            if rng.gen_bool(((1.0 - normal(0.0, std_dev, (p.acceleration - new_accel).magnitude()) / max_normal) * bb.resistance_modifier) as f64) {
                                p.acceleration = new_accel + random_unit_vector(&mut rng).scale(distr.sample(&mut rng));
                            }
                        });
                    } else {
                        particles.par_iter_mut().for_each(|p| {
                            p.acceleration = frame.acceleration;
                        });
                    }
                }

                {
                    let std_dev = frame.angular_velocity_variance.sqrt();

                    let distr = Normal::new(0.0, std_dev).unwrap();
                    let max_normal = normal(0.0, std_dev, 0.0);

                    if max_normal.is_finite() {
                        particles.par_iter_mut().for_each(|p| {
                            let mut rng = QuickRng::default();
                            if rng.gen_bool(((1.0 - normal(0.0, std_dev, frame.angular_velocity.angle_to(&p.angular_velocity)) / max_normal) * bb.resistance_modifier) as f64) {
                                p.angular_velocity = UnitQuaternion::from_axis_angle(&random_unit_vector(&mut rng), distr.sample(&mut rng)) * frame.angular_velocity;
                            }
                        });
                    } else {
                        particles.par_iter_mut().for_each(|p| {
                            p.angular_velocity = frame.angular_velocity;
                        });
                    }
                }
            }
            mut frame = bb.position_sub.recv() => {
                // Find the position of the robot base based on the observation of the position of an element
                // attached to the robot base.
                let isometry = frame.robot_element.get_isometry_from_base().inverse();
                frame.position = isometry * frame.position;
                let std_dev = frame.variance.sqrt();

                let distr = Normal::new(0.0, std_dev).unwrap();
                let max_normal = normal(0.0, std_dev, 0.0);

                if max_normal.is_finite() {
                    particles.par_iter_mut().for_each(|p| {
                        let mut rng = QuickRng::default();
                        if rng.gen_bool(((1.0 - normal(0.0, std_dev, (p.isometry.translation.vector - frame.position.coords).magnitude()) / max_normal) * bb.resistance_modifier) as f64) {
                            p.isometry.translation.vector = frame.position.coords + random_unit_vector(&mut rng).scale(distr.sample(&mut rng));
                        }
                    });
                } else {
                    particles.par_iter_mut().for_each(|p| {
                        p.isometry.translation = frame.position.into();
                    });
                }
            }
            mut frame = bb.velocity_sub.recv() => {
                // Find the velocity of the robot base based on the observation of the velocity of an element
                // attached to the robot base.
                frame.velocity = frame.robot_element.get_isometry_from_base().rotation * frame.velocity;
                let std_dev = frame.variance.sqrt();

                let distr = Normal::new(0.0, std_dev).unwrap();
                let max_normal = normal(0.0, std_dev, 0.0);

                if max_normal.is_finite() {
                    particles.par_iter_mut().for_each(|p| {
                        let mut rng = QuickRng::default();
                        if rng.gen_bool(((1.0 - normal(0.0, std_dev, (p.linear_velocity - frame.velocity).magnitude()) / max_normal) * bb.resistance_modifier) as f64) {
                            p.linear_velocity = frame.velocity + random_unit_vector(&mut rng).scale(distr.sample(&mut rng));
                        }
                    });
                } else {
                    particles.par_iter_mut().for_each(|p| {
                        p.linear_velocity = frame.velocity;
                    });
                }
            }
            mut frame = bb.orientation_sub.recv() => {
                // Find the orientation of the robot base based on the observation of the orientation of an element
                // attached to the robot base.
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                frame.orientation = inv_rotation * frame.orientation;

                let std_dev = frame.variance.sqrt();

                let distr = Normal::new(0.0, std_dev).unwrap();
                let max_normal = normal(0.0, std_dev, 0.0);

                if max_normal.is_finite() {
                    particles.par_iter_mut().for_each(|p| {
                        let mut rng = QuickRng::default();
                        if rng.gen_bool(((1.0 - normal(0.0, std_dev, frame.orientation.angle_to(&p.isometry.rotation)) / max_normal) * bb.resistance_modifier) as f64) {
                            p.isometry.rotation = UnitQuaternion::from_axis_angle(&random_unit_vector(&mut rng), distr.sample(&mut rng)) * frame.orientation;
                        }
                    });
                } else {
                    particles.par_iter_mut().for_each(|p| {
                        p.isometry.rotation = frame.orientation;
                    });
                }
            }
        }

        let ((position, linear_velocity, acceleration), (angular_velocity, orientation)) = join(
            || {
                let (mut position, mut linear_velocity, mut acceleration) = particles
                    .par_iter()
                    .map(|p| {
                        (
                            p.isometry.translation.vector,
                            p.linear_velocity,
                            p.acceleration,
                        )
                    })
                    .reduce(
                        || (Vector3::default(), Vector3::default(), Vector3::default()),
                        |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2),
                    );

                position.unscale_mut(bb.point_count as Float);
                linear_velocity.unscale_mut(bb.point_count as Float);
                acceleration.unscale_mut(bb.point_count as Float);

                (position, linear_velocity, acceleration)
            },
            || {
                join(
                    || match quat_mean(particles.iter().map(|x| x.angular_velocity)).unwrap() {
                        Ok(x) => x,
                        Err(e) => {
                            error!("{e}");
                            Default::default()
                        }
                    },
                    || match quat_mean(particles.iter().map(|x| x.isometry.rotation)).unwrap() {
                        Ok(x) => x,
                        Err(e) => {
                            error!("{e}");
                            bb.robot_base.get_isometry().rotation
                        }
                    },
                )
            },
        );

        bb.robot_base
            .set_isometry(Isometry::from_parts(position.into(), orientation));
        bb.robot_base.set_linear_velocity(linear_velocity);

        // Apply the predicted position, orientation, and velocity onto the robot base such that
        // other nodes can observe it.
        let delta_duration = start.elapsed();
        let delta = delta_duration.as_secs_f64() as Float;

        particles.par_iter_mut().for_each(|p| {
            p.linear_velocity += (acceleration + Vector3::new(0.0, 9.81, 0.0)) * delta;
            p.isometry.translation.vector += linear_velocity * delta;
            p.isometry.rotation = UnitQuaternion::default()
                .try_slerp(&angular_velocity, delta, 0.001)
                .unwrap_or_default()
                * p.isometry.rotation;
        });
        start += delta_duration;
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

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let bb = LocalizerBlackboard {
            point_count: self.point_count.get(),
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
            start_orientation: Default::default(),
            resistance_modifier: self.resistance_modifier,
        };

        start_machine::<CalibrateTransition, _, _, _>(bb, calibrate_localizer).await;
        unreachable!()
    }
}

fn quat_mean<T, I>(quats: T) -> Option<Result<UnitQuaternion, DavidsonError>>
where
    T: IntoIterator<Item = UnitQuaternion, IntoIter = I>,
    I: ExactSizeIterator<Item = UnitQuaternion>,
{
    let quats = quats.into_iter();
    let n = quats.len();
    if n == 0 {
        return None;
    }

    let rotation_matrix: Matrix4<Float> = quats
        .map(|q| {
            let q_vec = q.as_vector();
            q_vec * q_vec.transpose() / n as Float
        })
        .sum();

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
            Some(Ok(UnitQuaternion::new_normalize(Quaternion::new(
                ev[3] as Float,
                ev[0] as Float,
                ev[1] as Float,
                ev[2] as Float,
            ))))
        }
        Err(e) => Some(Err(e)),
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, UnitVector3};

    use crate::{quat_mean, Float, Vector3};

    const EPSILON: Float = 0.001;

    #[test]
    fn quat_mean_zeroes() {
        assert_eq!(
            quat_mean([Default::default(); 30]).unwrap().unwrap(),
            Default::default()
        );
    }

    #[test]
    fn quat_mean_all_equal() {
        let quat = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            0.4,
        );
        assert!(quat_mean([quat; 30]).unwrap().unwrap().angle_to(&quat) < EPSILON);
    }

    #[test]
    fn quat_mean_all_opposing() {
        let quat01 = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            0.4,
        );
        let quat02 = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            -0.4,
        );
        assert!(
            quat_mean([quat01, quat02])
                .unwrap()
                .unwrap()
                .angle_to(&Default::default())
                < EPSILON
        );
    }
}
