//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::{
    collections::hash_map::Entry,
    num::NonZeroUsize,
    ops::DerefMut,
    time::{Duration, Instant},
};

use eigenvalues::{
    lanczos::{HermitianLanczos, LanczosError},
    SpectrumTarget,
};
use frames::{IMUFrame, OrientationFrame, PositionFrame, VelocityFrame};
use fxhash::FxHashMap;
use nalgebra::{
    convert as nconvert, DMatrix, Isometry, Isometry3, Matrix4, Point3, Quaternion, RealField,
    Translation3, UnitQuaternion, UnitVector3, Vector3,
};
use rand::{rngs::SmallRng, Rng};
use rand_distr::{Distribution, Normal};
use rig::{RobotBase, RobotElementRef};
use smach::{State, StateResult};
use unros::{
    anyhow, async_trait,
    pubsub::{subs::DirectSubscription, Subscriber},
    rayon::{
        iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator},
        join,
    },
    rng::quick_rng,
    setup_logging, tokio, Node, NodeIntrinsics, RuntimeContext,
};
use unros::rayon::prelude::ParallelSliceMut;
use unros::rayon::iter::IndexedParallelIterator;
use utils::UnorderedQueue;

pub mod frames;
mod utils;

pub use utils::Float;

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
pub struct Localizer<N: Float> {
    pub point_count: NonZeroUsize,
    pub calibration_duration: Duration,
    pub start_position: Point3<N>,
    pub start_variance: N,
    pub max_delta: Duration,
    pub minimum_unnormalized_weight: N,
    pub undeprivation_factor: N,

    recalibrate_sub: Subscriber<()>,

    imu_sub: Subscriber<IMUFrame<N>>,
    position_sub: Subscriber<PositionFrame<N>>,
    velocity_sub: Subscriber<VelocityFrame<N>>,
    orientation_sub: Subscriber<OrientationFrame<N>>,

    robot_base: RobotBase,
    intrinsics: NodeIntrinsics<Self>,
}

impl<N: Float> Localizer<N> {
    pub fn new(robot_base: RobotBase, start_variance: N) -> Self {
        Self {
            point_count: NonZeroUsize::new(500).unwrap(),
            start_position: Default::default(),
            start_variance,
            calibration_duration: Duration::from_secs(3),
            recalibrate_sub: Subscriber::new(1),
            minimum_unnormalized_weight: nconvert(0.2),
            undeprivation_factor: nconvert(0.3),
            imu_sub: Subscriber::new(1),
            position_sub: Subscriber::new(1),
            orientation_sub: Subscriber::new(1),
            velocity_sub: Subscriber::new(1),
            robot_base,
            max_delta: Duration::from_millis(50),
            intrinsics: Default::default(),
        }
    }

    /// Provide an imu subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_imu_sub(&self) -> DirectSubscription<IMUFrame<N>> {
        self.imu_sub.create_subscription()
    }

    /// Provide a position subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_position_sub(&self) -> DirectSubscription<PositionFrame<N>> {
        self.position_sub.create_subscription()
    }

    /// Provide a velocity subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_velocity_sub(&self) -> DirectSubscription<VelocityFrame<N>> {
        self.velocity_sub.create_subscription()
    }

    /// Provide an orientation subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_orientation_sub(&self) -> DirectSubscription<OrientationFrame<N>> {
        self.orientation_sub.create_subscription()
    }
}

struct CalibratingImu<N: Float> {
    count: usize,
    accel: Vector3<N>,
    angular_velocity: UnitQuaternion<N>,
}

#[derive(Debug)]
struct CalibratedImu<N: Float> {
    accel_scale: N,
    accel_correction: UnitQuaternion<N>,
    angular_velocity_bias: UnitQuaternion<N>,
}

struct LocalizerBlackboard<N: Float> {
    point_count: usize,
    start_position: nalgebra::Vector3<N>,
    start_std_dev: N,
    max_delta: Duration,

    minimum_unnormalized_weight: N,
    undeprivation_factor: N,

    linear_acceleration_std_dev_count: usize,
    angular_velocity_std_dev_count: usize,

    linear_acceleration_std_devs: UnorderedQueue<N>,
    angular_velocity_std_devs: UnorderedQueue<N>,

    calibration_duration: Duration,

    recalibrate_sub: Subscriber<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu<N>>,
    start_orientation: nalgebra::UnitQuaternion<N>,

    imu_sub: Subscriber<IMUFrame<N>>,
    position_sub: Subscriber<PositionFrame<N>>,
    velocity_sub: Subscriber<VelocityFrame<N>>,
    orientation_sub: Subscriber<OrientationFrame<N>>,

    robot_base: RobotBase,

    context: RuntimeContext,
}

/// The calibration stage of the localizer.
///
/// This stage runs for `calibration_duration` before applying the calibrations and exiting.
async fn calibrate_localizer<N: Float>(
    mut bb: LocalizerBlackboard<N>,
) -> StateResult<LocalizerBlackboard<N>> {
    let context = bb.context;
    setup_logging!(context);
    info!("Calibrating localizer");

    let mut imu_map = FxHashMap::<RobotElementRef, CalibratingImu<N>>::default();
    let mut total_gravity = Vector3::default();

    tokio::select! {
        _ = tokio::time::sleep(bb.calibration_duration) => {}
        _ = async { loop {
            let imu = bb.imu_sub.recv().await;
            total_gravity += imu.acceleration;
            let isometry: Isometry3<N> = nconvert(imu.robot_element.get_isometry_from_base());

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
                accel_scale: nconvert::<_, N>(9.81) / calibrating.accel.magnitude()
                    * nconvert(calibrating.count),
                accel_correction,
                angular_velocity_bias: UnitQuaternion::default()
                    .try_slerp(
                        &calibrating.angular_velocity,
                        N::one() / nconvert(calibrating.count),
                        nconvert(0.01),
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
    bb.into()
}

#[inline]
fn normal<N: Float>(mean: N, std_dev: N, x: N) -> N {
    let two = N::one() + N::one();
    let e: N = nconvert(std::f64::consts::E);
    let tau: N = nconvert(std::f64::consts::TAU);
    e.powf(((x - mean) / std_dev).powi(2) / -two) / std_dev / tau.sqrt()
}

pub fn gravity<N: Float>() -> Vector3<N> {
    Vector3::new(N::zero(), nconvert(-9.81), N::zero())
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
fn random_unit_vector<N: Float>(rng: &mut SmallRng) -> nalgebra::UnitVector3<N> {
    loop {
        let x = rng.gen_range(-1.0..1.0);
        let y = rng.gen_range(-1.0..1.0);
        let z = rng.gen_range(-1.0..1.0);
        let vec: nalgebra::Vector3<N> = nconvert(Vector3::new(x, y, z));
        let length = vec.magnitude();
        if length <= N::one() {
            break UnitVector3::new_unchecked(vec.unscale(length));
        }
    }
}

#[derive(Clone, Copy)]
pub struct Particle<N: RealField> {
    position: nalgebra::Vector3<N>,
    position_weight: N,

    orientation: nalgebra::UnitQuaternion<N>,
    orientation_weight: N,

    linear_velocity: nalgebra::Vector3<N>,
    linear_velocity_weight: N,

    angular_velocity: nalgebra::UnitQuaternion<N>,
    angular_velocity_weight: N,

    linear_acceleration: nalgebra::Vector3<N>,
    linear_acceleration_weight: N,
}

/// The active stage of the localizer.  
/// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
///
/// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
async fn run_localizer<N: Float>(
    mut bb: LocalizerBlackboard<N>,
) -> StateResult<LocalizerBlackboard<N>> {
    let context = bb.context;
    setup_logging!(context);

    let mut rng = quick_rng();
    let default_weight = N::one() / nconvert(bb.point_count);
    let mut particles: Vec<Particle<N>> = (0..bb.point_count)
        .map(|_| {
            // let rotation = UnitQuaternion::from_axis_angle(
            //     &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            //     rng.gen_range(0.0..TAU),
            // );
            // let rotation = UnitQuaternion::default();

            let trans_distr = Normal::new(0.0, bb.start_std_dev.to_f32()).unwrap();

            Particle {
                position: bb.start_position
                    + random_unit_vector(&mut rng)
                        .scale(nconvert(trans_distr.sample(rng.deref_mut()))),
                position_weight: default_weight,
                orientation: bb.start_orientation,
                orientation_weight: default_weight,
                linear_velocity: random_unit_vector(&mut rng).scale(nconvert(trans_distr.sample(rng.deref_mut()))),
                linear_velocity_weight: default_weight,
                angular_velocity: Default::default(),
                angular_velocity_weight: default_weight,
                linear_acceleration: gravity(),
                linear_acceleration_weight: default_weight,
            }
        })
        .collect();
    drop(rng);

    let mut start = Instant::now();
    let mut acceleration_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count);
    let mut linear_velocity_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count);
    let mut position_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count);

    let mut angular_velocity_weights: Vec<(UnitQuaternion<N>, N)> =
        Vec::with_capacity(bb.point_count);
    let mut orientation_weights: Vec<(UnitQuaternion<N>, N)> = Vec::with_capacity(bb.point_count);

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
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                let mut ang_scaled_axis = frame.angular_velocity.scaled_axis();
                ang_scaled_axis = nconvert::<_, UnitQuaternion<N>>(inv_rotation) * ang_scaled_axis;
                frame.angular_velocity = UnitQuaternion::from_scaled_axis(ang_scaled_axis);

                frame.acceleration = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_global_isometry().rotation) * frame.acceleration;

                let calibration = bb.calibrations.get(&frame.robot_element);

                if let Some(calibration) = &calibration {
                    frame.angular_velocity = calibration.angular_velocity_bias.inverse() * frame.angular_velocity;
                    frame.acceleration = calibration.accel_correction * frame.acceleration * calibration.accel_scale;
                };

                {
                    let std_dev = frame.acceleration_variance.sqrt();
                    bb.linear_acceleration_std_devs.push(std_dev);

                    if frame.acceleration_variance == N::zero() {
                        particles.par_iter_mut().for_each(|p| {
                            p.linear_acceleration = frame.acceleration;
                            p.linear_acceleration_weight = default_weight;
                        });
                    } else {
                        let mut sum = particles.par_iter_mut().map(|p| {
                            p.linear_acceleration_weight *= normal(N::zero(), std_dev, (p.linear_acceleration - frame.acceleration).magnitude());
                            p.linear_acceleration_weight
                        })
                        .sum();
                    if sum <= bb.minimum_unnormalized_weight {
                        particles.par_sort_unstable_by(|a, b| a.linear_acceleration_weight.partial_cmp(&b.linear_acceleration_weight).unwrap());
                        let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
                        let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
                        let count: usize = count.to_subset_unchecked();
                        
                        let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
                        particles.par_iter_mut().take(count).for_each(|p| {
                            let mut rng = quick_rng();
                            p.linear_acceleration = frame.acceleration + random_unit_vector(&mut rng).scale(nconvert(distr.sample(rng.deref_mut())));
                            p.linear_acceleration_weight += corrective_weight;
                        });
                        sum =bb.minimum_unnormalized_weight;
                    }
                            particles.par_iter_mut().for_each(|p| {
                                p.linear_acceleration_weight /= sum;
                            });
                        }
                    
                }

                {
                    let std_dev = frame.angular_velocity_variance.sqrt();
                    bb.angular_velocity_std_devs.push(std_dev);

                    if frame.angular_velocity_variance == N::zero() {
                        particles.par_iter_mut().for_each(|p| {
                            p.angular_velocity = frame.angular_velocity;
                            p.angular_velocity_weight = default_weight;
                        });
                    } else {
                        let mut sum = particles.par_iter_mut().map(|p| {
                            p.angular_velocity_weight *= normal(N::zero(), std_dev, frame.angular_velocity.angle_to(&p.angular_velocity));
                            p.angular_velocity_weight
                        })
                        .sum();
                    if sum <= bb.minimum_unnormalized_weight {
                        particles.par_sort_unstable_by(|a, b| a.angular_velocity_weight.partial_cmp(&b.angular_velocity_weight).unwrap());
                        let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
                        let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
                        let count: usize = count.to_subset_unchecked();
                        
                        let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
                        particles.par_iter_mut().take(count).for_each(|p| {
                            let mut rng = quick_rng();
                            p.angular_velocity = UnitQuaternion::from_axis_angle(&random_unit_vector(&mut rng), nconvert(distr.sample(rng.deref_mut()))) * frame.angular_velocity;
                            p.angular_velocity_weight += corrective_weight;
                        });
                        sum =bb.minimum_unnormalized_weight;
                    }
                            particles.par_iter_mut().for_each(|p| {
                                p.angular_velocity_weight /= sum;
                            });
                    }
                }
            }
            mut frame = bb.position_sub.recv() => {
                // Find the position of the robot base based on the observation of the position of an element
                // attached to the robot base.
                let isometry = frame.robot_element.get_isometry_from_base().inverse();
                frame.position = nconvert::<_, Isometry3<N>>(isometry) * frame.position;
                let std_dev = frame.variance.sqrt();

                if frame.variance == N::zero() {
                    particles.par_iter_mut().for_each(|p| {
                        p.position = nconvert(frame.position.coords);
                        p.position_weight = default_weight;
                    });
                } else {
                    let mut sum = particles.par_iter_mut().map(|p| {
                        p.position_weight *= normal(N::zero(), std_dev, (p.position - frame.position.coords).magnitude());
                        p.position_weight
                    })
                    .sum();
                        if sum <= bb.minimum_unnormalized_weight {
                            particles.par_sort_unstable_by(|a, b| a.position_weight.partial_cmp(&b.position_weight).unwrap());
                            let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
                            let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
                            let count: usize = count.to_subset_unchecked();
                            
                            let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
                            particles.par_iter_mut().take(count).for_each(|p| {
                                let mut rng = quick_rng();
                                p.position = frame.position.coords + random_unit_vector(&mut rng).scale(nconvert(distr.sample(rng.deref_mut())));
                                p.position_weight += corrective_weight;
                            });
                            
                    sum =bb.minimum_unnormalized_weight;
                        }
                        particles.par_iter_mut().for_each(|p| {
                            p.position_weight /= sum;
                        });
                    
                }
            }
            mut frame = bb.velocity_sub.recv() => {
                // Find the velocity of the robot base based on the observation of the velocity of an element
                // attached to the robot base.
                frame.velocity = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_isometry_from_base().rotation) * frame.velocity;
                let std_dev = frame.variance.sqrt();

                if frame.variance == N::zero() {
                    particles.par_iter_mut().for_each(|p| {
                        p.linear_velocity = frame.velocity;
                        p.linear_velocity_weight = default_weight;
                    });
                } else {
                    let mut sum = particles.par_iter_mut().map(|p| {
                        p.linear_velocity_weight *= normal(N::zero(), std_dev, (p.linear_velocity - frame.velocity).magnitude());
                        p.linear_velocity_weight
                    })
                    .sum();
                if sum <= bb.minimum_unnormalized_weight {
                    particles.par_sort_unstable_by(|a, b| a.linear_velocity_weight.partial_cmp(&b.linear_velocity_weight).unwrap());
                    let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
                    let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
                    let count: usize = count.to_subset_unchecked();
                    
                    let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
                    particles.par_iter_mut().take(count).for_each(|p| {
                        let mut rng = quick_rng();
                        p.linear_velocity = frame.velocity + random_unit_vector(&mut rng).scale(nconvert(distr.sample(rng.deref_mut())));
                        p.linear_velocity_weight += corrective_weight;
                    });
                    sum =bb.minimum_unnormalized_weight;
                }
                
                        particles.par_iter_mut().for_each(|p| {
                            // if quick_rng().gen_bool(0.002) {
                            //     println!("{}", p.linear_velocity_weight);
                            // }
                            p.linear_velocity_weight /= sum;
                        });
                    
                }
            }
            mut frame = bb.orientation_sub.recv() => {
                // Find the orientation of the robot base based on the observation of the orientation of an element
                // attached to the robot base.
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                frame.orientation = nconvert::<_, UnitQuaternion<N>>(inv_rotation) * frame.orientation;

                let std_dev = frame.variance.sqrt();

                if frame.variance == N::zero() {
                    particles.par_iter_mut().for_each(|p| {
                        p.orientation = frame.orientation;
                        p.orientation_weight = default_weight;
                    });
                } else {
                    let mut sum = particles.par_iter_mut().map(|p| {
                        p.orientation_weight *= normal(N::zero(), std_dev, frame.orientation.angle_to(&p.orientation));
                        p.orientation_weight
                    })
                    .sum();
                if sum <= bb.minimum_unnormalized_weight {
                    particles.par_sort_unstable_by(|a, b| a.orientation_weight.partial_cmp(&b.orientation_weight).unwrap());
                    let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
                    let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
                    let count: usize = count.to_subset_unchecked();
                    
                    let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
                    particles.par_iter_mut().take(count).for_each(|p| {
                        let mut rng = quick_rng();
                        p.orientation = UnitQuaternion::from_axis_angle(&random_unit_vector(&mut rng), nconvert(distr.sample(rng.deref_mut()))) * frame.orientation;
                        p.orientation_weight += corrective_weight;
                    });
                    sum =bb.minimum_unnormalized_weight;
                }
                        particles.par_iter_mut().for_each(|p| {
                            p.orientation_weight /= sum;
                        });
                }
            }
        }

        let delta_duration = start.elapsed();
        let delta: N = nconvert(delta_duration.as_secs_f64());

        join(
            || {
                join(
                    || {
                        let mut running_weight = N::zero();
                        acceleration_weights.clear();
                        particles.iter().for_each(|p| {
                            acceleration_weights.push((p.linear_acceleration, running_weight));
                            running_weight += p.linear_acceleration_weight;
                        });
                        assert!((running_weight - N::one()).abs() < nconvert(1e-5), "{}", (running_weight - N::one()).abs());
                    },
                    || {
                        let mut running_weight = N::zero();
                        linear_velocity_weights.clear();
                        particles.iter().for_each(|p| {
                            linear_velocity_weights.push((p.linear_velocity, running_weight));
                            running_weight += p.linear_velocity_weight;
                        });
                        assert!((running_weight - N::one()).abs() < nconvert(1e-5), "{}", (running_weight - N::one()).abs());
                    },
                );
            },
            || {
                join(
                    || {
                        let mut running_weight = N::zero();
                        position_weights.clear();
                        particles.iter().for_each(|p| {
                            position_weights.push((p.position, running_weight));
                            running_weight += p.position_weight;
                        });
                        assert!((running_weight - N::one()).abs() < nconvert(1e-5), "{}", (running_weight - N::one()).abs());
                    },
                    || {
                        join(
                            || {
                                let mut running_weight = N::zero();
                                angular_velocity_weights.clear();
                                particles.iter().for_each(|p| {
                                    angular_velocity_weights
                                        .push((p.angular_velocity, running_weight));
                                    running_weight += p.angular_velocity_weight;
                                });
                                assert!((running_weight - N::one()).abs() < nconvert(1e-5), "{}", (running_weight - N::one()).abs());
                            },
                            || {
                                let mut running_weight = N::zero();
                                orientation_weights.clear();
                                particles.iter().for_each(|p| {
                                    orientation_weights.push((p.orientation, running_weight));
                                    running_weight += p.orientation_weight;
                                });
                                assert!((running_weight - N::one()).abs() < nconvert(1e-5), "{}", (running_weight - N::one()).abs());
                            },
                        )
                    },
                )
            },
        );

        particles.par_iter_mut().for_each(|p| {
            join(
                || {
                    let mut rng = quick_rng();
                    let mut sample: N = nconvert(rng.gen_range(0.0..1.0f32));

                    for (linear_velocity, weight) in linear_velocity_weights.iter().copied().rev() {
                        if sample >= weight {
                            sample = nconvert(rng.gen_range(0.0..1.0f32));
                            for (accel, weight) in acceleration_weights.iter().copied().rev() {
                                if sample >= weight {
                                    p.linear_velocity =
                                        linear_velocity + (accel - gravity()) * delta;
                                    break;
                                }
                            }
                            break;
                        }
                    }

                    sample = nconvert(rng.gen_range(0.0..1.0f32));
                    for (position, weight) in position_weights.iter().copied().rev() {
                        if sample >= weight {
                            sample = nconvert(rng.gen_range(0.0..1.0f32));
                            for (linear_vel, weight) in
                                linear_velocity_weights.iter().copied().rev()
                            {
                                if sample >= weight {
                                    p.position = position + linear_vel * delta;
                                    break;
                                }
                            }
                            break;
                        }
                    }

                    let mean_std_dev = bb
                        .linear_acceleration_std_devs
                        .as_slice()
                        .iter()
                        .copied()
                        .sum::<N>()
                        / nconvert(bb.linear_acceleration_std_dev_count);
                    let distr = Normal::new(0.0, mean_std_dev.to_f32()).unwrap();
                    let scale: N = nconvert(distr.sample(rng.deref_mut()));
                    // println!("{scale}");
                    p.linear_acceleration +=
                        random_unit_vector(&mut rng).scale(scale);
                },
                || {
                    let mut rng = quick_rng();
                    let mut sample: N = nconvert(rng.gen_range(0.0..1.0f32));

                    for (orientation, weight) in orientation_weights.iter().copied().rev() {
                        if sample >= weight {
                            sample = nconvert(rng.gen_range(0.0..1.0f32));
                            for (ang_vel, weight) in angular_velocity_weights.iter().rev() {
                                if sample >= *weight {
                                    p.orientation = UnitQuaternion::default().try_slerp(ang_vel, delta, nconvert(f32::EPSILON)).unwrap_or_default()
                                        * orientation;
                                    break;
                                }
                            }
                            break;
                        }
                    }

                    let mean_std_dev = bb
                        .angular_velocity_std_devs
                        .as_slice()
                        .iter()
                        .copied()
                        .sum::<N>()
                        / nconvert(bb.angular_velocity_std_dev_count);
                    let distr = Normal::new(0.0, mean_std_dev.to_f32()).unwrap();
                    let rand_quat = UnitQuaternion::from_axis_angle(
                        &random_unit_vector(&mut rng),
                        nconvert::<_, N>(distr.sample(rng.deref_mut())),
                    );
                    p.angular_velocity = rand_quat * p.angular_velocity;
                },
            );
        });

        let ((position, linear_velocity, _acceleration), (_angular_velocity, orientation)) = join(
            || {
                let (mut position, mut linear_velocity, mut acceleration) = particles
                    .par_iter()
                    .map(|p| (p.position, p.linear_velocity, p.linear_acceleration))
                    .reduce(
                        || (Vector3::default(), Vector3::default(), Vector3::default()),
                        |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2),
                    );

                position.unscale_mut(nconvert(bb.point_count));
                linear_velocity.unscale_mut(nconvert(bb.point_count));
                acceleration.unscale_mut(nconvert(bb.point_count));

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
                    || match quat_mean(particles.iter().map(|x| x.orientation)).unwrap() {
                        Ok(x) => x,
                        Err(e) => {
                            error!("{e}");
                            nconvert(bb.robot_base.get_isometry().rotation)
                        }
                    },
                )
            },
        );

        // println!("{acceleration:?}");
        bb.robot_base.set_isometry(Isometry::from_parts(
            nconvert(Translation3::from(position)),
            nconvert(orientation),
        ));
        bb.robot_base.set_linear_velocity(nconvert(linear_velocity));

        start += delta_duration;
    }
    bb.context = context;
    bb.into()
}

#[async_trait]
impl<N: Float> Node for Localizer<N> {
    const DEFAULT_NAME: &'static str = "positioning";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let bb = LocalizerBlackboard {
            point_count: self.point_count.get(),
            start_position: self.start_position.coords,
            start_std_dev: self.start_variance.sqrt(),
            calibration_duration: self.calibration_duration,
            recalibrate_sub: self.recalibrate_sub,
            calibrations: Default::default(),
            imu_sub: self.imu_sub,
            position_sub: self.position_sub,
            orientation_sub: self.orientation_sub,
            context,
            minimum_unnormalized_weight: self.minimum_unnormalized_weight,
            undeprivation_factor: self.undeprivation_factor,
            robot_base: self.robot_base,
            max_delta: self.max_delta,
            velocity_sub: self.velocity_sub,
            start_orientation: Default::default(),
            linear_acceleration_std_dev_count: 10,
            angular_velocity_std_dev_count: 10,
            linear_acceleration_std_devs: std::iter::repeat(N::zero()).take(10).collect(),
            angular_velocity_std_devs: std::iter::repeat(N::zero()).take(10).collect(),
        };

        let (calib, calib_trans) = State::new(calibrate_localizer);
        let (run, run_trans) = State::new(run_localizer);

        let start_state = calib.clone();

        calib_trans.set_transition(move |_| Some(run.clone()));
        run_trans.set_transition(move |_| Some(calib.clone()));

        start_state.start(bb).await;
        unreachable!()
    }
}

fn quat_mean<N, T, I>(quats: T) -> Option<Result<UnitQuaternion<N>, LanczosError>>
where
    N: Float,
    T: IntoIterator<Item = UnitQuaternion<N>, IntoIter = I>,
    I: ExactSizeIterator<Item = UnitQuaternion<N>>,
{
    let quats = quats.into_iter();
    let n = quats.len();
    if n == 0 {
        return None;
    }

    let rotation_matrix: Matrix4<N> = quats
        .map(|q| {
            let q_vec = q.as_vector();
            q_vec * q_vec.transpose() / nconvert::<_, N>(n)
        })
        .sum();

    // https://math.stackexchange.com/questions/61146/averaging-quaternions
    match HermitianLanczos::new::<DMatrix<f64>>(
        nconvert(rotation_matrix),
        10,
        SpectrumTarget::Highest,
    ) {
        Ok(x) => {
            let ev = x.eigenvectors.column(0);
            Some(Ok(UnitQuaternion::new_normalize(Quaternion::new(
                nconvert(ev[3]),
                nconvert(ev[0]),
                nconvert(ev[1]),
                nconvert(ev[2]),
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
