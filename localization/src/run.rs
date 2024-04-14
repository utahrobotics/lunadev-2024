//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::collections::VecDeque;
use std::sync::OnceLock;
use std::{ops::DerefMut, time::Instant};

use crate::utils::{gravity, normal, quat_mean, random_unit_vector};
use nalgebra::{convert as nconvert, Isometry3, Translation3, UnitQuaternion, Vector3};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use smach::StateResult;
use smartcore::ensemble::random_forest_regressor::RandomForestRegressor as RFR;
use smartcore::linalg::basic::matrix::DenseMatrix;
use unros::rayon::iter::IndexedParallelIterator;
use unros::rayon::prelude::ParallelSliceMut;
use unros::tokio::task::block_in_place;
use unros::{
    rayon::{
        iter::{IntoParallelRefMutIterator, ParallelIterator},
        join,
    },
    rng::quick_rng,
    setup_logging, tokio,
};

use crate::{Float, LocalizerBlackboard};

type RandomForestRegressor = RFR<f32, f32, DenseMatrix<f32>, Vec<f32>>;

const OBSERVATIONS: usize = 2;

static FOREST: OnceLock<RandomForestRegressor> = OnceLock::new();

fn get_forest() -> &'static RandomForestRegressor {
    FOREST.get_or_init(|| {
        bincode::deserialize(include_bytes!("model.bin"))
            .expect("Failed to deserialize random forest regressor")
    })
}

#[derive(Clone, Copy)]
struct Particle<N: Float> {
    orientation: nalgebra::UnitQuaternion<N>,
    orientation_weight: N,

    angular_velocity: nalgebra::UnitQuaternion<N>,
    angular_velocity_weight: N,
}

/// The active stage of the localizer.  
/// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
///
/// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
pub(super) async fn run_localizer<N: Float>(
    mut bb: LocalizerBlackboard<N>,
) -> StateResult<LocalizerBlackboard<N>> {
    let context = bb.context.unwrap();
    setup_logging!(context);

    // let mut rng = quick_rng();
    let default_weight = N::one() / nconvert(bb.point_count.get());
    let mut particles: Vec<Particle<N>> = (0..bb.point_count.get())
        .map(|_| {
            // let rotation = UnitQuaternion::from_axis_angle(
            //     &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            //     rng.gen_range(0.0..TAU),
            // );
            // let rotation = UnitQuaternion::default();

            // let trans_distr = Normal::new(0.0, bb.start_std_dev.to_f32()).unwrap();
            // let start_position =
            //     nconvert::<_, Vector3<N>>(bb.robot_base.get_isometry().translation.vector);

            Particle {
                orientation: bb.start_orientation,
                orientation_weight: default_weight,
                angular_velocity: Default::default(),
                angular_velocity_weight: default_weight,
            }
        })
        .collect();
    // drop(rng);

    let mut accel_obs = VecDeque::from_iter([(Vector3::default(), N::zero()); OBSERVATIONS]);
    let mut vel_obs = VecDeque::from_iter([(Vector3::default(), N::zero()); OBSERVATIONS]);
    let mut pos_obs = VecDeque::from_iter([(Vector3::default(), N::zero()); OBSERVATIONS]);

    let mut start = Instant::now();
    // let mut acceleration_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count.get());
    // let mut linear_velocity_weights: Vec<(Vector3<N>, N)> =
    //     Vec::with_capacity(bb.point_count.get());
    // let mut position_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count.get());

    let mut angular_velocity_weights: Vec<(UnitQuaternion<N>, N)> =
        Vec::with_capacity(bb.point_count.get());
    let mut orientation_weights: Vec<(UnitQuaternion<N>, N)> =
        Vec::with_capacity(bb.point_count.get());

    loop {
        // Simultaneously watch three different subscriptions at once.
        // 1. IMU observations
        // 2. Position observations
        // 3. Orientation observations
        tokio::select! {
            // Check for recalibration while simultaneously feeding observations into the algorithm
            () = bb.recalibrate_sub.recv() => {
                break;
            }
            // Process system if max_delta time has passed and no observations were received
            () = tokio::time::sleep(bb.max_delta) => {}
            // IMU Observations
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

                let mut std_dev = frame.acceleration_variance.sqrt();
                bb.linear_acceleration_std_devs.push(std_dev);

                accel_obs.pop_back();
                accel_obs.push_front((frame.acceleration, std_dev));

                std_dev = frame.angular_velocity_variance.sqrt();
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
                        sum = bb.minimum_unnormalized_weight;
                    }
                    particles.par_iter_mut().for_each(|p| {
                        p.angular_velocity_weight /= sum;
                    });
                }
            }

            // Position Observations
            mut frame = bb.position_sub.recv() => {
                // Find the position of the robot base based on the observation of the position of an element
                // attached to the robot base.
                let isometry = frame.robot_element.get_isometry_from_base().inverse();
                frame.position = nconvert::<_, Isometry3<N>>(isometry) * frame.position;
                let std_dev = frame.variance.sqrt();

                pos_obs.pop_back();
                pos_obs.push_front((frame.position.coords, std_dev));
            }

            // Velocity Observations
            mut frame = bb.velocity_sub.recv() => {
                // Find the velocity of the robot base based on the observation of the velocity of an element
                // attached to the robot base.
                frame.velocity = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_isometry_from_base().rotation) * frame.velocity;
                let std_dev = frame.variance.sqrt();

                vel_obs.pop_back();
                vel_obs.push_front((frame.velocity, std_dev));
            }

            // Orientation Observations
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
                        sum = bb.minimum_unnormalized_weight;
                    }
                    particles.par_iter_mut().for_each(|p| {
                        p.orientation_weight /= sum;
                    });
                }
            }
        }

        block_in_place(|| {
            let delta_duration = start.elapsed();
            let delta: N = nconvert(delta_duration.as_secs_f64());
            start += delta_duration;

            // Get running weights for each particle for easier sampling
            join(
                || {
                    let mut running_weight = N::zero();
                    angular_velocity_weights.clear();
                    particles.iter().for_each(|p| {
                        angular_velocity_weights
                            .push((p.angular_velocity, running_weight));
                        running_weight += p.angular_velocity_weight;
                    });
                    assert!(
                        (running_weight - N::one()).abs() < nconvert(1e-4),
                        "{}",
                        (running_weight - N::one()).abs()
                    );
                },
                || {
                    let mut running_weight = N::zero();
                    orientation_weights.clear();
                    particles.iter().for_each(|p| {
                        orientation_weights.push((p.orientation, running_weight));
                        running_weight += p.orientation_weight;
                    });
                    assert!(
                        (running_weight - N::one()).abs() < nconvert(1e-4),
                        "{}",
                        (running_weight - N::one()).abs()
                    );
                },
            );

            // Resample particles
            particles.par_iter_mut().for_each(|p| {
                // Concurrently resample translation and orientation
                join(
                    || {
                        // TODO
                    },
                    || {
                        let mut rng = quick_rng();
                        let mut sample: N = nconvert(rng.gen_range(0.0..1.0f32));

                        for (orientation, weight) in orientation_weights.iter().copied().rev() {
                            if sample >= weight {
                                sample = nconvert(rng.gen_range(0.0..1.0f32));
                                for (ang_vel, weight) in angular_velocity_weights.iter().rev() {
                                    if sample >= *weight {
                                        p.orientation = UnitQuaternion::default()
                                            .try_slerp(ang_vel, delta, nconvert(f32::EPSILON))
                                            .unwrap_or_default()
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

            // Apply likelihood table
            let (ang_vel_sum, orient_sum) = particles
                .par_iter_mut()
                .map(|p| {
                    p.angular_velocity_weight *=
                        (bb.likelihood_table.angular_velocity)(&mut p.angular_velocity);
                    p.orientation_weight *= (bb.likelihood_table.orientation)(&mut p.orientation);
                    (
                        p.angular_velocity_weight,
                        p.orientation_weight,
                    )
                })
                .reduce(
                    || (N::zero(), N::zero()),
                    |a, b| (a.0 + b.0, a.1 + b.1),
                );

            // if pos_sum == N::zero() {

            // }

            // Normalize weights
            particles.par_iter_mut().for_each(|p| {
                if ang_vel_sum != N::zero() {
                    p.angular_velocity_weight /= ang_vel_sum;
                }
                if orient_sum != N::zero() {
                    p.orientation_weight /= orient_sum;
                }
            });

            let forest = get_forest();
            let accel_obs_slice: &[_] = accel_obs.make_contiguous();
            let accel_x_obs = accel_obs_slice.iter().flat_map(|(p, c)|
                [p.x.to_f32(), c.to_f32()]
            ).collect();
            let acceleration_x = forest.predict(&DenseMatrix::from_2d_vec(&vec![accel_x_obs])).unwrap()[0];

            let accel_y_obs = accel_obs_slice.iter().flat_map(|(p, c)|
                [p.y.to_f32(), c.to_f32()]
            ).collect();
            let acceleration_y = forest.predict(&DenseMatrix::from_2d_vec(&vec![accel_y_obs])).unwrap()[0];

            let accel_z_obs = accel_obs_slice.iter().flat_map(|(p, c)|
                [p.z.to_f32(), c.to_f32()]
            ).collect();
            let acceleration_z = forest.predict(&DenseMatrix::from_2d_vec(&vec![accel_z_obs])).unwrap()[0];

            let vel_obs_slice: &[_] = vel_obs.make_contiguous();
            let vel_x_obs = vel_obs_slice.iter().flat_map(|(p, c)|
                [p.x.to_f32(), c.to_f32()]
            ).collect();
            let velocity_x = forest.predict(&DenseMatrix::from_2d_vec(&vec![vel_x_obs])).unwrap()[0];

            let vel_y_obs = vel_obs_slice.iter().flat_map(|(p, c)|
                [p.y.to_f32(), c.to_f32()]
            ).collect();
            let velocity_y = forest.predict(&DenseMatrix::from_2d_vec(&vec![vel_y_obs])).unwrap()[0];

            let vel_z_obs = vel_obs_slice.iter().flat_map(|(p, c)|
                [p.z.to_f32(), c.to_f32()]
            ).collect();
            let velocity_z = forest.predict(&DenseMatrix::from_2d_vec(&vec![vel_z_obs])).unwrap()[0];

            let pos_obs_slice: &[_] = pos_obs.make_contiguous();
            let pos_x_obs = pos_obs_slice.iter().flat_map(|(p, c)|
                [p.x.to_f32(), c.to_f32()]
            ).collect();
            let position_x = forest.predict(&DenseMatrix::from_2d_vec(&vec![pos_x_obs])).unwrap()[0];

            let pos_y_obs = pos_obs_slice.iter().flat_map(|(p, c)|
                [p.y.to_f32(), c.to_f32()]
            ).collect();
            let position_y = forest.predict(&DenseMatrix::from_2d_vec(&vec![pos_y_obs])).unwrap()[0];

            let pos_z_obs = pos_obs_slice.iter().flat_map(|(p, c)|
                [p.z.to_f32(), c.to_f32()]
            ).collect();
            let position_z = forest.predict(&DenseMatrix::from_2d_vec(&vec![pos_z_obs])).unwrap()[0];

            let acceleration: Vector3<N> = nconvert(Vector3::new(acceleration_x, acceleration_y, acceleration_z));
            let linear_velocity: Vector3<N> = nconvert(Vector3::new(velocity_x, velocity_y, velocity_z));
            let position: Vector3<N> = nconvert(Vector3::new(position_x, position_y, position_z));

            vel_obs.pop_back();
            vel_obs.push_front((linear_velocity + (acceleration - gravity()) * delta, N::zero()));

            pos_obs.pop_back();
            pos_obs.push_front((position + linear_velocity * delta, N::zero()));

            // Get mean position, linear_velocity, acceleration, angular_velocity, and orientation
            let (_angular_velocity, orientation) =
                join(
                    || match quat_mean(
                        particles
                            .iter()
                            .map(|x| (x.angular_velocity, x.angular_velocity_weight)),
                    )
                    .unwrap()
                    {
                        Ok(x) => x,
                        Err(e) => {
                            error!("{e}");
                            Default::default()
                        }
                    },
                    || match quat_mean(
                        particles
                            .iter()
                            .map(|x| (x.orientation, x.orientation_weight)),
                    )
                    .unwrap()
                    {
                        Ok(x) => x,
                        Err(e) => {
                            error!("{e}");
                            nconvert(bb.robot_base.get_isometry().rotation)
                        }
                    },
                );

            // Update robot base
            bb.robot_base.set_isometry(Isometry3::from_parts(
                nconvert(Translation3::from(position)),
                nconvert(orientation),
            ));
            bb.robot_base.set_linear_velocity(nconvert(linear_velocity));

            // Calculate the sum of squared differences from the mean
            // let sum_squared_diff = particles
            //     .par_iter_mut()
            //     .map(|p| {
            //         let diff = p.linear_velocity - linear_velocity;
            //         diff.component_mul(&diff)
            //     })
            //     .reduce(|| Vector3::default(), |a, b| a + b);

            // Calculate the variance
            // let variance = sum_squared_diff / nconvert::<_, N>((bb.point_count.get() - 1).max(1));
            // println!("{:.2} {:.2} {:.2}", variance.x, variance.y, variance.z);
        });
    }
    bb.context = Some(context);
    bb.into()
}
