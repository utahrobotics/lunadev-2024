// use nalgebra::{Isometry3, Point3, Vector3};

// use crate::Float;

// use super::LocalizerEngine;

// pub struct MclConfig<N: Float> {
//     pub linear_acceleration_std_dev_count: usize,
//     pub angular_velocity_std_dev_count: usize,
//     pub linear_acceleration_std_devs: Vec<N>,
//     pub angular_velocity_std_devs: Vec<N>,
// }

// pub struct MclEngine<N: Float> {

// }


// impl<N: Float> LocalizerEngine<N> for MclEngine<N> {
//     type Config = MclConfig<N>;

//     fn from_config(config: &Self::Config) -> Self {
//         Self {
//         }
//     }

//     fn observe_linear_acceleration(&mut self, accel: Vector3<N>) {
//     }

//     fn observe_linear_velocity(&mut self, vel: Vector3<N>) {
//     }

//     fn observe_position(&mut self, pos: Point3<N>) {
//     }

//     fn observe_angular_velocity(&mut self, ang_vel: Vector3<N>) {
//     }

//     fn observe_orientation(&mut self, orient: Vector3<N>) {
//     }

//     fn get_isometry(&self) -> Isometry3<N> {
//     }

//     fn get_linear_velocity(&self) -> Vector3<N> {
//     }

//     fn get_linear_acceleration(&self) -> Vector3<N> {
//     }

//     fn get_angular_velocity(&self) -> Vector3<N> {
//     }
// }

// #[derive(Clone, Copy)]
// struct Particle<N: Float> {
//     position: nalgebra::Vector3<N>,
//     position_weight: N,

//     orientation: nalgebra::UnitQuaternion<N>,
//     orientation_weight: N,

//     linear_velocity: nalgebra::Vector3<N>,
//     linear_velocity_weight: N,

//     angular_velocity: nalgebra::UnitQuaternion<N>,
//     angular_velocity_weight: N,

//     linear_acceleration: nalgebra::Vector3<N>,
//     linear_acceleration_weight: N,
// }

// /// The active stage of the localizer.  
// /// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
// ///
// /// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
// pub(super) async fn run_localizer<N: Float>(
//     mut bb: LocalizerBlackboard<N>,
// ) -> StateResult<LocalizerBlackboard<N>> {
//     let context = bb.context.unwrap();
//     setup_logging!(context);

//     let mut rng = quick_rng();
//     let default_weight = N::one() / nconvert(bb.point_count.get());
//     let mut particles: Vec<Particle<N>> = (0..bb.point_count.get())
//         .map(|_| {
//             // let rotation = UnitQuaternion::from_axis_angle(
//             //     &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
//             //     rng.gen_range(0.0..TAU),
//             // );
//             // let rotation = UnitQuaternion::default();

//             let trans_distr = Normal::new(0.0, bb.start_std_dev.to_f32()).unwrap();
//             let start_position =
//                 nconvert::<_, Vector3<N>>(bb.robot_base.get_isometry().translation.vector);

//             Particle {
//                 position: start_position
//                     + random_unit_vector(&mut rng)
//                         .scale(nconvert(trans_distr.sample(rng.deref_mut()))),
//                 position_weight: default_weight,
//                 orientation: bb.start_orientation,
//                 orientation_weight: default_weight,
//                 linear_velocity: random_unit_vector(&mut rng)
//                     .scale(nconvert(trans_distr.sample(rng.deref_mut()))),
//                 linear_velocity_weight: default_weight,
//                 angular_velocity: Default::default(),
//                 angular_velocity_weight: default_weight,
//                 linear_acceleration: gravity()
//                     + random_unit_vector(&mut rng)
//                         .scale(nconvert(trans_distr.sample(rng.deref_mut()))),
//                 linear_acceleration_weight: default_weight,
//             }
//         })
//         .collect();
//     drop(rng);

//     let mut start = Instant::now();
//     let mut acceleration_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count.get());
//     let mut linear_velocity_weights: Vec<(Vector3<N>, N)> =
//         Vec::with_capacity(bb.point_count.get());
//     let mut position_weights: Vec<(Vector3<N>, N)> = Vec::with_capacity(bb.point_count.get());

//     let mut angular_velocity_weights: Vec<(UnitQuaternion<N>, N)> =
//         Vec::with_capacity(bb.point_count.get());
//     let mut orientation_weights: Vec<(UnitQuaternion<N>, N)> =
//         Vec::with_capacity(bb.point_count.get());

//     loop {
//         // Simultaneously watch three different subscriptions at once.
//         // 1. IMU observations
//         // 2. Position observations
//         // 3. Orientation observations
//         tokio::select! {
//             // Check for recalibration while simultaneously feeding observations into the algorithm
//             () = bb.recalibrate_sub.recv() => {
//                 break;
//             }
//             // Process system if max_delta time has passed and no observations were received
//             () = tokio::time::sleep(bb.max_delta) => {}
//             // IMU Observations
//             mut frame = bb.imu_sub.recv() => {
//                 let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
//                 let mut ang_scaled_axis = frame.angular_velocity.scaled_axis();
//                 ang_scaled_axis = nconvert::<_, UnitQuaternion<N>>(inv_rotation) * ang_scaled_axis;
//                 frame.angular_velocity = UnitQuaternion::from_scaled_axis(ang_scaled_axis);

//                 frame.acceleration = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_global_isometry().rotation) * frame.acceleration;

//                 let calibration = bb.calibrations.get(&frame.robot_element);

//                 if let Some(calibration) = &calibration {
//                     frame.angular_velocity = calibration.angular_velocity_bias.inverse() * frame.angular_velocity;
//                     frame.acceleration = calibration.accel_correction * frame.acceleration * calibration.accel_scale;
//                 };

//                 let mut std_dev = frame.acceleration_variance.sqrt();
//                 bb.linear_acceleration_std_devs.push(std_dev);

//                 if frame.acceleration_variance == N::zero() {
//                     particles.par_iter_mut().for_each(|p| {
//                         p.linear_acceleration = frame.acceleration;
//                         p.linear_acceleration_weight = default_weight;
//                     });
//                 } else {
//                     let mut sum = particles.par_iter_mut().map(|p| {
//                         p.linear_acceleration_weight *= normal(N::zero(), std_dev, (p.linear_acceleration - frame.acceleration).magnitude());
//                         p.linear_acceleration_weight
//                     })
//                     .sum();
//                     if sum <= bb.minimum_unnormalized_weight {
//                         particles.par_sort_unstable_by(|a, b| a.linear_acceleration_weight.partial_cmp(&b.linear_acceleration_weight).unwrap());
//                         let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
//                         let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
//                         let count: usize = count.to_subset_unchecked();

//                         let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
//                         particles.par_iter_mut().take(count).for_each(|p| {
//                             let mut rng = quick_rng();
//                             p.linear_acceleration = frame.acceleration + random_unit_vector(&mut rng).scale(nconvert(distr.sample(rng.deref_mut())));
//                             p.linear_acceleration_weight += corrective_weight;
//                         });
//                         sum =bb.minimum_unnormalized_weight;
//                     }
//                     particles.par_iter_mut().for_each(|p| {
//                         p.linear_acceleration_weight /= sum;
//                     });
//                 }

//                 std_dev = frame.angular_velocity_variance.sqrt();
//                 bb.angular_velocity_std_devs.push(std_dev);

//                 if frame.angular_velocity_variance == N::zero() {
//                     particles.par_iter_mut().for_each(|p| {
//                         p.angular_velocity = frame.angular_velocity;
//                         p.angular_velocity_weight = default_weight;
//                     });
//                 } else {
//                     let mut sum = particles.par_iter_mut().map(|p| {
//                         p.angular_velocity_weight *= normal(N::zero(), std_dev, frame.angular_velocity.angle_to(&p.angular_velocity));
//                         p.angular_velocity_weight
//                     })
//                     .sum();
//                     if sum <= bb.minimum_unnormalized_weight {
//                         particles.par_sort_unstable_by(|a, b| a.angular_velocity_weight.partial_cmp(&b.angular_velocity_weight).unwrap());
//                         let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
//                         let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
//                         let count: usize = count.to_subset_unchecked();

//                         let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
//                         particles.par_iter_mut().take(count).for_each(|p| {
//                             let mut rng = quick_rng();
//                             p.angular_velocity = UnitQuaternion::from_axis_angle(&random_unit_vector(&mut rng), nconvert(distr.sample(rng.deref_mut()))) * frame.angular_velocity;
//                             p.angular_velocity_weight += corrective_weight;
//                         });
//                         sum = bb.minimum_unnormalized_weight;
//                     }
//                     particles.par_iter_mut().for_each(|p| {
//                         p.angular_velocity_weight /= sum;
//                     });
//                 }
//             }

//             // Position Observations
//             mut frame = bb.position_sub.recv() => {
//                 // Find the position of the robot base based on the observation of the position of an element
//                 // attached to the robot base.
//                 let isometry = frame.robot_element.get_isometry_from_base().inverse();
//                 frame.position = nconvert::<_, Isometry3<N>>(isometry) * frame.position;
//                 let std_dev = frame.variance.sqrt();

//                 if frame.variance == N::zero() {
//                     particles.par_iter_mut().for_each(|p| {
//                         p.position = nconvert(frame.position.coords);
//                         p.position_weight = default_weight;
//                     });
//                 } else {
//                     let mut sum = particles.par_iter_mut().map(|p| {
//                         p.position_weight *= normal(N::zero(), std_dev, (p.position - frame.position.coords).magnitude());
//                         p.position_weight
//                     })
//                     .sum();
//                     if sum <= bb.minimum_unnormalized_weight {
//                         particles.par_sort_unstable_by(|a, b| a.position_weight.partial_cmp(&b.position_weight).unwrap());
//                         let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
//                         let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
//                         let count: usize = count.to_subset_unchecked();

//                         let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
//                         particles.par_iter_mut().take(count).for_each(|p| {
//                             let mut rng = quick_rng();
//                             p.position = frame.position.coords + random_unit_vector(&mut rng).scale(nconvert(distr.sample(rng.deref_mut())));
//                             p.position_weight += corrective_weight;
//                         });

//                         sum = bb.minimum_unnormalized_weight;
//                     }
//                     particles.par_iter_mut().for_each(|p| {
//                         p.position_weight /= sum;
//                     });

//                 }
//             }

//             // Velocity Observations
//             mut frame = bb.velocity_sub.recv() => {
//                 // Find the velocity of the robot base based on the observation of the velocity of an element
//                 // attached to the robot base.
//                 frame.velocity = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_isometry_from_base().rotation) * frame.velocity;
//                 let std_dev = frame.variance.sqrt();

//                 if frame.variance == N::zero() {
//                     particles.par_iter_mut().for_each(|p| {
//                         p.linear_velocity = frame.velocity;
//                         p.linear_velocity_weight = default_weight;
//                     });
//                 } else {
//                     let mut sum = particles.par_iter_mut().map(|p| {
//                         p.linear_velocity_weight *= normal(N::zero(), std_dev, (p.linear_velocity - frame.velocity).magnitude());
//                         p.linear_velocity_weight
//                     })
//                     .sum();
//                     if sum <= bb.minimum_unnormalized_weight {
//                         particles.par_sort_unstable_by(|a, b| a.linear_velocity_weight.partial_cmp(&b.linear_velocity_weight).unwrap());
//                         let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
//                         let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
//                         let count: usize = count.to_subset_unchecked();

//                         let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
//                         particles.par_iter_mut().take(count).for_each(|p| {
//                             let mut rng = quick_rng();
//                             p.linear_velocity = frame.velocity + random_unit_vector(&mut rng).scale(nconvert(distr.sample(rng.deref_mut())));
//                             p.linear_velocity_weight += corrective_weight;
//                         });
//                         sum = bb.minimum_unnormalized_weight;
//                     }

//                     particles.par_iter_mut().for_each(|p| {
//                         p.linear_velocity_weight /= sum;
//                     });

//                 }
//             }

//             // Orientation Observations
//             mut frame = bb.orientation_sub.recv() => {
//                 // Find the orientation of the robot base based on the observation of the orientation of an element
//                 // attached to the robot base.
//                 let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
//                 frame.orientation = nconvert::<_, UnitQuaternion<N>>(inv_rotation) * frame.orientation;

//                 let std_dev = frame.variance.sqrt();

//                 if frame.variance == N::zero() {
//                     particles.par_iter_mut().for_each(|p| {
//                         p.orientation = frame.orientation;
//                         p.orientation_weight = default_weight;
//                     });
//                 } else {
//                     let mut sum = particles.par_iter_mut().map(|p| {
//                         p.orientation_weight *= normal(N::zero(), std_dev, frame.orientation.angle_to(&p.orientation));
//                         p.orientation_weight
//                     })
//                     .sum();
//                     if sum <= bb.minimum_unnormalized_weight {
//                         particles.par_sort_unstable_by(|a, b| a.orientation_weight.partial_cmp(&b.orientation_weight).unwrap());
//                         let count = (nconvert::<_, N>(particles.len()) * bb.undeprivation_factor).ceil();
//                         let corrective_weight = (bb.minimum_unnormalized_weight - sum) / count;
//                         let count: usize = count.to_subset_unchecked();

//                         let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();
//                         particles.par_iter_mut().take(count).for_each(|p| {
//                             let mut rng = quick_rng();
//                             p.orientation = UnitQuaternion::from_axis_angle(&random_unit_vector(&mut rng), nconvert(distr.sample(rng.deref_mut()))) * frame.orientation;
//                             p.orientation_weight += corrective_weight;
//                         });
//                         sum = bb.minimum_unnormalized_weight;
//                     }
//                     particles.par_iter_mut().for_each(|p| {
//                         p.orientation_weight /= sum;
//                     });
//                 }
//             }
//         }

//         block_in_place(|| {
//             let delta_duration = start.elapsed();
//             let delta: N = nconvert(delta_duration.as_secs_f64());
//             start += delta_duration;

//             // Get running weights for each particle for easier sampling
//             join(
//                 || {
//                     join(
//                         || {
//                             let mut running_weight = N::zero();
//                             acceleration_weights.clear();
//                             particles.iter().for_each(|p| {
//                                 acceleration_weights.push((p.linear_acceleration, running_weight));
//                                 running_weight += p.linear_acceleration_weight;
//                             });
//                             assert!(
//                                 (running_weight - N::one()).abs() < nconvert(1e-4),
//                                 "{}",
//                                 (running_weight - N::one()).abs()
//                             );
//                         },
//                         || {
//                             let mut running_weight = N::zero();
//                             linear_velocity_weights.clear();
//                             particles.iter().for_each(|p| {
//                                 linear_velocity_weights.push((p.linear_velocity, running_weight));
//                                 running_weight += p.linear_velocity_weight;
//                             });
//                             assert!(
//                                 (running_weight - N::one()).abs() < nconvert(1e-4),
//                                 "{} {}",
//                                 (running_weight - N::one()).abs(),
//                                 bb.robot_base.get_linear_velocity().magnitude()
//                             );
//                         },
//                     );
//                 },
//                 || {
//                     join(
//                         || {
//                             let mut running_weight = N::zero();
//                             position_weights.clear();
//                             particles.iter().for_each(|p| {
//                                 position_weights.push((p.position, running_weight));
//                                 running_weight += p.position_weight;
//                             });
//                             assert!(
//                                 (running_weight - N::one()).abs() < nconvert(1e-4),
//                                 "{}",
//                                 (running_weight - N::one()).abs()
//                             );
//                         },
//                         || {
//                             join(
//                                 || {
//                                     let mut running_weight = N::zero();
//                                     angular_velocity_weights.clear();
//                                     particles.iter().for_each(|p| {
//                                         angular_velocity_weights
//                                             .push((p.angular_velocity, running_weight));
//                                         running_weight += p.angular_velocity_weight;
//                                     });
//                                     assert!(
//                                         (running_weight - N::one()).abs() < nconvert(1e-4),
//                                         "{}",
//                                         (running_weight - N::one()).abs()
//                                     );
//                                 },
//                                 || {
//                                     let mut running_weight = N::zero();
//                                     orientation_weights.clear();
//                                     particles.iter().for_each(|p| {
//                                         orientation_weights.push((p.orientation, running_weight));
//                                         running_weight += p.orientation_weight;
//                                     });
//                                     assert!(
//                                         (running_weight - N::one()).abs() < nconvert(1e-4),
//                                         "{}",
//                                         (running_weight - N::one()).abs()
//                                     );
//                                 },
//                             )
//                         },
//                     )
//                 },
//             );

//             // Resample particles
//             particles.par_iter_mut().for_each(|p| {
//                 // Concurrently resample translation and orientation
//                 join(
//                     || {
//                         let mut rng = quick_rng();
//                         let mut sample: N = nconvert(rng.gen_range(0.0..1.0f32));

//                         for (linear_velocity, weight) in
//                             linear_velocity_weights.iter().copied().rev()
//                         {
//                             if sample >= weight {
//                                 sample = nconvert(rng.gen_range(0.0..1.0f32));
//                                 for (accel, weight) in acceleration_weights.iter().copied().rev() {
//                                     if sample >= weight {
//                                         p.linear_velocity =
//                                             linear_velocity + (accel - gravity()) * delta;
//                                         break;
//                                     }
//                                 }
//                                 break;
//                             }
//                         }

//                         sample = nconvert(rng.gen_range(0.0..1.0f32));
//                         for (position, weight) in position_weights.iter().copied().rev() {
//                             if sample >= weight {
//                                 sample = nconvert(rng.gen_range(0.0..1.0f32));
//                                 for (linear_vel, weight) in
//                                     linear_velocity_weights.iter().copied().rev()
//                                 {
//                                     if sample >= weight {
//                                         p.position = position + linear_vel * delta;
//                                         break;
//                                     }
//                                 }
//                                 break;
//                             }
//                         }

//                         let mean_std_dev = bb
//                             .linear_acceleration_std_devs
//                             .as_slice()
//                             .iter()
//                             .copied()
//                             .sum::<N>()
//                             / nconvert(bb.linear_acceleration_std_dev_count);
//                         let distr = Normal::new(0.0, mean_std_dev.to_f32()).unwrap();
//                         let scale: N = nconvert(distr.sample(rng.deref_mut()));
//                         p.linear_acceleration += random_unit_vector(&mut rng).scale(scale);
//                     },
//                     || {
//                         let mut rng = quick_rng();
//                         let mut sample: N = nconvert(rng.gen_range(0.0..1.0f32));

//                         for (orientation, weight) in orientation_weights.iter().copied().rev() {
//                             if sample >= weight {
//                                 sample = nconvert(rng.gen_range(0.0..1.0f32));
//                                 for (ang_vel, weight) in angular_velocity_weights.iter().rev() {
//                                     if sample >= *weight {
//                                         p.orientation = UnitQuaternion::default()
//                                             .try_slerp(ang_vel, delta, nconvert(f32::EPSILON))
//                                             .unwrap_or_default()
//                                             * orientation;
//                                         break;
//                                     }
//                                 }
//                                 break;
//                             }
//                         }

//                         let mean_std_dev = bb
//                             .angular_velocity_std_devs
//                             .as_slice()
//                             .iter()
//                             .copied()
//                             .sum::<N>()
//                             / nconvert(bb.angular_velocity_std_dev_count);
//                         let distr = Normal::new(0.0, mean_std_dev.to_f32()).unwrap();
//                         let rand_quat = UnitQuaternion::from_axis_angle(
//                             &random_unit_vector(&mut rng),
//                             nconvert::<_, N>(distr.sample(rng.deref_mut())),
//                         );
//                         p.angular_velocity = rand_quat * p.angular_velocity;
//                     },
//                 );
//             });

//             // Apply likelihood table
//             let (pos_sum, vel_sum, accel_sum, ang_vel_sum, orient_sum) = particles
//                 .par_iter_mut()
//                 .map(|p| {
//                     p.position_weight *= (bb.likelihood_table.position)(&mut p.position);
//                     p.linear_velocity_weight *=
//                         (bb.likelihood_table.linear_velocity)(&mut p.linear_velocity);
//                     p.linear_acceleration_weight *=
//                         (bb.likelihood_table.linear_acceleration)(&mut p.linear_acceleration);
//                     p.angular_velocity_weight *=
//                         (bb.likelihood_table.angular_velocity)(&mut p.angular_velocity);
//                     p.orientation_weight *= (bb.likelihood_table.orientation)(&mut p.orientation);
//                     (
//                         p.position_weight,
//                         p.linear_velocity_weight,
//                         p.linear_acceleration_weight,
//                         p.angular_velocity_weight,
//                         p.orientation_weight,
//                     )
//                 })
//                 .reduce(
//                     || (N::zero(), N::zero(), N::zero(), N::zero(), N::zero()),
//                     |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2, a.3 + b.3, a.4 + b.4),
//                 );

//             // if pos_sum == N::zero() {

//             // }

//             // Normalize weights
//             particles.par_iter_mut().for_each(|p| {
//                 if pos_sum != N::zero() {
//                     p.position_weight /= pos_sum;
//                 }
//                 if vel_sum != N::zero() {
//                     p.linear_velocity_weight /= vel_sum;
//                 }
//                 if accel_sum != N::zero() {
//                     p.linear_acceleration_weight /= accel_sum;
//                 }
//                 if ang_vel_sum != N::zero() {
//                     p.angular_velocity_weight /= ang_vel_sum;
//                 }
//                 if orient_sum != N::zero() {
//                     p.orientation_weight /= orient_sum;
//                 }
//             });

//             // Get mean position, linear_velocity, acceleration, angular_velocity, and orientation
//             let ((position, linear_velocity, _acceleration), (_angular_velocity, orientation)) =
//                 join(
//                     || {
//                         particles
//                             .par_iter()
//                             .map(|p| {
//                                 (
//                                     p.position * p.position_weight,
//                                     p.linear_velocity * p.linear_velocity_weight,
//                                     p.linear_acceleration * p.linear_acceleration_weight,
//                                 )
//                             })
//                             .reduce(
//                                 || (Vector3::default(), Vector3::default(), Vector3::default()),
//                                 |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2),
//                             )
//                     },
//                     || {
//                         join(
//                             || match quat_mean(
//                                 particles
//                                     .iter()
//                                     .map(|x| (x.angular_velocity, x.angular_velocity_weight)),
//                             )
//                             .unwrap()
//                             {
//                                 Ok(x) => x,
//                                 Err(e) => {
//                                     error!("{e}");
//                                     Default::default()
//                                 }
//                             },
//                             || match quat_mean(
//                                 particles
//                                     .iter()
//                                     .map(|x| (x.orientation, x.orientation_weight)),
//                             )
//                             .unwrap()
//                             {
//                                 Ok(x) => x,
//                                 Err(e) => {
//                                     error!("{e}");
//                                     nconvert(bb.robot_base.get_isometry().rotation)
//                                 }
//                             },
//                         )
//                     },
//                 );

//             // Update robot base
//             bb.robot_base.set_isometry(Isometry3::from_parts(
//                 nconvert(Translation3::from(position)),
//                 nconvert(orientation),
//             ));
//             bb.robot_base.set_linear_velocity(nconvert(linear_velocity));

//             // Calculate the sum of squared differences from the mean
//             // let sum_squared_diff = particles
//             //     .par_iter_mut()
//             //     .map(|p| {
//             //         let diff = p.linear_velocity - linear_velocity;
//             //         diff.component_mul(&diff)
//             //     })
//             //     .reduce(|| Vector3::default(), |a, b| a + b);

//             // Calculate the variance
//             // let variance = sum_squared_diff / nconvert::<_, N>((bb.point_count.get() - 1).max(1));
//             // println!("{:.2} {:.2} {:.2}", variance.x, variance.y, variance.z);
//         });
//     }
//     bb.context = Some(context);
//     bb.into()
// }


// /// A collection of functions that can be used to incorporate the likelihoods of more nuanced scenarios.
// ///
// /// For example, you may know the top speed of your robot, so you can assign all velocities
// /// above that speed a likelihood of zero. Do not that a likelihood of 1 *does not* mean 100% possible,
// /// whereas a likelihood of 0 *does* mean impossible. The effect of non-zero likelihoods depends on
// /// how the likelihood function works against all values in the filter.
// ///
// /// If there are two particles in the filter, and a likelihood function returns 1000 for the velocity
// /// of both particles, it means that both particles are equally likely to be true. This would be true
// /// even if the likelihood was 1 for both, or even 0.00001 for both. In other words, the ratio between
// /// likelihoods matters more than the magnitude.
// pub struct LikelihoodTable<N: Float> {
//     pub position: Box<dyn Fn(&mut Vector3<N>) -> N + Send + Sync>,
//     pub linear_velocity: Box<dyn Fn(&mut Vector3<N>) -> N + Send + Sync>,
//     pub linear_acceleration: Box<dyn Fn(&mut Vector3<N>) -> N + Send + Sync>,

//     pub orientation: Box<dyn Fn(&mut UnitQuaternion<N>) -> N + Send + Sync>,
//     pub angular_velocity: Box<dyn Fn(&mut UnitQuaternion<N>) -> N + Send + Sync>,
// }



// impl<N: Float> Default for LikelihoodTable<N> {
//     fn default() -> Self {
//         Self {
//             position: Box::new(|_| N::one()),
//             linear_velocity: Box::new(|_| N::one()),
//             linear_acceleration: Box::new(|_| N::one()),
//             orientation: Box::new(|_| N::one()),
//             angular_velocity: Box::new(|_| N::one()),
//         }
//     }
// }
// /// The number of standard deviations to keep track of for linear acceleration.
// ///
// /// The estimate for linear acceleration worsens by the mean of the standard deviations.
// pub linear_acceleration_std_dev_count: usize,
// /// The number of standard deviations to keep track of for angular velocity.
// ///
// /// The estimate for angular velocity worsens by the mean of the standard deviations.
// pub angular_velocity_std_dev_count: usize,

// linear_acceleration_std_devs: UnorderedQueue<N>,
// angular_velocity_std_devs: UnorderedQueue<N>,

// /// The number of points to use in the particle filter.
// pub point_count: NonZeroUsize,

// pub start_std_dev: N,

// /// The minimum unnormalized weight sum that all particles can have
// /// before undeprivation must be performed.
// ///
// /// Usually, if the unnormalized weight sum is small, it means that the robot
// /// is very confident but received a contratictory observation. For example, the
// /// robot may be confident that it is in position A, so when it receives an observation
// /// that suggests that it is at position B, it will not be able to update its estimate
// /// effectively.
// pub minimum_unnormalized_weight: N,
// /// The fraction of particles that will be resampled to be around the new observation.
// ///
// /// This helps the robot to combat the issue of particle deprivation.
// pub undeprivation_factor: N,