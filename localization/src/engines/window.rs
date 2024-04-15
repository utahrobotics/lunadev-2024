use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

use nalgebra::{convert as nconvert, Isometry3, Point3, UnitQuaternion, Vector3};
use rig::RobotBaseRef;

use crate::{gravity, utils::quat_mean, Float};
use unros::log;

use super::LocalizerEngine;

struct Observation<T, N> {
    observation: T,
    variance: N,
    delta: Duration,
}

pub struct WindowLocalizer<N: Float, FI, FV, FA, FAV> {
    isometry: Isometry3<N>,
    linear_velocity: Vector3<N>,
    linear_acceleration: Vector3<N>,
    angular_velocity: UnitQuaternion<N>,

    bucket_duration: Duration,

    linear_acceleration_queue: VecDeque<Observation<Vector3<N>, N>>,
    last_linear_acceleration: Instant,

    linear_velocity_queue: VecDeque<Observation<Vector3<N>, N>>,
    last_linear_velocity: Instant,

    position_queue: VecDeque<Observation<Vector3<N>, N>>,
    last_position: Instant,

    angular_velocity_queue: VecDeque<Observation<UnitQuaternion<N>, N>>,
    last_angular_velocity: Instant,

    orientation_queue: VecDeque<Observation<UnitQuaternion<N>, N>>,
    last_orientation: Instant,

    additional_time_factor: N,
    max_no_observation_duration: Duration,

    isometry_func: FI,
    linear_velocity_func: FV,
    linear_acceleration_func: FA,
    angular_velocity_func: FAV,
}

impl<N: Float, FI, FV, FA, FAV> WindowLocalizer<N, FI, FV, FA, FAV>
where
    FI: FnMut(&mut Isometry3<N>),
    FV: FnMut(&mut Vector3<N>),
    FA: FnMut(&mut Vector3<N>),
    FAV: FnMut(&mut UnitQuaternion<N>),
{
    fn total_variance<T>(queue: &VecDeque<Observation<T, N>>) -> N {
        queue.iter().map(|obs| obs.variance).sum()
    }

    fn total_time<T>(queue: &VecDeque<Observation<T, N>>) -> N {
        queue
            .iter()
            .map(|obs| nconvert::<_, N>(obs.delta.as_secs_f32()))
            .sum()
    }

    fn iter_with_factor<'a, T>(
        &'a self,
        queue: &'a VecDeque<Observation<T, N>>,
    ) -> impl Iterator<Item = (&Observation<T, N>, N)> {
        let total_time = Self::total_time(queue);
        let two = N::one() + N::one();
        let time_height = two / total_time;
        let mut elapsed_time = N::zero();

        let total_variance = Self::total_variance(queue);
        let additional_height = self.additional_time_factor / total_time;

        queue.iter().map(move |obs| {
            let delta: N = nconvert(obs.delta.as_secs_f32());
            let start_time =
                -time_height / total_time * elapsed_time + time_height + additional_height;
            let end_time = -time_height / total_time * (elapsed_time + delta)
                + time_height
                + additional_height;
            elapsed_time += delta;
            let time_factor =
                (end_time + start_time) / two * delta / (N::one() + self.additional_time_factor);

            let variance_factor = obs.variance / total_variance;

            (obs, (time_factor + variance_factor) / two)
        })
    }

    fn process(&mut self) {
        if let Some(result) = self
            .iter_with_factor(&self.linear_acceleration_queue)
            .map(|(obs, factor)| obs.observation * factor)
            .reduce(|a, b| a + b)
        {
            self.linear_acceleration = result;
        }
        if let Some(result) = self
            .iter_with_factor(&self.linear_velocity_queue)
            .map(|(obs, factor)| obs.observation * factor)
            .reduce(|a, b| a + b)
        {
            self.linear_velocity = result;
        }
        if let Some(result) = self
            .iter_with_factor(&self.position_queue)
            .map(|(obs, factor)| obs.observation * factor)
            .reduce(|a, b| a + b)
        {
            self.isometry.translation = result.into();
        }
        if let Some(result) = quat_mean(
            self.iter_with_factor(&self.angular_velocity_queue)
                .map(|(obs, factor)| (obs.observation, factor)),
        ) {
            match result {
                Ok(result) => self.angular_velocity = result,
                Err(e) => {
                    log::error!("{e}");
                }
            }
        }
        if let Some(result) = quat_mean(
            self.iter_with_factor(&self.orientation_queue)
                .map(|(obs, factor)| (obs.observation, factor)),
        ) {
            match result {
                Ok(result) => self.isometry.rotation = result,
                Err(e) => {
                    log::error!("{e}");
                }
            }
        }

        (self.isometry_func)(&mut self.isometry);
        (self.linear_velocity_func)(&mut self.linear_velocity);
        (self.linear_acceleration_func)(&mut self.linear_acceleration);
        (self.angular_velocity_func)(&mut self.angular_velocity);

        if !self.linear_acceleration_queue.is_empty() {
            let delta = self.last_linear_velocity.elapsed();

            if delta >= self.max_no_observation_duration {
                let delta: N = nconvert(delta.as_secs_f32());
                Self::add_observation(
                    self.linear_velocity + (self.linear_acceleration - gravity()) * delta,
                    Self::total_variance(&self.linear_acceleration_queue)
                        / nconvert(self.linear_acceleration_queue.len()),
                    &mut self.linear_velocity_queue,
                    &mut self.last_linear_velocity,
                    self.bucket_duration,
                );
            }
        }
        if !self.linear_velocity_queue.is_empty() {
            let delta = self.last_position.elapsed();

            if delta >= self.max_no_observation_duration {
                let delta: N = nconvert(delta.as_secs_f32());
                Self::add_observation(
                    self.isometry.translation.vector + self.linear_velocity * delta,
                    Self::total_variance(&self.linear_velocity_queue)
                        / nconvert(self.linear_velocity_queue.len()),
                    &mut self.position_queue,
                    &mut self.last_position,
                    self.bucket_duration,
                );
            }
        }
        if !self.angular_velocity_queue.is_empty() {
            let delta = self.last_orientation.elapsed();

            if delta >= self.max_no_observation_duration {
                let delta: N = nconvert(delta.as_secs_f32());
                Self::add_observation(
                    UnitQuaternion::default().slerp(&self.angular_velocity, delta)
                        * self.isometry.rotation,
                    Self::total_variance(&self.angular_velocity_queue)
                        / nconvert(self.angular_velocity_queue.len()),
                    &mut self.orientation_queue,
                    &mut self.last_orientation,
                    self.bucket_duration,
                );
            }
        }
    }

    fn add_observation<T>(
        observation: T,
        variance: N,
        queue: &mut VecDeque<Observation<T, N>>,
        instant: &mut Instant,
        bucket_duration: Duration,
    ) {
        let delta = instant.elapsed();
        *instant += delta;
        let mut total_time = Self::total_time(queue) + nconvert(bucket_duration.as_secs_f32());

        while total_time > nconvert(bucket_duration.as_secs_f32()) {
            let Some(obs) = queue.pop_back() else {
                break;
            };
            total_time -= nconvert(obs.delta.as_secs_f32());
        }
        queue.push_front(Observation {
            observation,
            variance,
            delta,
        });
    }
}

pub type DefaultWindowConfig<N> = WindowConfig<
    N,
    fn(&mut Isometry3<N>),
    fn(&mut Vector3<N>),
    fn(&mut Vector3<N>),
    fn(&mut UnitQuaternion<N>),
>;

pub struct WindowConfig<N: Float, FI, FV, FA, FAV> {
    pub bucket_duration: Duration,
    pub isometry_func: FI,
    pub linear_velocity_func: FV,
    pub linear_acceleration_func: FA,
    pub angular_velocity_func: FAV,
    pub max_no_observation_duration: Duration,
    pub additional_time_factor: N,
}

impl<N: Float> Default for DefaultWindowConfig<N> {
    fn default() -> Self {
        Self {
            bucket_duration: Duration::from_secs(1),
            max_no_observation_duration: Duration::from_millis(100),
            isometry_func: |_| {},
            linear_velocity_func: |_| {},
            linear_acceleration_func: |_| {},
            angular_velocity_func: |_| {},
            additional_time_factor: N::zero(),
        }
    }
}

impl<N: Float, FI, FV, FA, FAV> LocalizerEngine<N> for WindowLocalizer<N, FI, FV, FA, FAV>
where
    FI: FnMut(&mut Isometry3<N>) + Send + 'static + Clone,
    FV: FnMut(&mut Vector3<N>) + Send + 'static + Clone,
    FA: FnMut(&mut Vector3<N>) + Send + 'static + Clone,
    FAV: FnMut(&mut UnitQuaternion<N>) + Send + 'static + Clone,
{
    type Config = WindowConfig<N, FI, FV, FA, FAV>;

    fn from_config(config: &Self::Config, robot_base: RobotBaseRef) -> Self {
        let now = Instant::now();
        Self {
            isometry: nconvert(robot_base.get_isometry()),
            linear_velocity: nconvert(robot_base.get_linear_velocity()),
            linear_acceleration: gravity(),
            angular_velocity: UnitQuaternion::identity(),

            bucket_duration: config.bucket_duration,

            linear_acceleration_queue: VecDeque::new(),
            last_linear_acceleration: now,

            linear_velocity_queue: VecDeque::new(),
            last_linear_velocity: now,

            position_queue: VecDeque::new(),
            last_position: now,

            angular_velocity_queue: VecDeque::new(),
            last_angular_velocity: now,

            orientation_queue: VecDeque::new(),
            last_orientation: now,

            additional_time_factor: config.additional_time_factor,

            max_no_observation_duration: config.max_no_observation_duration,

            isometry_func: config.isometry_func.clone(),
            linear_velocity_func: config.linear_velocity_func.clone(),
            linear_acceleration_func: config.linear_acceleration_func.clone(),
            angular_velocity_func: config.angular_velocity_func.clone(),
        }
    }

    fn observe_linear_acceleration(&mut self, accel: Vector3<N>, variance: N) {
        Self::add_observation(
            accel,
            variance,
            &mut self.linear_acceleration_queue,
            &mut self.last_linear_acceleration,
            self.bucket_duration,
        );
        self.process();
    }

    fn observe_linear_velocity(&mut self, vel: Vector3<N>, variance: N) {
        Self::add_observation(
            vel,
            variance,
            &mut self.linear_velocity_queue,
            &mut self.last_linear_velocity,
            self.bucket_duration,
        );
        self.process();
    }

    fn observe_position(&mut self, pos: Point3<N>, variance: N) {
        Self::add_observation(
            pos.coords,
            variance,
            &mut self.position_queue,
            &mut self.last_position,
            self.bucket_duration,
        );
        self.process();
    }

    fn observe_angular_velocity(&mut self, ang_vel: UnitQuaternion<N>, variance: N) {
        Self::add_observation(
            ang_vel,
            variance,
            &mut self.angular_velocity_queue,
            &mut self.last_angular_velocity,
            self.bucket_duration,
        );
        self.process();
    }

    fn observe_orientation(&mut self, orient: UnitQuaternion<N>, variance: N) {
        Self::add_observation(
            orient,
            variance,
            &mut self.orientation_queue,
            &mut self.last_orientation,
            self.bucket_duration,
        );
        self.process();
    }

    fn no_observation(&mut self) {
        self.process();
    }

    fn get_isometry(&self) -> Isometry3<N> {
        self.isometry
    }

    fn get_linear_velocity(&self) -> Vector3<N> {
        self.linear_velocity
    }

    fn get_linear_acceleration(&self) -> Vector3<N> {
        self.linear_acceleration
    }

    fn get_angular_velocity(&self) -> UnitQuaternion<N> {
        self.angular_velocity
    }
}
