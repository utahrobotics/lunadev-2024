//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::{num::NonZeroUsize, time::Duration};

use calib::calibrate_localizer;
use frames::{IMUFrame, OrientationFrame, PositionFrame, VelocityFrame};
use fxhash::FxHashMap;
use nalgebra::{convert as nconvert, UnitQuaternion, Vector3};
use rig::{RobotBase, RobotElementRef};
use run::run_localizer;
use smach::State;
use unros::{
    node::AsyncNode, pubsub::{subs::DirectSubscription, Subscriber}, runtime::RuntimeContext, DontDrop, ShouldNotDrop
};
use utils::{random_unit_vector, UnorderedQueue};

mod calib;
pub mod frames;
mod run;
mod utils;

pub use run::OBSERVATIONS;
pub use utils::{gravity, Float};

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
#[derive(ShouldNotDrop)]
pub struct Localizer<N: Float> {
    bb: LocalizerBlackboard<N>,
    dont_drop: DontDrop<Self>,
}

impl<N: Float> Localizer<N> {
    pub fn new(robot_base: RobotBase, start_variance: N) -> Self {
        Self {
            bb: LocalizerBlackboard {
                point_count: NonZeroUsize::new(500).unwrap(),
                start_std_dev: start_variance.sqrt(),
                calibration_duration: Duration::from_secs(3),
                recalibrate_sub: Subscriber::new(1),
                minimum_unnormalized_weight: nconvert(0.4),
                undeprivation_factor: nconvert(0.05),
                likelihood_table: LikelihoodTable::default(),
                imu_sub: Subscriber::new(1),
                position_sub: Subscriber::new(1),
                orientation_sub: Subscriber::new(1),
                velocity_sub: Subscriber::new(1),
                robot_base,
                max_delta: Duration::from_millis(50),
                linear_acceleration_std_dev_count: 10,
                angular_velocity_std_dev_count: 10,
                linear_acceleration_std_devs: std::iter::repeat(N::zero()).take(10).collect(),
                angular_velocity_std_devs: std::iter::repeat(N::zero()).take(10).collect(),
                calibrations: Default::default(),
                context: None,
                start_orientation: UnitQuaternion::default(),
            },
            dont_drop: DontDrop::new("localizer"),
        }
    }

    /// Provide an imu subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_imu_sub(&self) -> DirectSubscription<IMUFrame<N>> {
        self.bb.imu_sub.create_subscription()
    }

    /// Provide a position subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_position_sub(&self) -> DirectSubscription<PositionFrame<N>> {
        self.bb.position_sub.create_subscription()
    }

    /// Provide a velocity subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_velocity_sub(&self) -> DirectSubscription<VelocityFrame<N>> {
        self.bb.velocity_sub.create_subscription()
    }

    /// Provide an orientation subscription.
    ///
    /// Some messages may be skipped if there are too many.
    pub fn create_orientation_sub(&self) -> DirectSubscription<OrientationFrame<N>> {
        self.bb.orientation_sub.create_subscription()
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

/// A collection of functions that can be used to incorporate the likelihoods of more nuanced scenarios.
///
/// For example, you may know the top speed of your robot, so you can assign all velocities
/// above that speed a likelihood of zero. Do not that a likelihood of 1 *does not* mean 100% possible,
/// whereas a likelihood of 0 *does* mean impossible. The effect of non-zero likelihoods depends on
/// how the likelihood function works against all values in the filter.
///
/// If there are two particles in the filter, and a likelihood function returns 1000 for the velocity
/// of both particles, it means that both particles are equally likely to be true. This would be true
/// even if the likelihood was 1 for both, or even 0.00001 for both. In other words, the ratio between
/// likelihoods matters more than the magnitude.
pub struct LikelihoodTable<N: Float> {
    pub position: Box<dyn Fn(&mut Vector3<N>) -> N + Send + Sync>,
    pub linear_velocity: Box<dyn Fn(&mut Vector3<N>) -> N + Send + Sync>,
    pub linear_acceleration: Box<dyn Fn(&mut Vector3<N>) -> N + Send + Sync>,

    pub orientation: Box<dyn Fn(&mut UnitQuaternion<N>) -> N + Send + Sync>,
    pub angular_velocity: Box<dyn Fn(&mut UnitQuaternion<N>) -> N + Send + Sync>,
}

impl<N: Float> Default for LikelihoodTable<N> {
    fn default() -> Self {
        Self {
            position: Box::new(|_| N::one()),
            linear_velocity: Box::new(|_| N::one()),
            linear_acceleration: Box::new(|_| N::one()),
            orientation: Box::new(|_| N::one()),
            angular_velocity: Box::new(|_| N::one()),
        }
    }
}

pub struct LocalizerBlackboard<N: Float> {
    /// The number of points to use in the particle filter.
    pub point_count: NonZeroUsize,

    pub start_std_dev: N,
    /// The maximum time between observations.
    ///
    /// Exceeding this time causes the filter to update its estimates without
    /// incorporating new data.
    pub max_delta: Duration,

    /// The minimum unnormalized weight sum that all particles can have
    /// before undeprivation must be performed.
    ///
    /// Usually, if the unnormalized weight sum is small, it means that the robot
    /// is very confident but received a contratictory observation. For example, the
    /// robot may be confident that it is in position A, so when it receives an observation
    /// that suggests that it is at position B, it will not be able to update its estimate
    /// effectively.
    pub minimum_unnormalized_weight: N,
    /// The fraction of particles that will be resampled to be around the new observation.
    ///
    /// This helps the robot to combat the issue of particle deprivation.
    pub undeprivation_factor: N,

    /// The number of standard deviations to keep track of for linear acceleration.
    ///
    /// The estimate for linear acceleration worsens by the mean of the standard deviations.
    pub linear_acceleration_std_dev_count: usize,
    /// The number of standard deviations to keep track of for angular velocity.
    ///
    /// The estimate for angular velocity worsens by the mean of the standard deviations.
    pub angular_velocity_std_dev_count: usize,

    linear_acceleration_std_devs: UnorderedQueue<N>,
    angular_velocity_std_devs: UnorderedQueue<N>,

    /// The duration to calibrate the localizer for.
    ///
    /// During calibration, the localizer will collect data from the IMU to determine
    /// how much to correct each IMU by, and what the current orientation of the robot is
    /// based on the prevailing direction for gravity
    pub calibration_duration: Duration,
    /// A collection of functions that can be used to incorporate the likelihoods of
    /// more nuanced scenarios.
    ///
    /// For example, you may know the top speed of your robot, so you can assign all velocities
    /// above that speed a likelihood of zero.
    pub likelihood_table: LikelihoodTable<N>,

    recalibrate_sub: Subscriber<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu<N>>,

    imu_sub: Subscriber<IMUFrame<N>>,
    position_sub: Subscriber<PositionFrame<N>>,
    velocity_sub: Subscriber<VelocityFrame<N>>,
    orientation_sub: Subscriber<OrientationFrame<N>>,

    start_orientation: UnitQuaternion<N>,

    robot_base: RobotBase,

    context: Option<RuntimeContext>,
}

impl<N: Float> AsyncNode for Localizer<N> {
    type Result = ();

    async fn run(mut self, context: RuntimeContext) -> Self::Result {
        self.dont_drop.ignore_drop = true;
        self.bb.context = Some(context);

        let (calib, calib_trans) = State::new(calibrate_localizer);
        let (run, run_trans) = State::new(run_localizer);

        let start_state = calib.clone();

        calib_trans.set_transition(move |_| Some(run.clone()));
        run_trans.set_transition(move |_| Some(calib.clone()));

        start_state.start(self.bb).await;
        unreachable!()
    }
}

impl<N: Float> std::ops::Deref for Localizer<N> {
    type Target = LocalizerBlackboard<N>;

    fn deref(&self) -> &Self::Target {
        &self.bb
    }
}

impl<N: Float> std::ops::DerefMut for Localizer<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.bb
    }
}
