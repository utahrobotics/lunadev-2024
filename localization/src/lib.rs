//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::time::Duration;

use calib::calibrate_localizer;
use engines::LocalizerEngine;
use frames::{IMUFrame, OrientationFrame, PositionFrame, VelocityFrame};
use fxhash::FxHashMap;
use nalgebra::{UnitQuaternion, Vector3};
use rig::{RobotBase, RobotElementRef};
use run::run_localizer;
use smach::State;
use unros::{
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, Subscriber},
    runtime::RuntimeContext,
    DontDrop, ShouldNotDrop,
};
use utils::random_unit_vector;

mod calib;
pub mod engines;
pub mod frames;
mod run;
mod utils;

pub use utils::{gravity, Float};

/// A Node that can digest multiple streams of spatial input to
/// determine where an object is in global space.
///
/// Processing does not occur until the node is running.
#[derive(ShouldNotDrop)]
pub struct Localizer<N: Float, E: LocalizerEngine<N>> {
    bb: LocalizerBlackboard<N, E>,
    dont_drop: DontDrop<Self>,
}

impl<N: Float, E: LocalizerEngine<N>> Localizer<N, E> {
    pub fn new(robot_base: RobotBase, engine_config: E::Config) -> Self {
        Self {
            bb: LocalizerBlackboard {
                calibration_duration: Duration::from_secs(3),
                recalibrate_sub: Subscriber::new(1),
                imu_sub: Subscriber::new(1),
                position_sub: Subscriber::new(1),
                orientation_sub: Subscriber::new(1),
                velocity_sub: Subscriber::new(1),
                robot_base,
                max_delta: Duration::from_millis(50),
                calibrations: Default::default(),
                context: None,
                start_orientation: Default::default(),
                engine_config,
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

pub struct LocalizerBlackboard<N: Float, E: LocalizerEngine<N>> {
    /// The maximum time between observations.
    ///
    /// Exceeding this time causes the filter to update its estimates without
    /// incorporating new data.
    pub max_delta: Duration,

    /// The duration to calibrate the localizer for.
    ///
    /// During calibration, the localizer will collect data from the IMU to determine
    /// how much to correct each IMU by, and what the current orientation of the robot is
    /// based on the prevailing direction for gravity
    pub calibration_duration: Duration,
    start_orientation: UnitQuaternion<N>,

    recalibrate_sub: Subscriber<()>,
    calibrations: FxHashMap<RobotElementRef, CalibratedImu<N>>,

    imu_sub: Subscriber<IMUFrame<N>>,
    position_sub: Subscriber<PositionFrame<N>>,
    velocity_sub: Subscriber<VelocityFrame<N>>,
    orientation_sub: Subscriber<OrientationFrame<N>>,

    pub engine_config: E::Config,

    robot_base: RobotBase,

    context: Option<RuntimeContext>,
}

impl<N: Float, E: LocalizerEngine<N>> AsyncNode for Localizer<N, E> {
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

impl<N: Float, E: LocalizerEngine<N>> std::ops::Deref for Localizer<N, E> {
    type Target = LocalizerBlackboard<N, E>;

    fn deref(&self) -> &Self::Target {
        &self.bb
    }
}

impl<N: Float, E: LocalizerEngine<N>> std::ops::DerefMut for Localizer<N, E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.bb
    }
}
