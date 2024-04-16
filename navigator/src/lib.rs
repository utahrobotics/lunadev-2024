use std::{ops::Deref, sync::Arc, time::Duration};

use drive::Steering;
use nalgebra::{Point3, UnitVector2, Vector2};
use ordered_float::NotNan;
use rig::{RigSpace, RobotBaseRef};
use unros::{
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, Publisher, PublisherRef, Subscriber, WatchSubscriber},
    runtime::RuntimeContext,
    setup_logging,
    tokio::task::block_in_place,
    DontDrop, ShouldNotDrop,
};

pub mod drive;
pub mod pathfinding;
// mod utils;
// pub mod pathfinders;

type Float = f32;

const PI: Float = std::f64::consts::PI as Float;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    ForwardOnly,
    Both,
    ReverseOnly,
}

#[derive(ShouldNotDrop)]
pub struct DifferentialDriver<F: FnMut(Float) -> Float + Send + 'static> {
    path_sub: Subscriber<Arc<[Point3<Float>]>>,
    steering_signal: Publisher<Steering>,
    robot_base: RobotBaseRef,
    pub refresh_rate: Duration,
    drive_mode: WatchSubscriber<DriveMode>,
    pub full_turn_angle: Float,
    pub turn_fn: F,
    dont_drop: DontDrop<Self>,
}

impl DifferentialDriver<fn(Float) -> Float> {
    pub fn new(robot_base: RobotBaseRef) -> Self {
        Self {
            path_sub: Subscriber::new(1),
            steering_signal: Default::default(),
            robot_base,
            refresh_rate: Duration::from_millis(20),
            drive_mode: WatchSubscriber::new(DriveMode::Both),
            // 30 degrees
            full_turn_angle: std::f64::consts::FRAC_PI_6 as Float,
            turn_fn: |frac| -2.0 * frac + 1.0,
            dont_drop: DontDrop::new("diff-driver"),
        }
    }
}

impl<F: FnMut(Float) -> Float + Send + 'static> DifferentialDriver<F> {
    pub fn steering_pub(&self) -> PublisherRef<Steering> {
        self.steering_signal.get_ref()
    }

    pub fn create_path_sub(&self) -> DirectSubscription<Arc<[Point3<Float>]>> {
        self.path_sub.create_subscription()
    }

    pub fn create_drive_mode_sub(&self) -> DirectSubscription<DriveMode> {
        self.drive_mode.create_subscription()
    }
}

impl<F> AsyncNode for DifferentialDriver<F>
where
    F: FnMut(Float) -> Float + Send + 'static,
{
    type Result = ();

    async fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;
        let sleeper = spin_sleep::SpinSleeper::default();

        loop {
            let mut path = self.path_sub.recv().await;

            block_in_place(|| {
                loop {
                    if context.is_runtime_exiting() {
                        break;
                    }
                    if let Some(new_path) = self.path_sub.try_recv() {
                        path = new_path;
                    }

                    if path.is_empty() {
                        break;
                    }
                    WatchSubscriber::try_update(&mut self.drive_mode);

                    let isometry = self.robot_base.get_isometry();
                    let position = Vector2::new(isometry.translation.x, isometry.translation.z);

                    let (i, _) = path
                        .iter()
                        .map(|v| Vector2::new(v.x, v.z))
                        .enumerate()
                        .map(|(i, v)| (i, (v - position).magnitude_squared()))
                        .min_by_key(|(_, distance)| NotNan::new(*distance).unwrap())
                        .unwrap();

                    if i == path.len() - 1 {
                        break;
                    }

                    let next = path[i + 1];
                    let next = Vector2::new(next.x, next.z);

                    let forward = isometry.get_forward_vector();
                    let forward = UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

                    let travel = (next - position).normalize();
                    let cross = (forward.x * travel.y - forward.y * travel.x).signum();
                    assert!(!travel.x.is_nan(), "{position:?} {next:?} {path:?}");
                    assert!(!travel.y.is_nan(), "{position:?} {next:?} {path:?}");
                    let mut angle = forward.angle(&travel);
                    let mut reversing = 1.0;

                    if self.drive_mode.deref() == &DriveMode::ReverseOnly
                        || (angle > PI / 2.0 && self.drive_mode.deref() != &DriveMode::ForwardOnly)
                    {
                        angle = PI - angle;
                        reversing = -1.0;
                    }

                    if angle > self.full_turn_angle {
                        if cross > 0.0 {
                            self.steering_signal
                                .set(Steering::new(1.0 * reversing, -1.0 * reversing));
                        } else {
                            self.steering_signal
                                .set(Steering::new(-1.0 * reversing, 1.0 * reversing));
                        }
                    } else {
                        let smaller_ratio = (self.turn_fn)(angle / self.full_turn_angle);

                        if cross > 0.0 {
                            self.steering_signal
                                .set(Steering::new(1.0 * reversing, smaller_ratio * reversing));
                        } else {
                            self.steering_signal
                                .set(Steering::new(smaller_ratio * reversing, 1.0 * reversing));
                        }
                    }

                    sleeper.sleep(self.refresh_rate);
                }
                self.steering_signal.set(Steering::new(0.0, 0.0));
            });

            // if let Some(goal_forward) = goal_forward {
            //     // Turn to goal orientation
            //     loop {
            //         let forward = robot_base.get_isometry().get_forward_vector();
            //         let forward =
            //             UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

            //         let arc_angle = (forward.x * goal_forward.y - forward.y * goal_forward.x)
            //             .signum()
            //             * forward.angle(&goal_forward);

            //         if arc_angle.abs() < self.angle_epsilon {
            //             break;
            //         }

            //         if arc_angle > 0.0 {
            //             self.steering_signal.set(Steering::new(-1.0, 1.0));
            //         } else {
            //             self.steering_signal.set(Steering::new(1.0, -1.0));
            //         }

            //         sleeper.sleep(self.refresh_rate);
            //     }
            //     self.steering_signal.set(Steering::new(0.0, 0.0));
            // }
        }
    }
}
