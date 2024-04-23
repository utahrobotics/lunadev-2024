use std::{ops::Deref, sync::Arc, time::Duration};

use drive::Steering;
use nalgebra::{Point3, UnitVector2, Vector2, Vector3};
use ordered_float::FloatCore;
use rig::{RigSpace, RobotBaseRef};
use unros::{
    float::Float,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    ForwardOnly,
    Both,
    ReverseOnly,
}

#[derive(ShouldNotDrop)]
pub struct DifferentialDriver<N: Float + FloatCore, F: FnMut(N) -> N + Send + 'static> {
    path_sub: Subscriber<Arc<[Point3<N>]>>,
    steering_signal: Publisher<Steering<N>>,
    robot_base: RobotBaseRef,
    pub refresh_rate: Duration,
    drive_mode: WatchSubscriber<DriveMode>,
    pub full_turn_angle: N,
    pub completion_distance: N,
    pub turn_fn: F,
    dont_drop: DontDrop<Self>,
}

impl<N: Float + FloatCore> DifferentialDriver<N, fn(N) -> N> {
    pub fn new(robot_base: RobotBaseRef) -> Self {
        Self {
            path_sub: Subscriber::new(1),
            steering_signal: Default::default(),
            robot_base,
            refresh_rate: Duration::from_millis(20),
            drive_mode: WatchSubscriber::new(DriveMode::Both),
            completion_distance: nalgebra::convert(0.15),
            // 30 degrees
            full_turn_angle: N::frac_pi_6(),
            turn_fn: |frac| nalgebra::convert::<_, N>(-2.0) * frac + N::one(),
            dont_drop: DontDrop::new("diff-driver"),
        }
    }
}

impl<N: Float + FloatCore, F: FnMut(N) -> N + Send + 'static> DifferentialDriver<N, F> {
    pub fn steering_pub(&self) -> PublisherRef<Steering<N>> {
        self.steering_signal.get_ref()
    }

    pub fn create_path_sub(&self) -> DirectSubscription<Arc<[Point3<N>]>> {
        self.path_sub.create_subscription()
    }

    pub fn create_drive_mode_sub(&self) -> DirectSubscription<DriveMode> {
        self.drive_mode.create_subscription()
    }
}

impl<N: Float + FloatCore, F> AsyncNode for DifferentialDriver<N, F>
where
    F: FnMut(N) -> N + Send + 'static,
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
                    let position: Vector2<N> = nalgebra::convert(Vector2::new(
                        isometry.translation.x,
                        isometry.translation.z,
                    ));

                    let (i, _) = path
                        .iter()
                        .map(|v| Vector2::new(v.x, v.z))
                        .enumerate()
                        .map(|(i, v)| (i, (v - position).magnitude_squared()))
                        .min_by_key(|(_, distance)| distance.to_not_nan().unwrap())
                        .unwrap();

                    let next = if i == path.len() - 1 {
                        path[i]
                    } else {
                        path[i + 1]
                    };

                    let next = Vector2::new(next.x, next.z);

                    let forward: Vector3<N> = nalgebra::convert(isometry.get_forward_vector());
                    let forward = UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

                    let mut travel = next - position;
                    let distance = travel.magnitude();

                    if i == path.len() - 1 && distance < self.completion_distance {
                        break;
                    }

                    travel.unscale_mut(distance);
                    let cross = FloatCore::signum(forward.x * travel.y - forward.y * travel.x);
                    assert!(!Float::is_nan(travel.x), "{position:?} {next:?} {path:?}");
                    assert!(!Float::is_nan(travel.y), "{position:?} {next:?} {path:?}");
                    let mut angle = forward.angle(&travel);
                    let mut reversing = N::one();

                    if self.drive_mode.deref() == &DriveMode::ReverseOnly
                        || (angle > N::pi() / nalgebra::convert(2.0)
                            && self.drive_mode.deref() != &DriveMode::ForwardOnly)
                    {
                        angle = N::pi() - angle;
                        reversing = -N::one();
                    }

                    if angle > self.full_turn_angle {
                        if cross > N::zero() {
                            self.steering_signal
                                .set(Steering::new(N::one() * reversing, -N::one() * reversing));
                        } else {
                            self.steering_signal
                                .set(Steering::new(-N::one() * reversing, N::one() * reversing));
                        }
                    } else {
                        let smaller_ratio = (self.turn_fn)(angle / self.full_turn_angle);

                        if cross > N::zero() {
                            self.steering_signal.set(Steering::new(
                                N::one() * reversing,
                                smaller_ratio * reversing,
                            ));
                        } else {
                            self.steering_signal.set(Steering::new(
                                smaller_ratio * reversing,
                                N::one() * reversing,
                            ));
                        }
                    }

                    sleeper.sleep(self.refresh_rate);
                }
                self.steering_signal
                    .set(Steering::new(N::zero(), N::zero()));
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
