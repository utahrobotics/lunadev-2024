use std::{ops::Deref, sync::Arc, time::Duration};

use nalgebra::{convert as nconvert, Isometry3, Point3, UnitVector2, Vector2, Vector3};
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

use super::{DriveMode, Steering};

#[derive(ShouldNotDrop)]
pub struct DifferentialDriver<N: Float + FloatCore, F: FnMut(N) -> N + Send + 'static> {
    path_sub: Subscriber<Arc<[Point3<N>]>>,
    steering_signal: Publisher<Steering<N>>,
    robot_base: RobotBaseRef,
    pub refresh_rate: Duration,
    drive_mode: WatchSubscriber<DriveMode>,
    pub full_turn_angle: N,
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

                    let isometry: Isometry3<N> = nconvert(self.robot_base.get_isometry());

                    let (i, _) = path
                        .windows(2)
                        .enumerate()
                        .filter_map(|(i, window)| {
                            let [from, to] = window.try_into().unwrap();
                            let to = to.coords;
                            let from = from.coords;
                            let mut segment = to - from;
                            let relative = to - isometry.translation.vector;
                            let segment_length = segment.magnitude();
                            segment.unscale_mut(segment_length);
                            let distance_along = segment.dot(&relative);
                            if distance_along < N::zero() || distance_along > segment_length {
                                return None;
                            }
                            let point_along = segment * distance_along;
                            let offset_sq = (point_along - relative).magnitude_squared();

                            Some((i + 1, offset_sq))
                        })
                        .chain({
                            let distance_sq =
                                (isometry.translation.vector - path[0].coords).magnitude_squared();
                            if distance_sq < nconvert(0.0001) {
                                None
                            } else {
                                Some((0, distance_sq))
                            }
                        })
                        .chain(std::iter::once((
                            path.len() - 1,
                            (isometry.translation.vector - path.last().unwrap().coords)
                                .magnitude_squared(),
                        )))
                        .min_by(|(_, a), (_, b)| a.total_cmp(b))
                        .unwrap();

                    let next = path[i];
                    let next = Vector2::new(next.x, next.z);
                    let position = Vector2::new(isometry.translation.x, isometry.translation.z);

                    let mut travel = next - position;
                    let distance = travel.magnitude();
                    travel.unscale_mut(distance);

                    let forward: Vector3<N> = isometry.get_forward_vector();
                    let forward = UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

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
