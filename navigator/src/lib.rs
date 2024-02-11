use std::time::Duration;

use global_msgs::Steering;
use nalgebra::{Point2, UnitVector2, Vector2};
use ordered_float::NotNan;
use rig::{RigSpace, RobotBaseRef};
use unros_core::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscriber, Subscription},
    setup_logging, tokio_rayon, Node, RuntimeContext,
};

pub mod pathfinders;

type Float = f32;

const PI: Float = std::f64::consts::PI as Float;

pub struct DifferentialDriver<F> {
    path_sub: Subscriber<Vec<Point2<Float>>>,
    steering_signal: Publisher<Steering>,
    robot_base: RobotBaseRef,
    pub refresh_rate: Duration,
    pub can_reverse: bool,
    pub full_turn_angle: Float,
    pub turn_fn: F,
}

impl DifferentialDriver<fn(Float) -> Float> {
    pub fn new(robot_base: RobotBaseRef) -> Self {
        Self {
            path_sub: Default::default(),
            steering_signal: Default::default(),
            robot_base,
            refresh_rate: Duration::from_millis(20),
            can_reverse: false,
            // 30 degrees
            full_turn_angle: 0.5235987756,
            turn_fn: |frac| -2.0 * frac + 1.0,
        }
    }

    pub fn accept_steering_sub(&mut self, sub: Subscription<Steering>) {
        self.steering_signal.accept_subscription(sub);
    }

    pub fn create_path_sub(&mut self) -> Subscription<Vec<Point2<Float>>> {
        self.path_sub.create_subscription(1)
    }
}

#[async_trait]
impl<F> Node for DifferentialDriver<F>
where
    F: FnMut(Float) -> Float + Send + 'static,
{
    const DEFAULT_NAME: &'static str = "waypoint-driver";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        let sleeper = spin_sleep::SpinSleeper::default();

        loop {
            let mut path = self.path_sub.recv().await;

            let (path_sub, steering_signal, robot_base, turn_fn) = tokio_rayon::spawn(move || {
                loop {
                    if let Some(new_path) = self.path_sub.try_recv() {
                        path = new_path;
                    }

                    if path.is_empty() {
                        break;
                    }

                    let isometry = self.robot_base.get_isometry();
                    let position = Vector2::new(isometry.translation.x, isometry.translation.z);

                    let (i, _) = path
                        .iter()
                        .enumerate()
                        .map(|(i, x)| (i, (x.coords - position).magnitude_squared()))
                        .min_by_key(|(_, distance)| NotNan::new(*distance).unwrap())
                        .unwrap();

                    if i == path.len() - 1 {
                        break;
                    }

                    let next = path[i + 1];

                    let forward = isometry.get_forward_vector();
                    let forward = UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

                    let travel = (next.cast::<Float>() - position.cast()).coords.normalize();
                    let cross = (forward.x * travel.y - forward.y * travel.x).signum();
                    let mut angle = forward.angle(&travel);
                    let mut reversing = 1.0;

                    if angle > PI / 2.0 && self.can_reverse {
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

                (
                    self.path_sub,
                    self.steering_signal,
                    self.robot_base,
                    self.turn_fn,
                )
            })
            .await;

            self.path_sub = path_sub;
            self.steering_signal = steering_signal;
            self.robot_base = robot_base;
            self.turn_fn = turn_fn;

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
