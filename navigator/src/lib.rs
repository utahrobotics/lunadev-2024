use std::{
    fmt::{Debug, Display},
    sync::mpsc,
    time::Duration,
};

use costmap::CostmapRef;
use global_msgs::Steering;
use nalgebra::{Point2, UnitQuaternion, UnitVector2, Vector2, Vector3};
use ordered_float::NotNan;
use pathfinding::directed::astar::astar;
use rig::{RigSpace, RobotBaseRef};
use successors::{successors, RobotState};
use unros_core::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscription},
    task::{Task, TaskHandle},
    tokio::sync::oneshot,
    tokio_rayon, Node, RuntimeContext,
};

type Float = f32;

struct DrivingTaskInit {
    data: DrivingTaskScheduleData,
    sender: oneshot::Sender<()>,
}

#[derive(Clone)]
pub struct DrivingTask {
    task_sender: mpsc::SyncSender<DrivingTaskInit>,
}

pub struct DrivingIsBusy {
    pub data: DrivingTaskScheduleData,
}

impl Display for DrivingIsBusy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Waypoint Driver is busy completing a previous task")
    }
}

impl Debug for DrivingIsBusy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "DrivingIsBusy")
    }
}

impl std::error::Error for DrivingIsBusy {}

pub struct DrivingTaskScheduleData {
    pub destination: Point2<Float>,
    pub orientation: Option<UnitQuaternion<Float>>
}

#[async_trait]
impl Task for DrivingTask {
    type Output = ();
    type ScheduleError = DrivingIsBusy;
    type ScheduleData = DrivingTaskScheduleData;
    type TaskData = ();

    async fn try_schedule(
        &self,
        data: Self::ScheduleData,
    ) -> Result<TaskHandle<Self>, Self::ScheduleError> {
        let (sender, recv) = oneshot::channel();

        if let Err(e) = self.task_sender.try_send(DrivingTaskInit { data, sender }) {
            let data = match e {
                mpsc::TrySendError::Full(x) => x.data,
                mpsc::TrySendError::Disconnected(x) => x.data,
            };
            return Err(DrivingIsBusy { data });
        }

        Ok(TaskHandle::from_oneshot_receiver(recv, (), "waypoint-driving".into()).await)
    }
}

pub struct WaypointDriver {
    driving_task: DrivingTask,
    steering_signal: Publisher<Steering>,
    robot_base: RobotBaseRef,
    task_receiver: mpsc::Receiver<DrivingTaskInit>,
    pub completion_distance: Float,
    costmap_ref: CostmapRef,
    pub refresh_rate: Duration,
    pub agent_radius: Float,
    pub max_height_diff: Float,
    pub can_reverse: bool,
    pub width: Float,
    pub angle_epsilon: Float,
}

impl WaypointDriver {
    pub fn new(
        robot_base: RobotBaseRef,
        costmap_ref: CostmapRef,
        agent_radius: Float,
        max_height_diff: Float,
        width: Float,
    ) -> Self {
        let (task_sender, task_receiver) = mpsc::sync_channel(0);
        assert!(agent_radius >= 0.0);
        assert!(max_height_diff >= 0.0);
        Self {
            driving_task: DrivingTask { task_sender },
            steering_signal: Default::default(),
            robot_base,
            task_receiver,
            completion_distance: 0.15,
            costmap_ref,
            refresh_rate: Duration::from_millis(60),
            agent_radius,
            max_height_diff,
            can_reverse: true,
            width,
            // 10 degrees
            angle_epsilon: 0.1745329,
        }
    }

    pub fn accept_steering_signal(&mut self, sub: Subscription<Steering>) {
        self.steering_signal.accept_subscription(sub);
    }

    pub fn get_driving_task(&self) -> &DrivingTask {
        &self.driving_task
    }
}

mod successors;

#[async_trait]
impl Node for WaypointDriver {
    const DEFAULT_NAME: &'static str = "waypoint-driver";

    async fn run(mut self, _context: RuntimeContext) -> anyhow::Result<()> {
        tokio_rayon::spawn(move || {
            let sleeper = spin_sleep::SpinSleeper::default();
            loop {
                let Ok(init) = self.task_receiver.recv() else {
                    break Ok(());
                };

                let mut waypoint = self.costmap_ref.global_to_local(init.data.destination);
                if waypoint.x < 0 {
                    waypoint.x = 0;
                }
                if waypoint.y < 0 {
                    waypoint.y = 0;
                }
                let mut waypoint = Vector2::new(waypoint.x as usize, waypoint.y as usize);
                if waypoint.x >= self.costmap_ref.get_area_width() {
                    waypoint.x = self.costmap_ref.get_area_width() - 1;
                }
                if waypoint.y >= self.costmap_ref.get_area_length() {
                    waypoint.y = self.costmap_ref.get_area_length() - 1;
                }
                let goal_forward = init.data.orientation.map(|orientation| {
                    let forward = orientation * Vector3::z_axis();
                    UnitVector2::new_normalize(Vector2::new(forward.x, forward.z))
                });

                loop {
                    let isometry = self.robot_base.get_isometry();

                    if (init.data.destination.coords - Vector2::new(isometry.translation.x, isometry.translation.z)).magnitude() <= self.completion_distance {
                        break;
                    }

                    let obstacles = self.costmap_ref.costmap_to_obstacle(
                        &self.costmap_ref.get_costmap(),
                        self.max_height_diff,
                        isometry.translation.y,
                        self.agent_radius,
                    );

                    let mut position = self.costmap_ref.global_to_local(Point2::new(
                        isometry.translation.x,
                        isometry.translation.y,
                    ));
                    if position.x < 0 {
                        position.x = 0;
                    }
                    if position.y < 0 {
                        position.y = 0;
                    }
                    let mut position = Vector2::new(position.x as usize, position.y as usize);
                    if position.x >= self.costmap_ref.get_area_width() {
                        position.x = self.costmap_ref.get_area_width() - 1;
                    }
                    if position.y >= self.costmap_ref.get_area_length() {
                        position.y = self.costmap_ref.get_area_length() - 1;
                    }

                    let forward = isometry.get_forward_vector();
                    let forward =
                        UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

                    let Some((raw_path, _distance)) = astar(
                        &RobotState {
                            position,
                            forward,
                            // The following values are ignored on the starting state
                            reversing: false,
                            arc_angle: 0.0,
                            turn_first: false,
                            radius: 0.0,
                        },
                        |current| successors(current, &obstacles, true, self.width),
                        |current| {
                            let mut cost = NotNan::new(
                                (current.position.cast::<f32>() - waypoint.cast()).magnitude(),
                            )
                            .unwrap();

                            if let Some(goal_forward) = goal_forward {
                                cost += goal_forward.angle(&current.forward) * self.width / 2.0;
                            }

                            cost
                        },
                        |current| current.position == waypoint,
                    ) else {
                        todo!("Failed to find path")
                    };

                    let next = &raw_path[1];

                    if next.turn_first {
                        // We don't need any logic for actually driving straight after the turn is complete
                        // Because by that point the pathfinding algorithm will have changed the next
                        // action to not turn first but instead swerve drive.
                        //
                        // But of course, that is just a hypothesis.
                        let value = if next.reversing { -1.0 } else { 1.0 };
                        if next.arc_angle > 0.0 {
                            self.steering_signal.set(Steering::new(-value, value));
                        } else {
                            self.steering_signal.set(Steering::new(value, -value));
                        }
                    } else if next.arc_angle.abs() <= self.angle_epsilon {
                        if next.reversing {
                            self.steering_signal.set(Steering::new(-1.0, -1.0));
                        } else {
                            self.steering_signal.set(Steering::new(1.0, 1.0));
                        }
                    } else {
                        let smaller_ratio =
                            (next.radius - self.width / 2.0) / (next.radius + self.width / 2.0);
                        if next.arc_angle > 0.0 {
                            if next.reversing {
                                self.steering_signal
                                    .set(Steering::new(-smaller_ratio, -1.0));
                            } else {
                                self.steering_signal.set(Steering::new(smaller_ratio, 1.0));
                            }
                        } else {
                            if next.reversing {
                                self.steering_signal.set(Steering::new(1.0, smaller_ratio));
                            } else {
                                self.steering_signal
                                    .set(Steering::new(-1.0, -smaller_ratio));
                            }
                        }
                    }

                    sleeper.sleep(self.refresh_rate);
                }

                if let Some(goal_forward) = goal_forward {
                    // Turn to goal orientation
                    loop {
                        let forward = self.robot_base.get_isometry().get_forward_vector();
                        let forward =
                            UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));
                        
                        let arc_angle = (forward.x * goal_forward.y - forward.y * goal_forward.x).signum() * forward.angle(&goal_forward);

                        if arc_angle.abs() < self.angle_epsilon {
                            break;
                        }

                        if arc_angle > 0.0 {
                            self.steering_signal.set(Steering::new(-1.0, 1.0));
                        } else {
                            self.steering_signal.set(Steering::new(1.0, -1.0));
                        }

                        sleeper.sleep(self.refresh_rate);
                    }
                }

                let _ = init.sender.send(());
            }
        })
        .await
    }
}
