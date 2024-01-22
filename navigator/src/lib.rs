use std::{
    f64::consts::PI,
    fmt::{Debug, Display},
    sync::mpsc,
};

use global_msgs::Steering;
use nalgebra::{wrap, Isometry3, Point2, UnitVector2, Vector2, Vector3};
use ordered_float::NotNan;
use pid::Pid;
use rig::RobotBaseRef;
use unros_core::{
    anyhow, async_trait,
    signal::{Publisher, Subscription},
    task::{Task, TaskHandle},
    tokio::{self, sync::oneshot},
    tokio_rayon, Node, RuntimeContext,
};

pub use pid;

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
    pub force: bool,
    pub waypoints: Vec<Point2<f64>>,
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
    steering_pid: Pid<f64>,
    pub completion_distance: f64,
    pub wheel_separation: f64,
}

impl WaypointDriver {
    pub fn new(robot_base: RobotBaseRef, steering_pid: Pid<f64>) -> Self {
        let (task_sender, task_receiver) = mpsc::sync_channel(0);
        Self {
            driving_task: DrivingTask { task_sender },
            steering_signal: Default::default(),
            robot_base,
            task_receiver,
            steering_pid,
            completion_distance: 0.3,
            wheel_separation: 0.7,
        }
    }

    pub fn accept_steering_signal(&mut self, sub: Subscription<Steering>) {
        self.steering_signal.accept_subscription(sub);
    }

    pub fn get_driving_task(&self) -> &DrivingTask {
        &self.driving_task
    }
}

#[async_trait]
impl Node for WaypointDriver {
    const DEFAULT_NAME: &'static str = "waypoint_driver";

    async fn run(mut self, _context: RuntimeContext) -> anyhow::Result<()> {
        let (task_sender, mut task_receiver) = tokio::sync::mpsc::channel(1);
        let _ = tokio_rayon::spawn(move || loop {
            let Ok(init) = self.task_receiver.recv() else {
                break;
            };
            if task_sender.blocking_send(init).is_err() {
                break;
            }
        });

        let mut left_pid = self.steering_pid.clone();
        let mut right_pid = self.steering_pid;

        // Safe to unwrap as the result will only
        // be an error if task_sender was dropped,
        // but so long as we are in this method,
        // task_sender will not be dropped
        let mut init = task_receiver.recv().await.unwrap();
        let half_wheel_separation = self.wheel_separation / 2.0;
        'main: loop {
            let DrivingTaskInit { data, sender } = init;
            let mut distance_travelled = 0.0f64;
            let Isometry3 {
                translation,
                rotation: mut orientation,
            } = self.robot_base.get_isometry();
            let mut position = translation.vector;
            let mut yaw_travelled = 0.0f64;

            for waypoint in data.waypoints {
                loop {
                    let position2 = Vector2::new(position.x, position.z);
                    let travel = waypoint.coords - position2;
                    let distance = travel.magnitude();
                    if travel.magnitude() <= self.completion_distance {
                        break;
                    }
                    let front_vector = orientation * -Vector3::z_axis();
                    let front_vector =
                        UnitVector2::new_normalize(Vector2::new(front_vector.x, front_vector.z));

                    let mut remaining_yaw =
                        travel.y.atan2(travel.x) - front_vector.y.atan2(front_vector.x);
                    remaining_yaw = wrap(remaining_yaw, -PI, PI);

                    let abs_remaining_yaw = remaining_yaw.abs();

                    if abs_remaining_yaw > PI / 2.0 {
                        // TODO: double check this is correct
                        let (left, right) = if remaining_yaw < 0.0 {
                            (-1.0, 1.0)
                        } else {
                            (-1.0, 1.0)
                        };

                        left_pid.setpoint(left);
                        right_pid.setpoint(right);

                        // We do not need to query the pid as we would want to rotate at full speed in this scenario
                        self.steering_signal.set(Steering {
                            left: NotNan::new(left as f32).unwrap(),
                            right: NotNan::new(right as f32).unwrap(),
                        })
                    } else {
                        let turning_radius =
                            distance / (2.0 * (1.0 - (2.0 * abs_remaining_yaw).cos()).sqrt());
                        let true_distance = turning_radius * 2.0 * abs_remaining_yaw;

                        let inner_turning_radius =
                            true_distance - half_wheel_separation / turning_radius;
                        let outer_turning_radius =
                            true_distance + half_wheel_separation / turning_radius;
                        let ratio = inner_turning_radius / outer_turning_radius;

                        // TODO: double check this is correct
                        let (left, right) = if remaining_yaw < 0.0 {
                            (ratio, 1.0)
                        } else {
                            (1.0, ratio)
                        };

                        left_pid.setpoint(left);
                        right_pid.setpoint(right);

                        // Estimate the actual current drive ratio
                        let (left, right) = if yaw_travelled.abs() < 0.001 {
                            (1.0, 1.0)
                        } else {
                            let last_turning_radius = distance_travelled / yaw_travelled.abs();
                            let ratio = (last_turning_radius - half_wheel_separation)
                                / (last_turning_radius + half_wheel_separation);

                            if yaw_travelled < 0.0 {
                                (ratio, 1.0)
                            } else {
                                (1.0, ratio)
                            }
                        };

                        // Feed into PIDs to get control values for each side
                        let mut control = left_pid.next_control_output(left as f64);
                        let left = (control.output / left_pid.output_limit) as f64;
                        control = right_pid.next_control_output(right as f64);
                        let right = (control.output / left_pid.output_limit) as f64;

                        self.steering_signal.set(Steering {
                            left: NotNan::new(left as f32).unwrap(),
                            right: NotNan::new(right as f32).unwrap(),
                        })
                    }

                    tokio::select! {
                        new_init = task_receiver.recv() => {
                            let new_init = new_init.unwrap();
                            if new_init.data.force {
                                init = new_init;
                                continue 'main;
                            }
                        }
                        () = self.robot_base.wait_for_change() => {
                            let Isometry3 { translation, rotation: new_orientation } = self.robot_base.get_isometry();
                            let new_position = translation.vector;
                            distance_travelled = (new_position - position).magnitude();
                            position = new_position;

                            let old_front_vector = orientation * - Vector3::z_axis();
                            let old_front_vector = UnitVector2::new_normalize(Vector2::new(old_front_vector.x, old_front_vector.z));

                            let new_front_vector = new_orientation * - Vector3::z_axis();
                            let new_front_vector = UnitVector2::new_normalize(Vector2::new(new_front_vector.x, new_front_vector.z));

                            yaw_travelled = new_front_vector.y.atan2(new_front_vector.x) - old_front_vector.y.atan2(old_front_vector.x);

                            orientation = new_orientation;
                        }
                    }
                }
            }

            let _ = sender.send(());
            init = task_receiver.recv().await.unwrap();
        }
    }
}

// fn pid_update_position()
