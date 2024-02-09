use std::{
    fmt::{Debug, Display}, sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    }, time::Duration
};

use costmap::CostmapRef;
use global_msgs::Steering;
use nalgebra::{Point2, UnitQuaternion, UnitVector2, Vector2, Vector3};
use ordered_float::NotNan;
use pathfinding::directed::astar::astar;
use rig::{RigSpace, RobotBaseRef};
use successors::{successors, traverse_to, RobotState};
use unros_core::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscription},
    setup_logging,
    task::{Task, TaskHandle},
    tokio::{self, sync::oneshot},
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
    pub orientation: Option<UnitQuaternion<Float>>,
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
    ) -> Result<TaskHandle<Self::Output, Self::TaskData>, Self::ScheduleError> {
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
        assert!(width >= 0.0);
        Self {
            driving_task: DrivingTask { task_sender },
            steering_signal: Default::default(),
            robot_base,
            task_receiver,
            completion_distance: 0.15,
            costmap_ref,
            refresh_rate: Duration::from_millis(20),
            agent_radius,
            max_height_diff,
            can_reverse: true,
            width,
            // 10 degrees
            angle_epsilon: 0.1745329,
        }
    }

    pub fn accept_steering_sub(&mut self, sub: Subscription<Steering>) {
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

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        let (path_sender, path_recv) = std::sync::mpsc::sync_channel(0);
        let completed = Arc::new(AtomicBool::default());
        let driver_completed = completed.clone();
        let robot_base = self.robot_base.clone();

        let pathfinder = tokio_rayon::spawn(move || {
            let mut job_counter = 0usize;
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

                    let obstacles = self.costmap_ref.costmap_to_obstacle(
                        &self.costmap_ref.get_costmap(),
                        self.max_height_diff,
                        isometry.translation.y,
                        self.agent_radius,
                    );

                    let mut position = self.costmap_ref.global_to_local(Point2::new(
                        isometry.translation.x,
                        isometry.translation.z,
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
                    let forward = UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));
                    let width = self.width as f32 / self.costmap_ref.get_cell_width();

                    let Some((path, _distance)) = astar(
                        &RobotState {
                            position,
                            forward,
                            // The following values are ignored on the starting state
                            left_steering: Default::default(),
                            right_steering: Default::default(),
                        },
                        |current| {
                            successors(
                                *current,
                                &obstacles,
                                self.can_reverse,
                                width,
                            )
                        },
                        |current| {
                            let travel = current.position.cast::<Float>() - waypoint.cast();
                            let mut cost = NotNan::new(travel.magnitude()).unwrap()
                                + travel.angle(&current.forward) * self.width / 2.0;

                            if let Some(goal_forward) = goal_forward {
                                cost += goal_forward.angle(&current.forward) * self.width / 2.0;
                            }

                            cost
                        },
                        |current| current.position == waypoint,
                    ) else {
                        todo!("Failed to find path")
                    };

                    // println!("A {:?}", path.first().unwrap());
                    // println!("B {:?}", traverse_to(path.first().unwrap(), path.last().unwrap(), &obstacles, self.can_reverse, width).unwrap());
                    // let mut new_path = Vec::with_capacity(path.len());
                    // let mut path = path.into_iter();

                    // let mut start = path.next().unwrap();
                    // new_path.push(start);

                    // let mut old_next = path.next().unwrap();
 
                    // while let Some(next) = path.next() {
                    //     if let Some(new_next) = traverse_to(&start, &next, &obstacles, self.can_reverse, self.width) {
                    //         old_next = new_next;
                    //     } else {
                    //         new_path.push(old_next);
                    //         start = old_next;
                    //         old_next = next;
                    //     }
                    // }

                    // new_path.push(old_next);

                    let new_path = path;
                    // println!("B {:?}", new_path.len());

                    let mut path: Vec<RobotState<Float>> = new_path
                        .into_iter()
                        .map(
                            |RobotState {
                                 position,
                                 forward,
                                 left_steering,
                                 right_steering,
                             }| RobotState {
                                position: self
                                    .costmap_ref
                                    .local_to_global(position.cast().into())
                                    .coords,
                                forward,
                                left_steering,
                                right_steering,
                            },
                        )
                        .collect();

                    path.first_mut().unwrap().position =
                        Vector2::new(isometry.translation.x, isometry.translation.z);
                    path.last_mut().unwrap().position = init.data.destination.coords;
                    // println!("{:?}", path.last().unwrap());

                    if path_sender
                        .send((job_counter, Some((path, goal_forward))))
                        .is_err()
                    {
                        return Ok(());
                    }

                    if driver_completed.load(Ordering::Relaxed) {
                        driver_completed.store(false, Ordering::Relaxed);
                        break;
                    }
                }

                job_counter += 1;
                let _ = init.sender.send(());
            }
        });

        let driver = tokio_rayon::spawn(move || {
            let mut job_counter = 0usize;
            let sleeper = spin_sleep::SpinSleeper::default();

            'main: loop {
                let Ok(item) = path_recv.recv() else {
                    return Ok(());
                };
                let (new_job_index, item) = item;
                if new_job_index != job_counter {
                    continue;
                }
                let (mut path, mut _goal_forward) = item.unwrap();

                loop {
                    match path_recv.try_recv() {
                        Ok((new_job_index, x)) => {
                            if let Some((new_path, _)) = x {
                                debug_assert_eq!(new_job_index, job_counter);
                                path = new_path;
                            } else {
                                continue 'main;
                            }
                        }
                        Err(e) => {
                            if e == mpsc::TryRecvError::Disconnected {
                                return Ok(());
                            }
                        }
                    }
                    let isometry = robot_base.get_isometry();
                    let position = Vector2::new(isometry.translation.x, isometry.translation.z);

                    let (i, distance) = path
                        .iter()
                        .enumerate()
                        .map(|(i, x)| (i, (x.position - position).magnitude_squared()))
                        .min_by_key(|(_, d)| NotNan::new(*d).unwrap())
                        .unwrap();

                    if (path.last().unwrap().position - position).magnitude()
                        < self.completion_distance
                    {
                        break;
                    }

                    // If we are closest to the last node but did not satisfy the end condition
                    // OR
                    // If we are simply too far from the path
                    if i == path.len() - 1 || distance > self.completion_distance {
                        error!("Exceeded max offset from path: {distance}");
                        self.steering_signal.set(Steering::new(0.0, 0.0));
                        let Ok(item) = path_recv.recv() else {
                            return Ok(());
                        };
                        let (new_job_index, item) = item;
                        if new_job_index != job_counter {
                            continue;
                        }
                        let Some((new_path, _)) = item else {
                            continue 'main;
                        };
                        path = new_path;
                        continue;
                    }

                    let next = path.get(i + 1).unwrap();

                    self.steering_signal.set(Steering {
                        left: next.left_steering,
                        right: next.right_steering,
                    });

                    sleeper.sleep(self.refresh_rate);
                }

                completed.store(true, Ordering::Relaxed);
                job_counter += 1;
                self.steering_signal.set(Steering::new(0.0, 0.0));

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
        });
        tokio::select! {
            res = pathfinder => res,
            res = driver => res
        }
    }
}
