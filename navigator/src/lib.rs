use std::{
    f32::consts::PI,
    fmt::{Debug, Display},
    sync::mpsc,
    time::Duration,
};

use costmap::CostmapRef;
use global_msgs::Steering;
use nalgebra::{wrap, Isometry3, Point2, UnitVector2, UnitVector3, Vector2, Vector3};
use ordered_float::NotNan;
use pathfinding::directed::astar::astar;
use rig::{RigSpace, RobotBaseRef};
use successors::SUCCESSORS;
use unros_core::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscription},
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
    pub waypoints: Vec<Point2<Float>>,
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
    pub agent_radius: f32,
    pub max_height_diff: f32,
    // pub max_
}

impl WaypointDriver {
    pub fn new(
        robot_base: RobotBaseRef,
        costmap_ref: CostmapRef,
        agent_radius: f32,
        max_height_diff: f32,
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
    const DEFAULT_NAME: &'static str = "waypoint_driver";

    async fn run(mut self, _context: RuntimeContext) -> anyhow::Result<()> {
        tokio_rayon::spawn(move || {
            let sleeper = spin_sleep::SpinSleeper::default();
            loop {
                let init = self.task_receiver.recv().unwrap();

                for waypoint in init.data.waypoints {
                    loop {
                        let isometry = self.robot_base.get_isometry();
                        let obstacles = self.costmap_ref.costmap_to_obstacle(
                            self.costmap_ref.get_costmap(),
                            self.max_height_diff,
                            isometry.translation.y,
                            self.agent_radius,
                        );

                        let mut origin = self.costmap_ref.global_to_local(Point2::new(
                            isometry.translation.x,
                            isometry.translation.y,
                        ));
                        if origin.x < 0 {
                            origin.x = 0;
                        }
                        if origin.y < 0 {
                            origin.y = 0;
                        }
                        let mut origin: Vector2<usize> =
                            Vector2::new(origin.x as usize, origin.y as usize);
                        if origin.x >= self.costmap_ref.get_area_width() {
                            origin.x = self.costmap_ref.get_area_width() - 1;
                        }
                        if origin.y >= self.costmap_ref.get_area_length() {
                            origin.y = self.costmap_ref.get_area_length() - 1;
                        }

                        let mut waypoint = self.costmap_ref.global_to_local(waypoint);
                        if waypoint.x < 0 {
                            waypoint.x = 0;
                        }
                        if waypoint.y < 0 {
                            waypoint.y = 0;
                        }
                        let mut waypoint: Vector2<usize> =
                            Vector2::new(waypoint.x as usize, waypoint.y as usize);
                        if waypoint.x >= self.costmap_ref.get_area_width() {
                            waypoint.x = self.costmap_ref.get_area_width() - 1;
                        }
                        if waypoint.y >= self.costmap_ref.get_area_length() {
                            waypoint.y = self.costmap_ref.get_area_length() - 1;
                        }

                        let Some((raw_path, _distance)) = astar(
                            &origin,
                            |p| {
                                let p: Vector2<isize> = p.cast();
                                SUCCESSORS
                                    .into_iter()
                                    .map(move |(x, c)| (x + p, c))
                                    .filter_map(|(p, c)|
                                        if p.x < 0 || p.y < 0 {
                                            None
                                        } else {
                                            let p = Vector2::new(p.x as usize, p.y as usize);
                                            if *obstacles.get((p.x, p.y))? {
                                                Some((p, NotNan::new(c).unwrap()))
                                            } else {
                                                None
                                            }
                                        }
                                    )
                            },
                            |p| NotNan::new((p - waypoint).cast::<f32>().magnitude()).unwrap(),
                            |p| p == &waypoint,
                        ) else {
                            todo!("Failed to find path")
                        };

                        let raw_path: Box<[_]> = raw_path.into_iter().map(|x| x.cast::<f32>()).collect();

                        let mut travel = UnitVector2::new_normalize(raw_path[1] - raw_path[0]);
                        let forward = self.robot_base.get_isometry().get_forward_vector();
                        let forward = UnitVector2::new_normalize(Vector2::new(forward.x, forward.z));

                        // if forward.angle(&travel) {

                        // }

                        sleeper.sleep(self.refresh_rate);
                    }
                }

                let _ = init.sender.send(());
            }
        })
        .await
    }
}
