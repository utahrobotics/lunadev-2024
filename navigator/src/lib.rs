use std::{
    fmt::{Debug, Display},
    sync::mpsc, f32::consts::PI,
};

use global_msgs::Steering;
use nalgebra::{Point3, UnitQuaternion, Vector3, Point2, Vector2, UnitVector2, wrap};
use pid::Pid;
use unros_core::{
    anyhow, async_trait,
    signal::{watched::WatchedSubscription, Signal, SignalRef},
    task::{Task, TaskHandle},
    tokio::{sync::oneshot, self},
    Node, RuntimeContext, tokio_rayon,
};

pub use pid;

struct DrivingTaskInit {
    data: DrivingTaskScheduleData,
    sender: oneshot::Sender<()>
}

#[derive(Clone)]
pub struct DrivingTask {
    task_sender: mpsc::SyncSender<DrivingTaskInit>,
}

pub struct DrivingIsBusy {
    pub data: DrivingTaskScheduleData
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
    pub waypoints: Vec<Point2<f32>>,
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

        Ok(TaskHandle::from_oneshot_receiver(
            recv,
            (),
            "waypoint-driving".into(),
        )
        .await)
    }
}

pub struct WaypointDriver {
    driving_task: DrivingTask,
    steering_signal: Signal<Steering>,
    position: WatchedSubscription<Point3<f32>>,
    velocity: WatchedSubscription<Vector3<f32>>,
    orientation: WatchedSubscription<UnitQuaternion<f32>>,
    task_receiver: mpsc::Receiver<DrivingTaskInit>,
    drive_pid: Pid<f64>,
    steering_pid: Pid<f64>,
    completion_distance: f32
}

impl WaypointDriver {
    pub fn new(
        position: WatchedSubscription<Point3<f32>>,
        velocity: WatchedSubscription<Vector3<f32>>,
        orientation: WatchedSubscription<UnitQuaternion<f32>>,
        drive_pid: Pid<f64>,
        steering_pid: Pid<f64>,

    ) -> Self {
        let (task_sender, task_receiver) = mpsc::sync_channel(0);
        Self {
            driving_task: DrivingTask { task_sender },
            steering_signal: Default::default(),
            position,
            velocity,
            orientation,
            task_receiver,
            drive_pid,
            steering_pid,
            completion_distance: 0.3
        }
    }

    pub fn get_steering_signal(&mut self) -> SignalRef<Steering> {
        self.steering_signal.get_ref()
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
        let _ = tokio_rayon::spawn(move || {
            loop {
                let Ok(init) = self.task_receiver.recv() else { break; };
                if task_sender.blocking_send(init).is_err() {
                    break;
                }
            }
        });
        // Safe to unwrap as the result will only
        // be an error if task_sender was dropped,
        // but so long as we are in this method,
        // task_sender will not be dropped
        let mut init = task_receiver.recv().await.unwrap();
        'main: loop {
            let DrivingTaskInit { data, sender } = init;

            for waypoint in data.waypoints {
                let mut position = self.position.get().await;
                let mut _velocity = self.velocity.get().await;
                let mut orientation = self.orientation.get().await;

                loop {
                    let position2 = Vector2::new(position.x, position.z);
                    let mut travel = waypoint.coords - position2;
                    if travel.magnitude() <= self.completion_distance {
                        break;
                    }
                    let front_vector = orientation * - Vector3::z_axis();
                    let front_vector = UnitVector2::new_normalize(Vector2::new(front_vector.x, front_vector.z));

                    let mut remaining_yaw = travel.y.atan2(travel.x) - front_vector.y.atan2(front_vector.x);
                    remaining_yaw = wrap(remaining_yaw, -PI, PI);

                    let control = self.steering_pid.next_control_output(remaining_yaw as f64);
                    let steering = (control.output / self.steering_pid.output_limit) as f32;

                    

                    tokio::select! {
                        new_init = task_receiver.recv() => {
                            let new_init = new_init.unwrap();
                            if new_init.data.force {
                                init = new_init;
                                continue 'main;
                            }
                        }
                        new_position = self.position.wait_for_change() => {
                            position = new_position;
                        }
                        new_velocity = self.velocity.wait_for_change() => {
                            _velocity = new_velocity;
                        }
                        new_orientation = self.orientation.wait_for_change() => {
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
