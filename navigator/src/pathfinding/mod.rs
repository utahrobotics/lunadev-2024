use std::{
    future::Future,
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use nalgebra::{convert as nconvert, Isometry3, Point3, UnitQuaternion, Vector3};
use obstacles::{utils::RecycledVec, HeightQuery, ObstacleHub, Shape};
use rig::RobotBaseRef;
use unros::{
    float::Float,
    node::AsyncNode,
    pubsub::{Publisher, PublisherRef},
    runtime::RuntimeContext,
    service::{new_service, Service, ServiceHandle},
    setup_logging, tokio, DontDrop, ShouldNotDrop,
};

use self::direct::DirectPathfinder;

mod alg;
pub mod direct;

#[derive(Debug)]
pub enum NavigationError {
    NoPath,
}

impl std::fmt::Display for NavigationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoPath => write!(f, "There was no path to the destination"),
        }
    }
}

impl std::error::Error for NavigationError {}

pub struct NavigationProgress {
    completion_percentage: Arc<AtomicU8>,
}

impl NavigationProgress {
    pub fn get_completion_percentage(&self) -> f32 {
        self.completion_percentage.load(Ordering::Relaxed) as f32 / 255.0
    }
}

pub type NavigationServiceHandle<N> =
    ServiceHandle<Point3<N>, NavigationError, NavigationProgress, Result<(), NavigationError>>;

// pub struct CompletionPercentage<'a> {
//     completion_percentage: &'a AtomicU8,
// }

// impl<'a> CompletionPercentage<'a> {
//     pub fn set_completion_percentage<N: RealField + FloatToInt<u8> + SupersetOf<u8>>(
//         &self,
//         percentage: NotNan<N>,
//     ) {
//         let percentage = percentage.into_inner() * nalgebra::convert(255.0);
//         let percentage = unsafe { percentage.to_int_unchecked() };
//         self.completion_percentage
//             .store(percentage, Ordering::Relaxed);
//     }
// }

pub trait PathfindingEngine<N: Float>: Send + 'static {
    fn pathfind(
        &mut self,
        end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        shape: Shape<N>,
        max_height_diff: N,
        context: &RuntimeContext,
    ) -> impl Future<Output = Option<Vec<Point3<N>>>> + Send;

    fn traverse_to(
        &mut self,
        from: Vector3<N>,
        to: Vector3<N>,
        shape: Shape<N>,
        max_height_diff: N,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
    ) -> impl Future<Output = bool> + Send;
}

#[derive(ShouldNotDrop)]
pub struct Pathfinder<N: Float = f32, E: PathfindingEngine<N> = DirectPathfinder<N>> {
    engine: E,
    obstacle_hub: ObstacleHub<N>,
    dont_drop: DontDrop<Self>,
    service: Service<Point3<N>, NavigationError, NavigationProgress, Result<(), NavigationError>>,
    service_handle: NavigationServiceHandle<N>,
    robot_base: RobotBaseRef,
    path_pub: Publisher<Arc<[Point3<N>]>>,
    completion_distance: N,
    pub resolution: N,
    pub shape: Shape<N>,
    pub max_height_diff: N,
    pub refresh_rate: Duration,
}

impl<N: Float, E: PathfindingEngine<N>> Pathfinder<N, E> {
    pub fn new_with_engine(
        shape: Shape<N>,
        resolution: N,
        engine: E,
        obstacle_hub: ObstacleHub<N>,
        robot_base: RobotBaseRef,
    ) -> Self {
        let (service, service_handle) = new_service();
        Self {
            engine,
            obstacle_hub,
            dont_drop: DontDrop::new("pathfinder"),
            service,
            service_handle,
            robot_base,
            path_pub: Publisher::default(),
            completion_distance: nalgebra::convert(0.15),
            resolution,
            shape,
            max_height_diff: nalgebra::convert(0.1),
            refresh_rate: Duration::from_millis(100),
        }
    }

    pub fn get_path_pub(&self) -> PublisherRef<Arc<[Point3<N>]>> {
        self.path_pub.get_ref()
    }

    pub fn get_navigation_handle(&self) -> NavigationServiceHandle<N> {
        self.service_handle.clone()
    }
}

impl<N: Float, E: PathfindingEngine<N>> AsyncNode for Pathfinder<N, E>  where RecycledVec<HeightQuery<N>>: Default {
    type Result = ();

    async fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        drop(self.service_handle);
        self.dont_drop.ignore_drop = true;
        let mut start_time = Instant::now();

        loop {
            let Some(mut req) = self.service.wait_for_request().await else {
                break;
            };

            let end = req.take_input().unwrap();
            let completion_percentage = Arc::new(AtomicU8::default());

            let Some(pending_task) = req.accept(NavigationProgress {
                completion_percentage: completion_percentage.clone(),
            }) else {
                error!("Scheduler of task dropped task init before we could respond");
                continue;
            };

            'main: loop {
                start_time += start_time.elapsed();

                let mut isometry: Isometry3<N> = nconvert(self.robot_base.get_isometry());
                let mut end_local = isometry.inverse_transform_point(&end);

                if let Some(path) = self
                    .engine
                    .pathfind(
                        end_local,
                        &self.obstacle_hub,
                        self.resolution,
                        self.shape.clone(),
                        self.max_height_diff,
                        &context,
                    )
                    .await
                {
                    let path: Arc<[Point3<N>]> = path.into_iter().map(|p| isometry * p).collect();
                    self.path_pub.set(path.clone());

                    'repathfind: loop {
                        isometry = nconvert(self.robot_base.get_isometry());
                        end_local = isometry.inverse_transform_point(&end);

                        if end_local.coords.magnitude() <= self.completion_distance {
                            self.path_pub.set(Arc::new([]));
                            pending_task.finish(Ok(()));
                            break 'main;
                        }
                        for window in path.windows(2) {
                            let [from, to] = window.try_into().unwrap();
                            let relative = isometry.translation.vector - from.coords;
                            let travel = to.coords - from.coords;

                            let cross = travel.cross(&relative);

                            let rotation_90 = UnitQuaternion::<N>::new(
                                cross.normalize() * nconvert::<_, N>(std::f64::consts::PI / 2.0),
                            );
                            let offset_vec = rotation_90 * travel.normalize();
                            let offset = offset_vec.dot(&relative);

                            if offset > self.completion_distance {
                                break 'repathfind;
                            }

                            if !self.engine.traverse_to(
                                from.coords,
                                to.coords,
                                self.shape.clone(),
                                self.max_height_diff,
                                &self.obstacle_hub,
                                self.resolution,
                            )
                            .await
                            {
                                break 'repathfind;
                            }
                        }
                        tokio::time::sleep(self.refresh_rate).await;
                    }
                };
                // pending_task.finish(Err(NavigationError::NoPath));
            }
        }
    }
}

