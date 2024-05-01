use std::{
    future::Future,
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use nalgebra::{convert as nconvert, Isometry3, Point3};
use obstacles::{utils::RecycledVec, HeightQuery, ObstacleHub};
use rig::RobotBaseRef;
use unros::{
    float::Float,
    node::AsyncNode,
    pubsub::{Publisher, PublisherRef},
    runtime::RuntimeContext,
    service::{new_service, Service, ServiceHandle},
    setup_logging, tokio, DontDrop, ShouldNotDrop,
};

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
        from: Isometry3<N>,
        end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        context: &RuntimeContext,
    ) -> impl Future<Output = Option<Vec<Point3<N>>>> + Send;

    fn traverse_to(
        &mut self,
        isometry: Isometry3<N>,
        from: Point3<N>,
        to: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
    ) -> impl Future<Output = bool> + Send;
}

#[derive(ShouldNotDrop)]
pub struct Pathfinder<N: Float, E: PathfindingEngine<N>> {
    engine: E,
    obstacle_hub: ObstacleHub<N>,
    dont_drop: DontDrop<Self>,
    service: Service<Point3<N>, NavigationError, NavigationProgress, Result<(), NavigationError>>,
    service_handle: NavigationServiceHandle<N>,
    robot_base: RobotBaseRef,
    path_pub: Publisher<Arc<[Point3<N>]>>,
    pub completion_distance: N,
    pub correction_distance: N,
    pub resolution: N,
    pub refresh_rate: Duration,
    pub max_fail_rate: N,
    pub repathfinding_window: usize,
}

impl<N: Float, E: PathfindingEngine<N>> Pathfinder<N, E> {
    pub fn new_with_engine(
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
            correction_distance: nalgebra::convert(0.15),
            resolution,
            refresh_rate: Duration::from_millis(100),
            repathfinding_window: 25,
            max_fail_rate: nalgebra::convert(0.5),
        }
    }

    pub fn get_path_pub(&self) -> PublisherRef<Arc<[Point3<N>]>> {
        self.path_pub.get_ref()
    }

    pub fn get_navigation_handle(&self) -> NavigationServiceHandle<N> {
        self.service_handle.clone()
    }
}

impl<N: Float, E: PathfindingEngine<N>> AsyncNode for Pathfinder<N, E>
where
    RecycledVec<HeightQuery<N>>: Default,
{
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
            let mut repathfinding_window =
                vec![false; self.repathfinding_window].into_boxed_slice();

            'main: loop {
                start_time += start_time.elapsed();

                let mut isometry: Isometry3<N> = nconvert(self.robot_base.get_isometry());

                if let Some(path) = self
                    .engine
                    .pathfind(isometry, end, &self.obstacle_hub, self.resolution, &context)
                    .await
                {
                    let path: Arc<[Point3<N>]> = path.into();
                    self.path_pub.set(path.clone());
                    repathfinding_window
                        .iter_mut()
                        .for_each(|flag| *flag = false);
                    let mut repathfinding_i = 0usize;
                    let max_fails = (self.max_fail_rate * nconvert(self.repathfinding_window))
                        .round()
                        .to_usize();

                    'repathfind: loop {
                        if repathfinding_window.iter().filter(|&flag| *flag).count() >= max_fails {
                            break;
                        }
                        tokio::time::sleep(self.refresh_rate).await;
                        isometry = nconvert(self.robot_base.get_isometry());

                        if (end.coords - isometry.translation.vector).magnitude()
                            <= self.completion_distance
                        {
                            pending_task.finish(Ok(()));
                            break 'main;
                        }
                        if let Some(min_offset) = path
                            .windows(2)
                            .filter_map(|window| {
                                let [from, to] = window.try_into().unwrap();
                                let relative = isometry.translation.vector - from.coords;
                                let mut travel = to.coords - from.coords;
                                let distance = travel.magnitude();
                                travel.unscale_mut(distance);

                                let length_along = relative.dot(&travel);
                                if length_along < N::zero() || length_along > distance {
                                    None
                                } else {
                                    Some((relative - travel * length_along).magnitude())
                                }
                            })
                            .min_by(|a, b| a.total_cmp(b))
                        {
                            if min_offset > self.correction_distance {
                                error!("Too far from path");
                                repathfinding_window[repathfinding_i] = true;
                                repathfinding_i = (repathfinding_i + 1) % self.repathfinding_window;
                                continue;
                            }
                        } else if (isometry.translation.vector - path[0].coords).magnitude()
                            > self.correction_distance
                        {
                            error!("Too far from path");
                            repathfinding_window[repathfinding_i] = true;
                            repathfinding_i = (repathfinding_i + 1) % self.repathfinding_window;
                            continue;
                        }
                        for window in path.windows(2) {
                            let [from, to] = window.try_into().unwrap();

                            if !self
                                .engine
                                .traverse_to(
                                    isometry,
                                    from,
                                    to,
                                    &self.obstacle_hub,
                                    self.resolution,
                                )
                                .await
                            {
                                error!("Obstacle on path");
                                repathfinding_window[repathfinding_i] = true;
                                repathfinding_i = (repathfinding_i + 1) % self.repathfinding_window;
                                continue 'repathfind;
                            }
                        }
                        repathfinding_window[repathfinding_i] = false;
                        repathfinding_i = (repathfinding_i + 1) % self.repathfinding_window;
                    }
                    self.path_pub.set(Arc::new([]));
                } else {
                    pending_task.finish(Err(NavigationError::NoPath));
                    break;
                }
            }
            self.path_pub.set(Arc::new([]));
        }
    }
}
