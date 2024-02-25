use std::{
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use costmap::CostmapRef;
use nalgebra::{DMatrix, Point2, Point3, Vector2};
use ordered_float::NotNan;
use pathfinding::directed::astar::astar;
use rig::RobotBaseRef;
use unros_core::{
    anyhow, async_trait, pubsub::{Publisher, Subscription}, service::{new_service, Service, ServiceHandle}, setup_logging, tokio_rayon, Node, NodeIntrinsics, RuntimeContext
};

use crate::Float;

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
        1.0 / self.completion_percentage.load(Ordering::Relaxed) as f32
    }
}

pub type NavigationServiceHandle =
    ServiceHandle<Point3<Float>, NavigationError, NavigationProgress, Result<(), NavigationError>>;

pub struct DirectPathfinder {
    service_handle: NavigationServiceHandle,
    path_signal: Publisher<Vec<Point2<Float>>>,
    robot_base: RobotBaseRef,
    service: Service<Point3<Float>, NavigationError, NavigationProgress, Result<(), NavigationError>>,
    pub completion_distance: Float,
    costmap_ref: CostmapRef,
    pub refresh_rate: Duration,
    pub agent_radius: Float,
    pub max_height_diff: Float,
    intrinsics: NodeIntrinsics<Self>,
}

impl DirectPathfinder {
    pub fn new(
        robot_base: RobotBaseRef,
        costmap_ref: CostmapRef,
        agent_radius: Float,
        max_height_diff: Float,
    ) -> Self {
        let (service, service_handle) = new_service();
        assert!(agent_radius >= 0.0);
        assert!(max_height_diff >= 0.0);
        Self {
            path_signal: Default::default(),
            robot_base,
            service,
            completion_distance: 0.15,
            costmap_ref,
            refresh_rate: Duration::from_millis(50),
            agent_radius,
            max_height_diff,
            intrinsics: Default::default(),
            service_handle,
        }
    }

    pub fn accept_path_sub(&mut self, sub: Subscription<Vec<Point2<Float>>>) {
        self.path_signal.accept_subscription(sub);
    }

    pub fn get_navigation_handle(&self) -> NavigationServiceHandle {
        self.service_handle.clone()
    }
}

#[async_trait]
impl Node for DirectPathfinder {
    const DEFAULT_NAME: &'static str = "direct-pathfinder";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        tokio_rayon::spawn(move || {
            let sleeper = spin_sleep::SpinSleeper::default();
            'outer: loop {
                let Some(mut req) = self.service.blocking_wait_for_request() else {
                    break Ok(());
                };

                let dest = req.take_input().unwrap();
                let completion_percentage = Arc::new(AtomicU8::default());

                let Some(pending_task) = req.accept(NavigationProgress {
                    completion_percentage: completion_percentage.clone(),
                }) else {
                    error!("Scheduler of task dropped task init before we could respond");
                    continue;
                };

                let dest = Point2::new(dest.x, dest.z);
                let mut dest_grid = self.costmap_ref.global_to_local(dest);
                if dest_grid.x < 0 {
                    dest_grid.x = 0;
                }
                if dest_grid.y < 0 {
                    dest_grid.y = 0;
                }
                let mut dest_grid = Vector2::new(dest_grid.x as usize, dest_grid.y as usize);
                if dest_grid.x >= self.costmap_ref.get_area_width() {
                    dest_grid.x = self.costmap_ref.get_area_width() - 1;
                }
                if dest_grid.y >= self.costmap_ref.get_area_length() {
                    dest_grid.y = self.costmap_ref.get_area_length() - 1;
                }

                let mut start_time = Instant::now();

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

                    if (position.cast::<Float>() - dest_grid.cast()).magnitude()
                        <= self.completion_distance
                    {
                        break;
                    }

                    let Some((path, _distance)) = astar(
                        &position,
                        |current| {
                            let current = current.cast::<isize>();
                            const ROOT_2: Float = 1.4142135623730;
                            [
                                (Vector2::new(-1, -1) + current, ROOT_2),
                                (Vector2::new(-1, 0) + current, 1.0),
                                (Vector2::new(-1, 1) + current, ROOT_2),
                                (Vector2::new(0, -1) + current, 1.0),
                                (Vector2::new(0, 1) + current, 1.0),
                                (Vector2::new(1, -1) + current, ROOT_2),
                                (Vector2::new(1, 0) + current, 1.0),
                                (Vector2::new(1, 1) + current, ROOT_2),
                            ]
                            .into_iter()
                            .filter_map(|(next, cost)| {
                                if next.x < 0 || next.y < 0 {
                                    None
                                } else {
                                    let next = Vector2::new(next.x as usize, next.y as usize);

                                    if next.x >= self.costmap_ref.get_area_width()
                                        || next.y >= self.costmap_ref.get_area_length()
                                    {
                                        None
                                    } else if !*obstacles.get((next.y, next.x))? {
                                        return None;
                                    } else {
                                        Some((next, NotNan::new(cost).unwrap()))
                                    }
                                }
                            })
                        },
                        |current| {
                            NotNan::new((current.cast::<Float>() - dest_grid.cast()).magnitude())
                                .unwrap()
                        },
                        |current| *current == dest_grid,
                    ) else {
                        self.path_signal.set(vec![]);
                        pending_task.finish(Err(NavigationError::NoPath));
                        continue 'outer;
                    };

                    let mut new_path = Vec::with_capacity(path.len());
                    let mut path = path.into_iter();

                    let mut start = path.next().unwrap();
                    new_path.push(start.cast::<isize>());
                    let mut last = path.next().unwrap();

                    for next in path {
                        if traverse_to(start, next, &obstacles) {
                            last = next;
                        } else {
                            new_path.push(last.cast::<isize>());
                            start = last;
                            last = next;
                        }
                    }

                    new_path.push(last.cast::<isize>());

                    let mut path: Vec<_> = new_path
                        .into_iter()
                        .map(|point| {
                            self.costmap_ref
                                .local_to_global(point.into())
                                .cast::<Float>()
                        })
                        .collect();

                    *path.first_mut().unwrap() =
                        Point2::new(isometry.translation.x, isometry.translation.z);
                    *path.last_mut().unwrap() = dest;
                    // println!("{:?}", path.last().unwrap());

                    self.path_signal.set(path);

                    let elapsed = start_time.elapsed();
                    sleeper.sleep(self.refresh_rate.saturating_sub(elapsed));
                    start_time += elapsed;
                }

                completion_percentage.store(255, Ordering::Relaxed);
                pending_task.finish(Ok(()));
            }
        })
        .await
    }
}

#[inline]
fn traverse_to(from: Vector2<usize>, to: Vector2<usize>, obstacles: &DMatrix<bool>) -> bool {
    let mut travel = to.cast::<Float>() - from.cast();
    let distance = travel.magnitude();
    travel.unscale_mut(distance);

    for i in 0..distance.floor() as usize {
        let intermediate = from.cast() + travel * i as Float;

        if !*obstacles
            .get((
                intermediate.y.round() as usize,
                intermediate.x.round() as usize,
            ))
            .unwrap()
        {
            return false;
        }
    }

    true
}
