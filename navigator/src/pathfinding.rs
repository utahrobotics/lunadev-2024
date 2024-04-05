use std::{
    convert::FloatToInt,
    f32::consts::FRAC_1_SQRT_2,
    marker::PhantomData,
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use costmap::Costmap;
use nalgebra::{Point3, RealField, Vector3};
use ordered_float::NotNan;
use rig::RobotBaseRef;
use simba::scalar::SupersetOf;
use unros::{
    anyhow, async_trait, asyncify_run,
    pubsub::{Publisher, Subscriber, WatchSubscriber},
    service::{new_service, Service, ServiceHandle},
    setup_logging, Node, NodeIntrinsics, RuntimeContext,
};

use crate::utils::astar;

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

pub struct CompletionPercentage<'a> {
    completion_percentage: &'a AtomicU8,
}

impl<'a> CompletionPercentage<'a> {
    pub fn set_completion_percentage<N: RealField + FloatToInt<u8> + SupersetOf<u8>>(
        &self,
        percentage: NotNan<N>,
    ) {
        let percentage = percentage.into_inner() * nalgebra::convert(255.0);
        let percentage = unsafe { percentage.to_int_unchecked() };
        self.completion_percentage
            .store(percentage, Ordering::Relaxed);
    }
}

pub trait PathfindingEngine<N: RealField>: Send + 'static {
    fn pathfind(
        &mut self,
        start: Point3<N>,
        end: Point3<N>,
        costmap: &Costmap<N>,
        resolution: N,
        agent_radius: N,
        max_height_diff: N,
    ) -> Option<Vec<Point3<N>>>;
}

pub struct Pathfinder<E: PathfindingEngine<N>, N: RealField + SupersetOf<f32> + Copy = f32> {
    engine: E,
    costmap_sub: Subscriber<Costmap<N>>,
    intrinsics: NodeIntrinsics<Self>,
    service: Service<Point3<N>, NavigationError, NavigationProgress, Result<(), NavigationError>>,
    service_handle: NavigationServiceHandle<N>,
    robot_base: RobotBaseRef,
    path_pub: Publisher<Arc<[Point3<N>]>>,
    completion_distance: N,
    pub refresh_rate: Duration,
    pub resolution: N,
    pub agent_radius: N,
    pub max_height_diff: N,
}

impl<N: RealField + SupersetOf<f32> + Copy, E: PathfindingEngine<N>> Pathfinder<E, N> {
    pub fn new_with_engine(engine: E, robot_base: RobotBaseRef) -> Self {
        let (service, service_handle) = new_service();
        Self {
            engine,
            costmap_sub: Subscriber::new(1),
            intrinsics: Default::default(),
            service,
            service_handle,
            robot_base,
            path_pub: Publisher::default(),
            completion_distance: nalgebra::convert(0.15),
            refresh_rate: Duration::from_millis(50),
            resolution: nalgebra::convert(0.1),
            agent_radius: nalgebra::convert(0.3),
            max_height_diff: nalgebra::convert(0.15),
        }
    }
}

#[async_trait]
impl<N: RealField + SupersetOf<f32> + Copy, E: PathfindingEngine<N>> Node for Pathfinder<E, N> {
    const DEFAULT_NAME: &'static str = "pathfinder";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        drop(self.service_handle);
        let Ok(mut costmap) = self.costmap_sub.into_watch_or_closed().await else {
            return Ok(());
        };
        let sleeper = spin_sleep::SpinSleeper::default();
        let mut start_time = Instant::now();

        asyncify_run(move || loop {
            let Some(mut req) = self.service.blocking_wait_for_request() else {
                break Ok(());
            };

            let end = req.take_input().unwrap();
            let completion_percentage = Arc::new(AtomicU8::default());

            let Some(pending_task) = req.accept(NavigationProgress {
                completion_percentage: completion_percentage.clone(),
            }) else {
                error!("Scheduler of task dropped task init before we could respond");
                continue;
            };

            start_time += start_time.elapsed();

            loop {
                let start: Vector3<N> =
                    nalgebra::convert(self.robot_base.get_isometry().translation.vector);

                if (end.coords - start).magnitude() <= self.completion_distance {
                    pending_task.finish(Ok(()));
                    break;
                }

                WatchSubscriber::try_update(&mut costmap);
                let Some(path) = self.engine.pathfind(
                    start.into(),
                    end,
                    &costmap,
                    self.resolution,
                    self.agent_radius,
                    self.max_height_diff,
                ) else {
                    pending_task.finish(Err(NavigationError::NoPath));
                    break;
                };

                self.path_pub.set(path.into_boxed_slice().into());
                sleeper.sleep(self.refresh_rate.saturating_sub(start_time.elapsed()));
            }
        })
        .await
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}

pub struct DirectPathfinder<N> {
    phantom: PhantomData<N>,
}

impl<
        N: RealField
            + Copy
            + FloatToInt<isize>
            + FloatToInt<usize>
            + FloatToInt<u8>
            + SupersetOf<usize>
            + SupersetOf<isize>
            + SupersetOf<i64>
            + SupersetOf<f32>
            + std::hash::Hash,
    > PathfindingEngine<N> for DirectPathfinder<N>
{
    fn pathfind(
        &mut self,
        start: Point3<N>,
        end: Point3<N>,
        costmap: &Costmap<N>,
        resolution: N,
        agent_radius: N,
        max_height_diff: N,
    ) -> Option<Vec<Point3<N>>> {
        let mut start = start.coords;

        if !costmap.is_global_point_safe(start.into(), agent_radius, max_height_diff) {
            let (path, _) = astar(
                &CVec3 {
                    inner: start,
                    resolution,
                },
                |current| {
                    [
                        Vector3::new(-resolution, N::zero(), N::zero()) + current.inner,
                        Vector3::new(resolution, N::zero(), N::zero()) + current.inner,
                        Vector3::new(N::zero(), N::zero(), -resolution) + current.inner,
                        Vector3::new(N::zero(), N::zero(), resolution) + current.inner,
                        nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                            -FRAC_1_SQRT_2,
                            0.0,
                            FRAC_1_SQRT_2,
                        )) * resolution
                            + current.inner,
                        nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                            FRAC_1_SQRT_2,
                            0.0,
                            FRAC_1_SQRT_2,
                        )) * resolution
                            + current.inner,
                        nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                            FRAC_1_SQRT_2,
                            0.0,
                            -FRAC_1_SQRT_2,
                        )) * resolution
                            + current.inner,
                        nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                            -FRAC_1_SQRT_2,
                            0.0,
                            -FRAC_1_SQRT_2,
                        )) * resolution
                            + current.inner,
                    ]
                    .into_iter()
                    .filter_map(|next| {
                        if costmap.is_global_point_safe(next.into(), agent_radius, max_height_diff)
                        {
                            Some((
                                CVec3 {
                                    inner: next,
                                    resolution,
                                },
                                1usize,
                            ))
                        } else {
                            None
                        }
                    })
                },
                |current| {
                    let diff = current.inner - end.coords;
                    unsafe { (diff.magnitude() / resolution).round().to_int_unchecked() }
                },
                |current| current.inner == end.coords,
            );
            start = path.into_iter().last().unwrap().inner;
        }

        let (path, _distance) = astar(
            // In local space, our start position is always (0, 0)
            &CVec3 {
                inner: start,
                resolution,
            },
            |current| {
                [
                    Vector3::new(-resolution, N::zero(), N::zero()) + current.inner,
                    Vector3::new(resolution, N::zero(), N::zero()) + current.inner,
                    Vector3::new(N::zero(), N::zero(), -resolution) + current.inner,
                    Vector3::new(N::zero(), N::zero(), resolution) + current.inner,
                    nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                        -FRAC_1_SQRT_2,
                        0.0,
                        FRAC_1_SQRT_2,
                    )) * resolution
                        + current.inner,
                    nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                        FRAC_1_SQRT_2,
                        0.0,
                        FRAC_1_SQRT_2,
                    )) * resolution
                        + current.inner,
                    nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                        FRAC_1_SQRT_2,
                        0.0,
                        -FRAC_1_SQRT_2,
                    )) * resolution
                        + current.inner,
                    nalgebra::convert::<_, Vector3<N>>(Vector3::new(
                        -FRAC_1_SQRT_2,
                        0.0,
                        -FRAC_1_SQRT_2,
                    )) * resolution
                        + current.inner,
                ]
                .into_iter()
                .filter_map(|next| {
                    if costmap.is_global_point_safe(next.into(), agent_radius, max_height_diff) {
                        Some((
                            CVec3 {
                                inner: next,
                                resolution,
                            },
                            1usize,
                        ))
                    } else {
                        None
                    }
                })
            },
            |current| {
                let diff = current.inner - end.coords;
                unsafe { (diff.magnitude() / resolution).round().to_int_unchecked() }
            },
            |current| current.inner == end.coords,
        );

        Some(path.into_iter().map(|x| x.inner.into()).collect())
    }
}

#[derive(Hash, Clone, Copy, Debug)]
struct CVec3<T: RealField> {
    inner: Vector3<T>,
    resolution: T,
}

impl<T: RealField + SupersetOf<f32> + Copy> PartialEq for CVec3<T> {
    fn eq(&self, other: &Self) -> bool {
        let diff = self.inner - other.inner;
        diff.magnitude() < self.resolution * nalgebra::convert(0.5)
    }
}

impl<T: RealField + SupersetOf<f32> + Copy> Eq for CVec3<T> {}
