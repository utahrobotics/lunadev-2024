use std::{
    future::Future, marker::PhantomData, sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    }, time::{Duration, Instant}
};

use nalgebra::{convert as nconvert, Point3, RealField, UnitQuaternion, Vector2, Vector3};
use obstacles::ObstacleHub;
use ordered_float::{FloatCore, NotNan};
use rig::RobotBaseRef;
use simba::scalar::SupersetOf;
use spin_sleep::SpinSleeper;
use unros::{
    float::Float, node::AsyncNode, pubsub::{subs::DirectSubscription, Publisher, PublisherRef, Subscriber, WatchSubscriber}, runtime::RuntimeContext, service::{new_service, Service, ServiceHandle}, setup_logging, tokio::{self, sync::oneshot}, DontDrop, ShouldNotDrop
};

mod alg;

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
        start: Point3<N>,
        end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        agent_radius: N,
        max_height_diff: N,
        context: &RuntimeContext,
    ) -> impl Future<Output=Option<Vec<Point3<N>>>>;
}

#[derive(ShouldNotDrop)]
pub struct Pathfinder<
    N: Float = f32,
    E: PathfindingEngine<N> = DirectPathfinder<N>,
> {
    engine: E,
    obstacle_hub: ObstacleHub<N>,
    dont_drop: DontDrop<Self>,
    service: Service<Point3<N>, NavigationError, NavigationProgress, Result<(), NavigationError>>,
    service_handle: NavigationServiceHandle<N>,
    robot_base: RobotBaseRef,
    path_pub: Publisher<Arc<[Point3<N>]>>,
    completion_distance: N,
    pub resolution: N,
    pub agent_radius: N,
    pub max_height_diff: N,
    pub refresh_rate: Duration,
}

impl<N: Float, E: PathfindingEngine<N>> Pathfinder<N, E> {
    pub fn new_with_engine(agent_radius: N, engine: E, obstacle_hub: ObstacleHub<N>, robot_base: RobotBaseRef) -> Self {
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
            resolution: agent_radius,
            agent_radius,
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

impl<
        N: Float,
        E: PathfindingEngine<N>,
    > AsyncNode for Pathfinder<N, E>
{
    type Result = ();

    async fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        drop(self.service_handle);
        self.dont_drop.ignore_drop = true;
        let mut start_time = Instant::now();
        let sleeper = SpinSleeper::default();

        loop {
            let Some(mut req) = self.service.blocking_wait_for_request() else {
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
                let mut start: Vector3<N> =
                    nalgebra::convert(self.robot_base.get_isometry().translation.vector);

                if let Some(path) = self.engine.pathfind(
                    start.into(),
                    end,
                    &self.obstacle_hub,
                    self.resolution,
                    self.agent_radius,
                    self.max_height_diff,
                    &context,
                ).await {
                    let path: Arc<[Point3<N>]> = Arc::from(path.into_boxed_slice());
                    self.path_pub.set(path.clone());

                    'repathfind: loop {
                        if context.is_runtime_exiting() {
                            return;
                        }
                        start =
                            nalgebra::convert(self.robot_base.get_isometry().translation.vector);

                        if (end.coords - start).magnitude() <= self.completion_distance {
                            pending_task.finish(Ok(()));
                            break 'main;
                        }
                        for [from, to] in path.array_windows::<2>() {
                            let relative = start - from.coords;
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

                            if !traverse_to(
                                from.coords,
                                to.coords,
                                self.agent_radius,
                                self.max_height_diff,
                                &self.obstacle_hub,
                                self.resolution,
                            ) {
                                break 'repathfind;
                            }
                        }
                        sleeper.sleep(self.refresh_rate);
                    }
                };
                // pending_task.finish(Err(NavigationError::NoPath));
            }
        }
    }
}

#[derive(Default)]
pub struct DirectPathfinder<N> {
    phantom: PhantomData<N>,
}

impl<N: Float> PathfindingEngine<N> for DirectPathfinder<N>
{
    fn pathfind(
        &mut self,
        start: Point3<N>,
        end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        agent_radius: N,
        max_height_diff: N,
        context: &RuntimeContext,
    ) -> Option<Vec<Point3<N>>> {
        setup_logging!(context);
        let mut start = start.coords;
        let mut pre_path = vec![];

        if !costmap.is_global_point_safe(start.into(), agent_radius, max_height_diff) {
            if let Some(mut path) = astar(
                &CVec3 {
                    inner: start,
                    resolution,
                },
                |current| {
                    let current = *current;
                    [
                        Vector3::new(-resolution, N::zero(), N::zero()),
                        Vector3::new(resolution, N::zero(), N::zero()),
                        Vector3::new(N::zero(), N::zero(), -resolution),
                        Vector3::new(N::zero(), N::zero(), resolution),
                    ]
                    .into_iter()
                    .filter_map(move |mut next| {
                        next += current.inner;
                        if costmap.is_global_point_safe(next.into(), agent_radius, max_height_diff)
                        {
                            Some(CVec3 {
                                inner: next,
                                resolution,
                            })
                        } else {
                            None
                        }
                    })
                },
                |current| {
                    costmap.is_global_point_safe(
                        current.inner.into(),
                        agent_radius,
                        max_height_diff,
                    )
                },
            ) {
                start = path.pop().unwrap().inner;
                pre_path = path;
            }
        }

        let start_time = Instant::now();
        let mut too_long = false;
        let mut out_of_bounds = false;

        let result = astar(
            &CVec3 {
                inner: start,
                resolution,
            },
            |current| {
                if !too_long && start_time.elapsed().as_secs() >= 2 {
                    too_long = true;
                }
                let current = *current;
                [
                    Vector3::new(-resolution, N::zero(), N::zero()),
                    Vector3::new(resolution, N::zero(), N::zero()),
                    Vector3::new(N::zero(), N::zero(), -resolution),
                    Vector3::new(N::zero(), N::zero(), resolution),
                    // Vector3::new(-resolution, N::zero(), resolution) * nalgebra::convert::<_, N>(FRAC_1_SQRT_2),
                    // Vector3::new(resolution, N::zero(), resolution) * nalgebra::convert::<_, N>(FRAC_1_SQRT_2),
                    // Vector3::new(resolution, N::zero(), -resolution) * nalgebra::convert::<_, N>(FRAC_1_SQRT_2),
                    // Vector3::new(-resolution, N::zero(), -resolution) * nalgebra::convert::<_, N>(FRAC_1_SQRT_2),
                ]
                .into_iter()
                .filter(move |_| !too_long)
                .filter_map(move |mut next| {
                    next += current.inner;
                    // println!("{next:?} {}", (next - end.coords).magnitude());
                    // std::thread::sleep(std::time::Duration::from_millis(100));
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
                .chain(std::iter::once(()).filter_map(move |()| {
                    let mut diff = current.inner - end.coords;
                    diff.y = N::zero();
                    let distance = diff.magnitude();
                    if distance <= resolution {
                        Some((
                            CVec3 {
                                inner: end.coords,
                                resolution,
                            },
                            (distance / resolution).round().to_subset_unchecked(),
                        ))
                    } else {
                        None
                    }
                }))
            },
            |current| {
                let mut diff = current.inner - end.coords;
                diff.y = N::zero();
                let cost: usize = (diff.magnitude() / resolution)
                    .round()
                    .to_subset_unchecked();
                cost / 2
            },
            |current| {
                if current
                    == &(CVec3 {
                        inner: end.coords,
                        resolution,
                    })
                {
                    true
                } else if !costmap.is_global_point_in_bounds(current.inner.into()) {
                    out_of_bounds = true;
                    true
                } else {
                    false
                }
            },
        );

        if too_long {
            error!("Pathfinding took too long");
        }

        let (mut post_path, _distance) = result?;

        if post_path.len() == 1 {
            post_path.push(CVec3 {
                inner: end.coords,
                resolution,
            });
        }

        let mut path = pre_path;
        path.append(&mut post_path);

        let mut new_path: Vec<Point3<N>> = Vec::with_capacity(path.len());
        let mut path = path.into_iter().map(|x| x.inner.into());

        let mut start = path.next().unwrap();
        new_path.push(start);
        let mut last = path.next().unwrap();

        for next in path {
            if traverse_to(
                start.coords,
                next.coords,
                agent_radius,
                max_height_diff,
                &costmap,
                resolution,
            ) {
                last = next;
            } else {
                new_path.push(last);
                start = last;
                last = next;
            }
        }

        new_path.push(end);

        Some(new_path)
    }
}

#[inline]
fn traverse_to<N>(
    from: Vector3<N>,
    to: Vector3<N>,
    agent_radius: N,
    max_height_diff: N,
    obstacle_hub: &ObstacleHub<N>,
    resolution: N,
) -> bool
where
    N: RealField
        + SupersetOf<usize>
        + SupersetOf<f32>
        + SupersetOf<i64>
        + SupersetOf<isize>
        + SupersetOf<u8>
        + Copy,
{
    if !costmap.is_global_point_in_bounds(to.into()) {
        return true;
    }
    let mut travel = to - from;
    let distance = travel.magnitude();
    travel.unscale_mut(distance);

    let count: usize = (distance / resolution).floor().to_subset_unchecked();

    for i in 1..count {
        let intermediate: Vector3<N> = from + travel * nalgebra::convert::<_, N>(i);

        if !costmap.is_global_point_safe(intermediate.into(), agent_radius, max_height_diff) {
            return false;
        }
    }

    true
}

#[derive(Clone, Copy, Debug)]
struct CVec3<T: RealField> {
    inner: Vector3<T>,
    resolution: T,
}

impl<T: RealField + SupersetOf<i64> + Copy> PartialEq for CVec3<T> {
    fn eq(&self, other: &Self) -> bool {
        let self_x: i64 = (self.inner.x / self.resolution).to_subset_unchecked();
        let self_z: i64 = (self.inner.z / self.resolution).to_subset_unchecked();
        let other_x: i64 = (other.inner.x / self.resolution).to_subset_unchecked();
        let other_z: i64 = (other.inner.z / self.resolution).to_subset_unchecked();

        self_x == other_x && self_z == other_z
    }
}

impl<T: RealField + SupersetOf<i64> + Copy> std::hash::Hash for CVec3<T> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_i64((self.inner.x / self.resolution).to_subset_unchecked());
        state.write_i64((self.inner.z / self.resolution).to_subset_unchecked());
    }
}

impl<T: RealField + SupersetOf<i64> + Copy> Eq for CVec3<T> {}
