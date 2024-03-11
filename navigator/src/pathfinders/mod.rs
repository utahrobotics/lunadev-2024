use std::{
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
    time::Duration,
};

use nalgebra::{Point2, Point3};
use rig::RobotBaseRef;
use unros::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscription},
    service::{new_service, Pending, Service, ServiceHandle},
    setup_logging, tokio_rayon, Node, NodeIntrinsics, RuntimeContext,
};

use crate::Float;

mod global;
mod local;

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

pub struct DirectPathfinder<T: CostmapReference> {
    service_handle: NavigationServiceHandle,
    path_signal: Publisher<Arc<[Point2<Float>]>>,
    robot_base: RobotBaseRef,
    service:
        Service<Point3<Float>, NavigationError, NavigationProgress, Result<(), NavigationError>>,
    pub completion_distance: Float,
    costmap_ref: T,
    pub refresh_rate: Duration,
    pub agent_radius: Float,
    pub max_height_diff: Float,
    intrinsics: NodeIntrinsics<Self>,
}

impl<T: CostmapReference> DirectPathfinder<T> {
    pub fn new(
        robot_base: RobotBaseRef,
        costmap_ref: T,
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

    pub fn accept_path_sub(&mut self, sub: Subscription<Arc<[Point2<Float>]>>) {
        self.path_signal.accept_subscription(sub);
    }

    pub fn get_navigation_handle(&self) -> NavigationServiceHandle {
        self.service_handle.clone()
    }
}

#[async_trait]
impl<T: CostmapReference> Node for DirectPathfinder<T> {
    const DEFAULT_NAME: &'static str = "direct-pathfinder";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        tokio_rayon::spawn(move || loop {
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

            T::process(&mut self, dest, pending_task, completion_percentage);
        })
        .await
    }
}

pub trait CostmapReference: Send + 'static + Sized {
    fn process(
        node: &mut DirectPathfinder<Self>,
        dest: Point3<Float>,
        pending_task: Pending<Result<(), NavigationError>>,
        completion_percentage: Arc<AtomicU8>,
    );
}
