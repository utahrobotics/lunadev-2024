use std::sync::{atomic::{AtomicU8, Ordering}, Arc};

use costmap::Costmap;
use nalgebra::{Point3, RealField};
use unros::{anyhow, async_trait, asyncify_run, pubsub::Subscriber, service::ServiceHandle, Node, NodeIntrinsics, RuntimeContext};

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

pub type NavigationServiceHandle<N: RealField> =
    ServiceHandle<Point3<N>, NavigationError, NavigationProgress, Result<(), NavigationError>>;

pub trait PathfindingEngine<N: RealField>: Send + 'static {
    fn pathfind(&mut self, start: Point3<N>, end: Point3<N>) -> Vec<Point3<N>>;
}


pub struct Pathfinder<N: RealField, E: PathfindingEngine<N>> {
    engine: E,
    costmap_sub: Subscriber<Costmap<N>>,
    intrinsics: NodeIntrinsics<Self>,

}


impl<N: RealField, E: PathfindingEngine<N>> Pathfinder<N, E> {
    pub fn new_with_engine(engine: E) -> Self {
        Self {
            engine,
            costmap_sub: Subscriber::new(1),
            intrinsics: Default::default()
        }
    }
}


#[async_trait]
impl<N: RealField, E: PathfindingEngine<N>> Node for Pathfinder<N, E> {
    const DEFAULT_NAME: &'static str = "pathfinder";

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()> {
        asyncify_run(move || {
            let costmap = self.costmap_sub.recv().await.ok_or_else(|| anyhow!("No costmap received"))?;
            let start = costmap.get_start();
            let end = costmap.get_end();
            let path = self.engine.pathfind(start, end);
            Ok(())
        }).await
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}