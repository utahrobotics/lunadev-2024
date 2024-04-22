use std::sync::{atomic::AtomicUsize, Arc};

use sources::ObstacleSource;
use unros::{float::Float, tokio::sync::RwLock};

pub mod sources;

#[non_exhaustive]
#[derive(Clone, Debug)]
pub enum Shape<N: Float> {
    Cylinder { radius: N, height: N },
    // Composite(Vec<Self>),
}

struct ObstacleHubInner<N: Float> {
    sources: RwLock<Vec<Box<dyn ObstacleSource<N>>>>,
    counter: AtomicUsize,
}

pub struct ObstacleHub<N: Float> {
    inner: Arc<ObstacleHubInner<N>>,
}

impl<N: Float> ObstacleHub<N> {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(ObstacleHubInner {
                sources: RwLock::new(Vec::new()),
                counter: AtomicUsize::new(0),
            }),
        }
    }
}
