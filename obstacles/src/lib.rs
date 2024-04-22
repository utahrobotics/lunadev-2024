use std::sync::Arc;

use futures::{stream::FuturesUnordered, StreamExt};
use nalgebra::Point3;
use sources::{Busy, HeightAndVariance, HeightOnly, ObstacleSource};
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
}

pub struct ObstacleHub<N: Float> {
    inner: Arc<ObstacleHubInner<N>>,
}

impl<N: Float> ObstacleHub<N> {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(ObstacleHubInner {
                sources: RwLock::new(Vec::new()),
            }),
        }
    }

    pub async fn add_source(&self, source: impl ObstacleSource<N> + 'static) {
        self.inner.sources.write().await.push(Box::new(source));
    }

    pub async fn get_height_only_within(
        &self,
        origin: Point3<N>,
        shape: Shape<N>,
        mut callback: impl FnMut(Result<HeightOnly<N>, Busy>) -> bool
    ) {
        let sources = self.inner.sources.read().await;
        let mut futures = FuturesUnordered::new();
        for source in sources.iter() {
            futures.push(source.get_height_only_within(origin, shape.clone()));
        }
        while let Some(result) = futures.next().await {
            if !callback(result) {
                break;
            }
        }
    }
    pub async fn get_height_and_variance_within(
        &self,
        origin: Point3<N>,
        shape: Shape<N>,
        mut callback: impl FnMut(Result<HeightAndVariance<N>, Busy>) -> bool
    ) {
        let sources = self.inner.sources.read().await;
        let mut futures = FuturesUnordered::new();
        for source in sources.iter() {
            futures.push(source.get_height_and_variance_within(origin, shape.clone()));
        }
        while let Some(result) = futures.next().await {
            if !callback(result) {
                break;
            }
        }
    }
}
