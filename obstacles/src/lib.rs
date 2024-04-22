use std::sync::Arc;

use futures::{stream::FuturesUnordered, StreamExt};
use nalgebra::Isometry3;
use sources::{HeightAndVariance, HeightOnly, ObstacleSource};
use unros::{
    float::Float,
    tokio::sync::RwLock,
};

pub mod sources;

#[non_exhaustive]
#[derive(Clone, Debug)]
pub enum Shape<N: Float> {
    Cylinder {
        radius: N,
        height: N,
        isometry: Isometry3<N>,
    },
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
        shape: Shape<N>,
        mut callback: impl FnMut(HeightOnly<N>) -> bool,
    ) {
        let mut indices_to_remove = Vec::new();
        {
            let sources = self.inner.sources.read().await;
            let mut futures = FuturesUnordered::new();
            for (i, source) in sources.iter().enumerate() {
                let shape = shape.clone();
                futures.push(async move { (i, source.get_height_only_within(shape).await) });
            }
            while let Some((i, result)) = futures.next().await {
                let Some(value) = result else {
                    indices_to_remove.push(i);
                    continue;
                };
                if !callback(value) {
                    break;
                }
            }
        }

        if indices_to_remove.is_empty() {
            return;
        }
        indices_to_remove.sort_unstable_by(|a, b| b.cmp(a));
        let mut sources = self.inner.sources.write().await;
        for i in indices_to_remove {
            sources.swap_remove(i);
        }
    }
    pub async fn get_height_and_variance_within(
        &self,
        shape: Shape<N>,
        mut callback: impl FnMut(HeightAndVariance<N>) -> bool,
    ) {
        let mut indices_to_remove = Vec::new();
        {
            let sources = self.inner.sources.read().await;
            let mut futures = FuturesUnordered::new();
            for (i, source) in sources.iter().enumerate() {
                let shape = shape.clone();
                futures
                    .push(async move { (i, source.get_height_and_variance_within(shape).await) });
            }
            while let Some((i, result)) = futures.next().await {
                let Some(value) = result else {
                    indices_to_remove.push(i);
                    continue;
                };
                if !callback(value) {
                    break;
                }
            }
        }

        if indices_to_remove.is_empty() {
            return;
        }
        indices_to_remove.sort_unstable_by(|a, b| b.cmp(a));
        let mut sources = self.inner.sources.write().await;
        for i in indices_to_remove {
            sources.swap_remove(i);
        }
    }
}
