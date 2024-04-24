use std::sync::Arc;

use futures::{stream::FuturesUnordered, StreamExt};
use nalgebra::{Isometry3, Translation3};
use sources::{HeightAndVariance, HeightOnly, ObstacleSource};
use unros::{float::Float, tokio::sync::RwLock};

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

impl<N: Float> Shape<N> {
    pub fn set_origin(&mut self, origin: impl Into<Translation3<N>>) {
        match self {
            Self::Cylinder { isometry, .. } => {
                isometry.translation = origin.into();
            }
        }
    }
}

struct ObstacleHubInner<N: Float> {
    sources: RwLock<Vec<Box<dyn ObstacleSource<N>>>>,
}

pub struct ObstacleHub<N: Float> {
    inner: Arc<ObstacleHubInner<N>>,
}

impl<N: Float> Default for ObstacleHub<N> {
    fn default() -> Self {
        Self {
            inner: Arc::new(ObstacleHubInner {
                sources: RwLock::new(Vec::new()),
            }),
        }
    }
}

impl<N: Float> Clone for ObstacleHub<N> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

pub struct AddSourceMutError<T>(pub T);

impl<T> std::fmt::Debug for AddSourceMutError<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("AddSourceMutError").finish()
    }
}

impl<T> std::fmt::Display for AddSourceMutError<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "could not add source as ObstacleHub is shared")
    }
}

impl<N: Float> ObstacleHub<N> {
    pub async fn add_source(&self, source: impl ObstacleSource<N> + 'static) {
        self.inner.sources.write().await.push(Box::new(source));
    }

    pub fn add_source_mut<T: ObstacleSource<N> + 'static>(
        &mut self,
        source: T,
    ) -> Result<(), AddSourceMutError<T>> {
        match Arc::get_mut(&mut self.inner) {
            Some(inner) => {
                inner.sources.get_mut().push(Box::new(source));
                Ok(())
            }
            None => Err(AddSourceMutError(source)),
        }
    }

    pub async fn get_height_only_within<T>(
        &self,
        shape: &Shape<N>,
        mut filter: impl FnMut(HeightOnly<N>) -> Option<T>,
    ) -> Option<T> {
        let mut indices_to_remove = Vec::new();
        let result = 'check: {
            let sources = self.inner.sources.read().await;
            let mut futures = FuturesUnordered::new();
            for (i, source) in sources.iter().enumerate() {
                futures.push(async move { (i, source.get_height_only_within(shape).await) });
            }
            while let Some((i, result)) = futures.next().await {
                let Some(value) = result else {
                    indices_to_remove.push(i);
                    continue;
                };
                if let Some(result) = filter(value) {
                    break 'check Some(result);
                }
            }
            None
        };

        if indices_to_remove.is_empty() {
            return result;
        }
        indices_to_remove.sort_unstable_by(|a, b| b.cmp(a));
        let mut sources = self.inner.sources.write().await;
        for i in indices_to_remove {
            sources.swap_remove(i);
        }
        result
    }
    pub async fn get_height_and_variance_within<T>(
        &self,
        shape: &Shape<N>,
        mut filter: impl FnMut(HeightAndVariance<N>) -> Option<T>,
    ) -> Option<T> {
        let mut indices_to_remove = Vec::new();
        let result = 'check: {
            let sources = self.inner.sources.read().await;
            let mut futures = FuturesUnordered::new();
            for (i, source) in sources.iter().enumerate() {
                futures
                    .push(async move { (i, source.get_height_and_variance_within(shape).await) });
            }
            while let Some((i, result)) = futures.next().await {
                let Some(value) = result else {
                    indices_to_remove.push(i);
                    continue;
                };
                if let Some(result) = filter(value) {
                    break 'check Some(result);
                }
            }
            None
        };

        if indices_to_remove.is_empty() {
            return result;
        }
        indices_to_remove.sort_unstable_by(|a, b| b.cmp(a));
        let mut sources = self.inner.sources.write().await;
        for i in indices_to_remove {
            sources.swap_remove(i);
        }
        result
    }
}
