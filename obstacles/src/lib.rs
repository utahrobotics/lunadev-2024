use std::sync::Arc;

use async_trait::async_trait;
use futures::{stream::FuturesUnordered, Future, StreamExt};
use nalgebra::Isometry3;
use unros::{float::Float, tokio::sync::RwLock};
use utils::RecycledVec;

pub mod sources;
pub mod utils;

#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub enum Shape<N: Float> {
    Cylinder { radius: N, height: N },
}

// impl<N: Float> Shape<N> {
//     pub fn set_origin(&mut self, origin: impl Into<Translation3<N>>) {
//         match self {
//             Self::Cylinder { isometry, .. } => {
//                 isometry.translation = origin.into();
//             }
//         }
//     }
// }

struct ObstacleHubInner<N: Float> {
    sources: RwLock<Vec<Box<dyn HeightMap<N>>>>,
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
    pub async fn add_source(&self, source: impl HeightMap<N> + 'static) {
        self.inner.sources.write().await.push(Box::new(source));
    }

    pub fn add_source_mut<T: HeightMap<N> + 'static>(
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
}

impl<N: Float> ObstacleHub<N> {
    pub async fn query_height<T, F: Future<Output = Option<T>>>(
        &self,
        queries: impl IntoIterator<Item = HeightQuery<N>>,
        mut filter: impl FnMut(RecycledVec<RecycledVec<N>>) -> F,
    ) -> Option<T>
    where
        RecycledVec<HeightQuery<N>>: Default,
    {
        let shapes: RecycledVec<HeightQuery<N>> = queries.into_iter().collect();
        let shapes = Arc::new(shapes);
        let mut indices_to_remove = Vec::new();
        let result = 'check: {
            let sources = self.inner.sources.read().await;
            let mut futures = FuturesUnordered::new();
            for (i, source) in sources.iter().enumerate() {
                let fut = source.query_height(shapes.clone());
                futures.push(async move { (i, fut.await) });
            }
            while let Some((i, result)) = futures.next().await {
                let Some(value) = result else {
                    indices_to_remove.push(i);
                    continue;
                };
                if let Some(result) = filter(value).await {
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

#[derive(Clone, Copy, Debug)]
pub struct HeightQuery<N: Float> {
    pub max_points: usize,
    pub shape: Shape<N>,
    pub isometry: Isometry3<N>,
}

#[async_trait]
pub trait HeightMap<N: Float>: Send + Sync {
    async fn query_height<'a>(
        &self,
        queries: Arc<RecycledVec<HeightQuery<N>>>,
    ) -> Option<RecycledVec<RecycledVec<N>>>;
}
