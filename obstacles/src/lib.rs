use std::sync::Arc;

use async_trait::async_trait;
use bitvec::vec::BitVec;
use futures::{stream::FuturesUnordered, Future, StreamExt};
use nalgebra::{convert as nconvert, Isometry3};
use unros::{
    float::Float,
    tokio::{runtime::Handle, sync::RwLock},
};
use utils::RecycledVec;

pub mod sources;
pub mod utils;

#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub enum Shape<N: Float> {
    Cylinder { radius: N, height: N },
}

impl<N: Float> Shape<N> {
    pub fn scale_mut(&mut self, scale: N) {
        match self {
            Self::Cylinder { radius, height } => {
                *radius *= scale;
                *height *= scale;
            }
        }
    }

    pub fn scale(mut self, scale: N) -> Self {
        self.scale_mut(scale);
        self
    }
}

struct ObstacleHubInner<N: Float> {
    sources: RwLock<Vec<Arc<dyn HeightMap<N>>>>,
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
        self.inner.sources.write().await.push(Arc::new(source));
    }

    pub fn add_source_mut<T: HeightMap<N> + 'static>(
        &mut self,
        source: T,
    ) -> Result<(), AddSourceMutError<T>> {
        match Arc::get_mut(&mut self.inner) {
            Some(inner) => {
                inner.sources.get_mut().push(Arc::new(source));
                Ok(())
            }
            None => Err(AddSourceMutError(source)),
        }
    }
}

impl<N: Float> ObstacleHub<N> {
    pub async fn query_height<'a>(
        &'a self,
        queries: impl IntoIterator<Item = HeightQuery<N>>,
    ) -> PendingHeightQueries<
        'a,
        N,
        impl Future<
            Output = (
                usize,
                Option<(
                    RecycledVec<RecycledVec<N>>,
                    Arc<RecycledVec<HeightQuery<N>>>,
                )>,
            ),
        >,
    >
    where
        RecycledVec<HeightQuery<N>>: Default,
    {
        let shapes: RecycledVec<HeightQuery<N>> = queries.into_iter().collect();
        if shapes.is_empty() {
            return PendingHeightQueries {
                hub: self,
                futures: FuturesUnordered::new(),
                indices_to_remove: vec![],
            };
        }
        let shapes = Arc::new(shapes);

        let sources = self.inner.sources.read().await;
        let futures = FuturesUnordered::new();
        for (i, source) in sources.iter().cloned().enumerate() {
            let shapes = shapes.clone();
            futures.push(async move {
                (
                    i,
                    source
                        .query_height(shapes.clone())
                        .await
                        .map(|value| (value, shapes)),
                )
            });
        }

        PendingHeightQueries {
            hub: self,
            futures,
            indices_to_remove: vec![],
        }
    }

    pub async fn safe_by_height(
        &self,
        queries: impl IntoIterator<Item = HeightQuery<N>>,
        current_height: N,
        max_difference: N,
        min_fraction: N,
    ) -> BitVec
    where
        RecycledVec<HeightQuery<N>>: Default,
    {
        let mut bools = BitVec::new();
        let mut pending = self.query_height(queries).await;

        'main: while let Some((mut vec_of_heights, queries)) = pending.next_with_queries().await {
            for (mut heights, query) in vec_of_heights.drain(..).zip(queries.iter()) {
                let too_high_count = heights
                    .drain(..)
                    .filter(|&height| (height - current_height).abs() > max_difference)
                    .count();
                if nconvert::<_, N>(too_high_count)
                    > nconvert::<_, N>(query.max_points) * min_fraction
                {
                    bools.push(false);
                    continue 'main;
                }
            }
            bools.push(true);
        }

        bools
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

pub struct PendingHeightQueries<'a, N: Float, F> {
    hub: &'a ObstacleHub<N>,
    futures: FuturesUnordered<F>,
    indices_to_remove: Vec<usize>,
}

impl<
        'a,
        N: Float,
        F: Future<
            Output = (
                usize,
                Option<(
                    RecycledVec<RecycledVec<N>>,
                    Arc<RecycledVec<HeightQuery<N>>>,
                )>,
            ),
        >,
    > PendingHeightQueries<'a, N, F>
{
    pub async fn next_with_queries(
        &mut self,
    ) -> Option<(
        RecycledVec<RecycledVec<N>>,
        Arc<RecycledVec<HeightQuery<N>>>,
    )> {
        while let Some((i, option)) = self.futures.next().await {
            if let Some(value) = option {
                return Some(value);
            } else {
                self.indices_to_remove.push(i);
            }
        }
        None
    }
    pub async fn next(&mut self) -> Option<RecycledVec<RecycledVec<N>>> {
        self.next_with_queries().await.map(|(value, _)| value)
    }
}

impl<'a, N: Float, F> Drop for PendingHeightQueries<'a, N, F> {
    fn drop(&mut self) {
        if self.indices_to_remove.is_empty() {
            return;
        }
        let Ok(handle) = Handle::try_current() else {
            return;
        };

        self.indices_to_remove.sort_unstable_by(|a, b| b.cmp(a));
        let indices_to_remove = std::mem::take(&mut self.indices_to_remove);
        let inner = self.hub.inner.clone();
        handle.spawn(async move {
            let mut sources = inner.sources.write().await;
            for i in indices_to_remove {
                sources.swap_remove(i);
            }
        });
    }
}
