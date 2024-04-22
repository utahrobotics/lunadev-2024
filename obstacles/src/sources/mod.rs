use async_trait::async_trait;
use nalgebra::convert as nconvert;
use unros::float::Float;

use crate::Shape;

pub mod depth;

pub struct HeightOnly<N: Float> {
    pub height: N,
    pub unknown: N,
}

pub struct HeightAndVariance<N: Float> {
    pub height: N,
    pub variance: N,
    pub unknown: N,
}

impl<N: Float> HeightOnly<N> {
    fn from_iter<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = Option<N>>,
    {
        let mut count = 0usize;
        let mut unknown_count = 0usize;
        let mut height = iter
            .into_iter()
            .filter_map(|x| {
                if x.is_none() {
                    unknown_count += 1;
                }
                x
            })
            .map(|x| {
                count += 1;
                x
            })
            .sum();
        height /= nconvert(count);

        Self {
            height,
            unknown: nconvert::<_, N>(unknown_count) / nconvert(count),
        }
    }
}

impl<N: Float> HeightAndVariance<N> {
    fn from_iter<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = Option<N>>,
    {
        let mut count = 0usize;
        let mut unknown_count = 0usize;
        let mut heights = vec![];
        let mut height = iter
            .into_iter()
            .filter_map(|x| {
                if x.is_none() {
                    unknown_count += 1;
                }
                x
            })
            .map(|x| {
                count += 1;
                heights.push(x);
                x
            })
            .sum();
        height /= nconvert(count);

        Self {
            height,
            variance: heights
                .into_iter()
                .map(|x| {
                    let diff = x - height;
                    diff * diff
                })
                .sum::<N>()
                / nconvert(count - 1),
            unknown: nconvert::<_, N>(unknown_count) / nconvert(count),
        }
    }
}

pub struct Busy;

#[async_trait]
pub trait ObstacleSource<N: Float>: Send + Sync {
    async fn get_height_only_within(&self, shape: Shape<N>) -> Result<HeightOnly<N>, Busy>;
    async fn get_height_and_variance_within(
        &self,
        shape: Shape<N>,
    ) -> Result<HeightAndVariance<N>, Busy>;
}
