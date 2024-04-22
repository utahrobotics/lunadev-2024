use async_trait::async_trait;
use nalgebra::{convert as nconvert, Point3};
use unros::float::Float;

use crate::Shape;

pub mod depth;

pub trait HeightDistribution<N> {
    fn from_iter<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = Option<N>>;
}

pub struct HeightOnly<N: Float> {
    pub height: N,
    pub unknown: N,
}

pub struct HeightAndVariance<N: Float> {
    pub height: N,
    pub variance: N,
    pub unknown: N,
}

impl<N: Float> HeightDistribution<N> for HeightOnly<N> {
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

impl<N: Float> HeightDistribution<N> for HeightAndVariance<N> {
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

#[async_trait]
pub trait ObstacleSourceHeightGeneric<N: Float, H: HeightDistribution<N>> {
    async fn get_height_distr_within(&self, origin: Point3<N>, shape: Shape<N>) -> H
    where
        Self: Sized;
}

#[async_trait]
pub trait ObstacleSource<N: Float>:
    ObstacleSourceHeightGeneric<N, HeightOnly<N>> + ObstacleSourceHeightGeneric<N, HeightAndVariance<N>>
{
    async fn get_height_only_within(&self, origin: Point3<N>, shape: Shape<N>) -> HeightOnly<N>
    where
        Self: Sized,
    {
        self.get_height_distr_within(origin, shape).await
    }
    async fn get_height_and_variance_within(
        &self,
        origin: Point3<N>,
        shape: Shape<N>,
    ) -> HeightAndVariance<N>
    where
        Self: Sized,
    {
        self.get_height_distr_within(origin, shape).await
    }
}
