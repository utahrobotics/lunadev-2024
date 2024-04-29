use std::ops::{Deref, DerefMut};

use unros::utils::{ResourceGuard, ResourceQueue};

use crate::HeightQuery;

static VEC_FLOAT_QUEUE: ResourceQueue<Vec<f32>> = ResourceQueue::new(16, Vec::new);
static VEC_DOUBLE_QUEUE: ResourceQueue<Vec<f64>> = ResourceQueue::new(16, Vec::new);
static VEC_FLOAT_QUERY_QUEUE: ResourceQueue<Vec<HeightQuery<f32>>> =
    ResourceQueue::new(16, Vec::new);
static VEC_DOUBLE_QUERY_QUEUE: ResourceQueue<Vec<HeightQuery<f64>>> =
    ResourceQueue::new(16, Vec::new);
static VEC_VEC_FLOAT_QUEUE: ResourceQueue<Vec<RecycledVec<f32>>> = ResourceQueue::new(16, Vec::new);
static VEC_VEC_DOUBLE_QUEUE: ResourceQueue<Vec<RecycledVec<f64>>> =
    ResourceQueue::new(16, Vec::new);

pub struct RecycledVec<T: 'static> {
    inner: ResourceGuard<'static, Vec<T>>,
}

impl Default for RecycledVec<f32> {
    fn default() -> Self {
        let mut inner = VEC_FLOAT_QUEUE.get();
        inner.clear();
        Self { inner }
    }
}

impl Default for RecycledVec<f64> {
    fn default() -> Self {
        let mut inner = VEC_DOUBLE_QUEUE.get();
        inner.clear();
        Self { inner }
    }
}

impl Default for RecycledVec<HeightQuery<f32>> {
    fn default() -> Self {
        let mut inner = VEC_FLOAT_QUERY_QUEUE.get();
        inner.clear();
        Self { inner }
    }
}

impl Default for RecycledVec<HeightQuery<f64>> {
    fn default() -> Self {
        let mut inner = VEC_DOUBLE_QUERY_QUEUE.get();
        inner.clear();
        Self { inner }
    }
}

impl Default for RecycledVec<RecycledVec<f32>> {
    fn default() -> Self {
        let mut inner = VEC_VEC_FLOAT_QUEUE.get();
        inner.clear();
        Self { inner }
    }
}

impl Default for RecycledVec<RecycledVec<f64>> {
    fn default() -> Self {
        let mut inner = VEC_VEC_DOUBLE_QUEUE.get();
        inner.clear();
        Self { inner }
    }
}

impl<T: 'static> Deref for RecycledVec<T> {
    type Target = Vec<T>;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: 'static> DerefMut for RecycledVec<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: 'static> Extend<T> for RecycledVec<T> {
    fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) {
        self.inner.extend(iter);
    }
}

impl<T: 'static> FromIterator<T> for RecycledVec<T>
where
    Self: Default,
{
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let mut vec = Self::default();
        vec.extend(iter);
        vec
    }
}
