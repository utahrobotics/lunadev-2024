use std::iter::Sum;

use nalgebra::RealField;
use simba::scalar::{SubsetOf, SupersetOf};

pub(crate) struct UnorderedQueue<T> {
    queue: Box<[T]>,
    index: usize,
}

impl<T> UnorderedQueue<T> {
    pub fn push(&mut self, value: T) {
        self.queue[self.index] = value;
        self.index += 1;
        if self.index >= self.queue.len() {
            self.index = 0;
        }
    }

    pub fn as_slice(&self) -> &[T] {
        &self.queue
    }

    // pub fn as_mut_slice(&mut self) -> &mut [T] {
    //     &mut self.queue
    // }
}

impl<T> FromIterator<T> for UnorderedQueue<T> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        UnorderedQueue {
            queue: iter.into_iter().collect(),
            index: 0,
        }
    }
}

pub trait Float:
    RealField
    + Copy
    + Default
    + SupersetOf<f32>
    + SupersetOf<f64>
    + SubsetOf<f32>
    + SubsetOf<f64>
    + SupersetOf<usize>
    + Sum
{
    fn to_f32(self) -> f32;
    fn to_f64(self) -> f64;
    fn is_nan(self) -> bool;
}

impl Float for f32 {
    fn to_f32(self) -> f32 {
        self
    }

    fn to_f64(self) -> f64 {
        self as f64
    }

    fn is_nan(self) -> bool {
        self.is_nan()
    }
}
impl Float for f64 {
    fn to_f32(self) -> f32 {
        self as f32
    }

    fn to_f64(self) -> f64 {
        self
    }

    fn is_nan(self) -> bool {
        self.is_nan()
    }
}
