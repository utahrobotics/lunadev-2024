use std::sync::Arc;

use unros::rayon::iter::{
    plumbing::{bridge, Consumer, Producer, ProducerCallback, UnindexedConsumer},
    IndexedParallelIterator, ParallelIterator,
};

#[derive(Debug)]
pub struct ArcIter<T> {
    slice: Arc<[T]>,
}

impl<T> ArcIter<T> {
    pub fn new(slice: Arc<[T]>) -> Self {
        ArcIter { slice }
    }
}

impl<T> Clone for ArcIter<T> {
    fn clone(&self) -> Self {
        ArcIter {
            slice: self.slice.clone(),
        }
    }
}

impl<T: Sync + Send + Copy> ParallelIterator for ArcIter<T> {
    type Item = T;

    fn drive_unindexed<C>(self, consumer: C) -> C::Result
    where
        C: UnindexedConsumer<Self::Item>,
    {
        bridge(self, consumer)
    }

    fn opt_len(&self) -> Option<usize> {
        Some(self.len())
    }
}

impl<T: Sync + Send + Copy> IndexedParallelIterator for ArcIter<T> {
    fn drive<C>(self, consumer: C) -> C::Result
    where
        C: Consumer<Self::Item>,
    {
        bridge(self, consumer)
    }

    fn len(&self) -> usize {
        self.slice.len()
    }

    fn with_producer<CB>(self, callback: CB) -> CB::Output
    where
        CB: ProducerCallback<Self::Item>,
    {
        callback.callback(IterProducer {
            slice: self.slice.clone(),
            index: 0,
            end_index: self.slice.len() - 1,
        })
    }
}

struct IterProducer<T> {
    slice: Arc<[T]>,
    index: usize,
    end_index: usize,
}

struct SyncArcIter<T> {
    slice: Arc<[T]>,
    index: usize,
    end_index: usize,
    done: bool,
}

impl<T: Copy> Iterator for SyncArcIter<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        self.index += 1;
        self.slice.get(self.index - 1).copied()
    }
}

impl<T: Copy> ExactSizeIterator for SyncArcIter<T> {}
impl<T: Copy> DoubleEndedIterator for SyncArcIter<T> {
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        if self.end_index < self.index {
            None
        } else {
            let i = self.end_index;
            if self.end_index == 0 {
                self.done = true;
            } else {
                self.end_index -= 1;
            }
            self.slice.get(i).copied()
        }
    }
}

impl<T: Sync + Send + Copy> Producer for IterProducer<T> {
    type Item = T;
    type IntoIter = SyncArcIter<T>;

    fn into_iter(self) -> Self::IntoIter {
        SyncArcIter {
            slice: self.slice.clone(),
            index: self.index,
            end_index: self.end_index,
            done: false,
        }
    }

    fn split_at(self, midpoint: usize) -> (Self, Self) {
        (
            IterProducer {
                slice: self.slice.clone(),
                index: self.index,
                end_index: midpoint,
            },
            IterProducer {
                slice: self.slice.clone(),
                index: midpoint,
                end_index: self.end_index,
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::HashSet, sync::Arc};

    use unros::rayon::iter::{IntoParallelIterator, ParallelIterator};

    use super::ArcIter;

    #[test]
    fn test01() {
        let src: Box<[usize]> = (0..50000usize).into_iter().collect();
        let src: Arc<[usize]> = Arc::from(src);

        let dst: HashSet<usize> = ArcIter::new(src.clone()).into_par_iter().collect();
        let src: HashSet<usize> = src.iter().copied().collect();

        assert_eq!(src, dst);
    }
}
