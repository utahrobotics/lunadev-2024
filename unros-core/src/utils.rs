//! Several utilities that complement the Unros framework.

use std::{
    ops::{Deref, DerefMut},
    sync::OnceLock,
};

use crossbeam::queue::ArrayQueue;

struct DropWrapperInner<T, F> {
    inner: T,
    drop_fn: F,
}

/// A simple struct around a type that calls a function when it is dropped.
///
/// This can be used to avoid implementing `Drop` on a new type that wraps an existing type.
/// However, since it is possible to get a mutable reference to the inner type, it is possible
/// to move the inner type out or replace it, which will *not* call the drop function. The drop
/// function is only called when this struct is dropped.
pub struct DropWrapper<T, F: FnOnce(T)> {
    inner: Option<DropWrapperInner<T, F>>,
}

impl<T, F> Drop for DropWrapper<T, F>
where
    F: FnOnce(T),
{
    fn drop(&mut self) {
        if let Some(inner) = self.inner.take() {
            (inner.drop_fn)(inner.inner);
        }
    }
}

impl<T, F: FnOnce(T)> DropWrapper<T, F> {
    pub fn new(inner: T, drop_fn: F) -> Self {
        Self {
            inner: Some(DropWrapperInner { inner, drop_fn }),
        }
    }
}

impl<T, F: FnOnce(T)> Deref for DropWrapper<T, F> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner.as_ref().unwrap().inner
    }
}

impl<T, F: FnOnce(T)> DerefMut for DropWrapper<T, F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner.as_mut().unwrap().inner
    }
}

// pub struct TimeVec<T, F = Box<dyn Fn(&VecDeque<T>) -> T + Send>> {
//     vec: VecDeque<T>,
//     head_time: Instant,
//     duration: Duration,
//     max_length: usize,
//     default: F,
// }

// impl<T, F: FnMut(&VecDeque<T>) -> T + Send> TimeVec<T, F> {
//     #[must_use]
//     pub fn new(max_length: usize, duration: Duration, mut default: F) -> Self {
//         let mut vec = VecDeque::with_capacity(max_length);
//         for _ in 0..max_length {
//             vec.push_back(default(&vec));
//         }
//         Self {
//             vec,
//             head_time: Instant::now(),
//             duration,
//             max_length,
//             default,
//         }
//     }

//     #[must_use]
//     pub fn new_default(
//         max_length: usize,
//         duration: Duration,
//     ) -> TimeVec<T, Box<dyn Fn(&VecDeque<T>) -> T + Send>>
//     where
//         T: Default,
//     {
//         TimeVec::new(max_length, duration, Box::new(|_| Default::default()))
//     }

//     fn update(&mut self) {
//         let single_duration = self.duration.div_f32(self.max_length as f32);
//         let elapsed = self.head_time.elapsed();
//         let mut lag = elapsed.div_duration_f32(single_duration) as usize;
//         lag = lag.min(self.max_length);
//         self.vec.drain(0..lag);
//         for _ in 0..lag {
//             self.vec.push_back((self.default)(&self.vec));
//         }
//         if lag > 0 {
//             self.head_time += elapsed;
//         }
//     }

//     pub fn peek(&mut self) -> &mut T {
//         self.update();
//         self.vec.back_mut().unwrap()
//     }

//     fn get_index(&mut self, instant: Instant) -> Option<usize> {
//         self.update();
//         let prelapsed = self.head_time.checked_duration_since(instant)?;
//         let single_duration = self.duration.div_f32(self.max_length as f32);
//         let i = prelapsed.div_duration_f32(single_duration) as usize;
//         if i >= self.max_length {
//             None
//         } else {
//             Some(self.max_length - i - 1)
//         }
//     }

//     pub fn get(&mut self, instant: Instant) -> Option<&mut T> {
//         let i = self.get_index(instant)?;
//         self.vec.get_mut(i)
//     }

//     pub fn get_pair(&mut self, instant: Instant) -> Option<(&mut T, &mut T)> {
//         let i = self.get_index(instant)?;
//         if i >= self.max_length - 1 {
//             None
//         } else {
//             let mut iter = self.vec.range_mut(i..=i + 1);
//             let a = iter.next().unwrap();
//             let b = iter.next().unwrap();
//             Some((a, b))
//         }
//     }

//     pub fn get_head_time(&mut self) -> Instant {
//         self.update();
//         self.head_time
//     }

//     pub fn get_vec(&mut self) -> BorrowedVecDeque<T, F> {
//         self.update();
//         BorrowedVecDeque(self)
//     }
// }

// pub struct BorrowedVecDeque<'a, T, F: FnMut(&VecDeque<T>) -> T>(&'a mut TimeVec<T, F>);

// impl<'a, T, F: FnMut(&VecDeque<T>) -> T> Deref for BorrowedVecDeque<'a, T, F> {
//     type Target = VecDeque<T>;

//     fn deref(&self) -> &Self::Target {
//         &self.0.vec
//     }
// }

// impl<'a, T, F: FnMut(&VecDeque<T>) -> T> DerefMut for BorrowedVecDeque<'a, T, F> {
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         &mut self.0.vec
//     }
// }

// impl<'a, T, F: FnMut(&VecDeque<T>) -> T> Drop for BorrowedVecDeque<'a, T, F> {
//     fn drop(&mut self) {
//         match self.0.vec.len().cmp(&self.0.max_length) {
//             Ordering::Less => {
//                 for _ in 0..(self.0.max_length - self.0.vec.len()) {
//                     self.0.vec.push_back((self.0.default)(&self.0.vec));
//                 }
//             }

//             Ordering::Equal => {}
//             Ordering::Greater => {
//                 self.0.vec.drain(0..self.0.vec.len() - self.0.max_length);
//             }
//         }
//     }
// }

/// A concurrent queue for storing a pool of resources.
pub struct ResourceQueue<T> {
    queue: OnceLock<ArrayQueue<T>>,
    max_length: usize,
    default: fn() -> T,
}

impl<T> ResourceQueue<T> {
    /// Create a new queue where elements can be generated by the given function pointer.
    pub const fn new(max_length: usize, default: fn() -> T) -> Self {
        Self {
            queue: OnceLock::new(),
            max_length,
            default,
        }
    }

    /// Gets any available resource.
    pub fn get(&self) -> ResourceGuard<T> {
        let inner = self
            .queue
            .get_or_init(|| ArrayQueue::new(self.max_length))
            .pop()
            .unwrap_or_else(self.default);
        ResourceGuard {
            inner: Some(inner),
            queue: self,
        }
    }

    /// Manually sets a resource into this queue.
    ///
    /// You only need to call this if you want to add elements into this queue since
    /// `ResourceGuard` calls this automatically.
    ///
    /// If you set more values than this queue can hold, the oldest resource is dropped.
    pub fn set(&self, value: T) {
        self.queue
            .get_or_init(|| ArrayQueue::new(self.max_length))
            .force_push(value);
    }
}

/// An RAII Guard around a resource.
///
/// When this is dropped, the resource will be returned to the `ResourceQueue`.
pub struct ResourceGuard<'a, T> {
    inner: Option<T>,
    queue: &'a ResourceQueue<T>,
}

impl<'a, T> ResourceGuard<'a, T> {
    /// Unwraps this object, returning the resource instead of returning it to the
    /// `ResourceQueue`.
    ///
    /// If the queue is lacking a resource, a new one will be generated.
    pub fn do_not_return(mut this: Self) -> T {
        this.inner.take().unwrap()
    }
}

impl<'a, T> Deref for ResourceGuard<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.inner.as_ref().unwrap()
    }
}

impl<'a, T> DerefMut for ResourceGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.as_mut().unwrap()
    }
}

impl<'a, T> Drop for ResourceGuard<'a, T> {
    fn drop(&mut self) {
        if let Some(inner) = self.inner.take() {
            self.queue.queue.get().unwrap().force_push(inner);
        }
    }
}
