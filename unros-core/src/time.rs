use std::{
    cmp::Ordering,
    collections::VecDeque,
    ops::{Deref, DerefMut},
    time::{Duration, Instant},
};

pub struct TimeVec<T, F = fn(&VecDeque<T>) -> T> {
    vec: VecDeque<T>,
    head_time: Instant,
    duration: Duration,
    max_length: usize,
    default: F,
}

impl<T, F: FnMut(&VecDeque<T>) -> T> TimeVec<T, F> {
    #[must_use]
    pub fn new(max_length: usize, duration: Duration, mut default: F) -> Self {
        let mut vec = VecDeque::with_capacity(max_length);
        for _ in 0..max_length {
            vec.push_back(default(&vec));
        }
        Self {
            vec,
            head_time: Instant::now(),
            duration,
            max_length,
            default,
        }
    }

    #[must_use]
    pub fn new_default(max_length: usize, duration: Duration) -> TimeVec<T>
    where
        T: Default,
    {
        TimeVec::new(max_length, duration, |_| Default::default())
    }

    fn update(&mut self) {
        let single_duration = self.duration.div_f32(self.max_length as f32);
        let elapsed = self.head_time.elapsed();
        let mut lag = elapsed.div_duration_f32(single_duration) as usize;
        lag = lag.min(self.max_length);
        self.vec.drain(0..lag);
        for _ in 0..lag {
            self.vec.push_back((self.default)(&self.vec));
        }
        if lag > 0 {
            self.head_time += elapsed;
        }
    }

    pub fn peek(&mut self) -> &mut T {
        self.update();
        self.vec.back_mut().unwrap()
    }

    fn get_index(&mut self, instant: Instant) -> Option<usize> {
        self.update();
        let prelapsed = self.head_time.checked_duration_since(instant)?;
        let single_duration = self.duration.div_f32(self.max_length as f32);
        let i = prelapsed.div_duration_f32(single_duration) as usize;
        if i >= self.max_length {
            None
        } else {
            Some(self.max_length - i - 1)
        }
    }

    pub fn get(&mut self, instant: Instant) -> Option<&mut T> {
        let i = self.get_index(instant)?;
        self.vec.get_mut(i)
    }

    pub fn get_pair(&mut self, instant: Instant) -> Option<(&mut T, &mut T)> {
        let i = self.get_index(instant)?;
        if i >= self.max_length - 1 {
            None
        } else {
            let mut iter = self.vec.range_mut(i..=i + 1);
            let a = iter.next().unwrap();
            let b = iter.next().unwrap();
            Some((a, b))
        }
    }

    pub fn get_head_time(&mut self) -> Instant {
        self.update();
        self.head_time
    }

    pub fn get_vec(&mut self) -> BorrowedVecDeque<T, F> {
        self.update();
        BorrowedVecDeque(self)
    }
}

pub struct BorrowedVecDeque<'a, T, F: FnMut(&VecDeque<T>) -> T>(&'a mut TimeVec<T, F>);

impl<'a, T, F: FnMut(&VecDeque<T>) -> T> Deref for BorrowedVecDeque<'a, T, F> {
    type Target = VecDeque<T>;

    fn deref(&self) -> &Self::Target {
        &self.0.vec
    }
}

impl<'a, T, F: FnMut(&VecDeque<T>) -> T> DerefMut for BorrowedVecDeque<'a, T, F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0.vec
    }
}

impl<'a, T, F: FnMut(&VecDeque<T>) -> T> Drop for BorrowedVecDeque<'a, T, F> {
    fn drop(&mut self) {
        match self.0.vec.len().cmp(&self.0.max_length) {
            Ordering::Less => {
                for _ in 0..(self.0.max_length - self.0.vec.len()) {
                    self.0.vec.push_back((self.0.default)(&self.0.vec));
                }
            }

            Ordering::Equal => {}
            Ordering::Greater => {
                self.0.vec.drain(0..self.0.vec.len() - self.0.max_length);
            }
        }
    }
}
