//! Subscriptions are produced by Subscribers with the intent of being consumed by Publishers.
//! However, Subscriptions can be manipulated in the same way as iterators to change its generic
//! type, allowing Subscriptions and Publishers with different generic types to connect.

use std::{
    borrow::Cow,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc::{sync_channel, SyncSender},
        Arc, Mutex, Weak,
    },
};

use crossbeam::utils::Backoff;
use log::warn;
use rayon::ThreadPoolBuilder;

use super::{MonoPublisher, Publisher, SubscriberInner};

/// A token produced only by Publishers, ensuring that only Unros
/// can call specific methods.
pub struct PublisherToken<'a>(pub(super) PhantomData<&'a ()>);

/// A trait for all Subscriptions, similar to the `Iterator` trait.
pub trait Subscription {
    type Item;

    /// Places a value into this Subscription.
    ///
    /// Returns `true` iff the value could be consumed. If `false` is returned,
    /// this `Subscription` should be dropped by the caller.
    fn push(&mut self, value: Self::Item, token: PublisherToken) -> bool;

    /// Changes the generic type of this `Subscription` using the given `map` function.
    fn map<F, O>(self, map: F) -> Map<Self, F, O>
    where
        Self: Sized,
        F: FnMut(O) -> Self::Item,
    {
        Map {
            inner: self,
            map,
            _phantom: PhantomData,
        }
    }

    /// Changes the generic type of this `Subscription` using the given `filter_map` function.
    ///
    /// If the function returns `None`, the value will not be published.
    fn filter_map<F, O>(self, map: F) -> FilterMap<Self, F, O>
    where
        Self: Sized,
        F: FnMut(O) -> Option<Self::Item>,
    {
        FilterMap {
            inner: self,
            map,
            _phantom: PhantomData,
        }
    }

    /// Convenience method to box this subscription.
    fn boxed(self) -> BoxedSubscription<Self::Item>
    where
        Self: Sized + Send + 'static,
    {
        Box::new(self)
    }

    fn detach_unordered(self, num_threads: usize) -> UnorderedDetached<Self::Item>
    where
        Self: Sized + Send + Sync + 'static,
        Self::Item: Send,
    {
        let (sender, receiver) = sync_channel(0);
        let receiver = Mutex::new(receiver);
        let sub = Mutex::new(self);

        std::thread::spawn(move || {
            let thread_pool = ThreadPoolBuilder::new()
                .panic_handler(|_| {})
                .num_threads(num_threads)
                .build()
                .unwrap();
            thread_pool.broadcast(move |_| loop {
                let msg = {
                    let Ok(receiver) = receiver.lock() else {
                        break;
                    };
                    receiver.recv()
                };

                let Ok(mut sub) = sub.lock() else {
                    break;
                };

                match msg {
                    Ok(DetachedCommand::NewValue(value)) => {
                        sub.push(value, PublisherToken(PhantomData));
                    }
                    Ok(DetachedCommand::Increment) => {
                        sub.increment_publishers(PublisherToken(PhantomData));
                    }
                    Ok(DetachedCommand::Decrement) => {
                        sub.decrement_publishers(PublisherToken(PhantomData));
                    }
                    Ok(DetachedCommand::SetName(name)) => {
                        sub.set_name_mut(name);
                    }
                    Err(_) => {
                        break;
                    }
                }
            });
        });
        UnorderedDetached { sender }
    }

    fn detach_ordered(self, num_threads: usize) -> OrderedDetached<Self::Item>
    where
        Self: Sized + Send + Sync + 'static,
        Self::Item: Send,
    {
        let (sender, receiver) = sync_channel(0);
        let receiver = Mutex::new(receiver);
        let sub = Mutex::new(self);
        let task_index = AtomicUsize::new(0);

        std::thread::spawn(move || {
            let thread_pool = ThreadPoolBuilder::new()
                .panic_handler(|_| {})
                .num_threads(num_threads)
                .build()
                .unwrap();
            thread_pool.broadcast(move |_| {
                let backoff = Backoff::new();
                loop {
                    let msg = {
                        let Ok(receiver) = receiver.lock() else {
                            break;
                        };
                        receiver.recv()
                    };

                    let Ok(mut sub) = sub.lock() else {
                        break;
                    };

                    match msg {
                        Ok(DetachedCommand::NewValue((index, value))) => {
                            while task_index.load(Ordering::Acquire) != index {
                                backoff.snooze();
                            }
                            backoff.reset();
                            sub.push(value, PublisherToken(PhantomData));
                            task_index.fetch_add(1, Ordering::AcqRel);
                        }
                        Ok(DetachedCommand::Increment) => {
                            sub.increment_publishers(PublisherToken(PhantomData));
                        }
                        Ok(DetachedCommand::Decrement) => {
                            sub.decrement_publishers(PublisherToken(PhantomData));
                        }
                        Ok(DetachedCommand::SetName(name)) => {
                            sub.set_name_mut(name);
                        }
                        Err(_) => {
                            break;
                        }
                    }
                }
            });
        });
        OrderedDetached {
            sender,
            index: Arc::default(),
        }
    }

    fn detach_sequenced(self, num_threads: usize) -> SequencedDetached<Self::Item>
    where
        Self: Sized + Send + Sync + 'static,
        Self::Item: Send,
    {
        let (sender, receiver) = sync_channel(0);
        let receiver = Mutex::new(receiver);
        let mut task_index = 0usize;
        let sub = Mutex::new(self.filter_map(move |(index, value)| {
            if index >= task_index {
                task_index = task_index.wrapping_add(index + 1);
                Some(value)
            } else {
                None
            }
        }));

        std::thread::spawn(move || {
            let thread_pool = ThreadPoolBuilder::new()
                .panic_handler(|_| {})
                .num_threads(num_threads)
                .build()
                .unwrap();
            thread_pool.broadcast(move |_| loop {
                let msg = {
                    let Ok(receiver) = receiver.lock() else {
                        break;
                    };
                    receiver.recv()
                };

                let Ok(mut sub) = sub.lock() else {
                    break;
                };

                match msg {
                    Ok(DetachedCommand::NewValue(item)) => {
                        sub.push(item, PublisherToken(PhantomData));
                    }
                    Ok(DetachedCommand::Increment) => {
                        sub.increment_publishers(PublisherToken(PhantomData));
                    }
                    Ok(DetachedCommand::Decrement) => {
                        sub.decrement_publishers(PublisherToken(PhantomData));
                    }
                    Ok(DetachedCommand::SetName(name)) => {
                        sub.set_name_mut(name);
                    }
                    Err(_) => {
                        break;
                    }
                }
            });
        });
        SequencedDetached {
            sender,
            index: Arc::default(),
        }
    }

    // fn zip<V: 'static>(mut self, mut other: DirectSubscription<V>) -> DirectSubscription<(T, V)> where Self: Sized {
    // self.pub_count.append(&mut other.pub_count);
    // DirectSubscription {
    //     queue: Box::new(move |(left, right)| {
    //         let left_result = self.queue.push(left);
    //         let right_result = other.queue.push(right);
    //         match left_result {
    //             EnqueueResult::Ok => right_result,
    //             EnqueueResult::Full => {
    //                 if right_result == EnqueueResult::Closed {
    //                     EnqueueResult::Closed
    //                 } else {
    //                     EnqueueResult::Full
    //                 }
    //             }
    //             EnqueueResult::Closed => EnqueueResult::Closed,
    //         }
    //     }),
    //     notify: self.notify,
    //     lag: 0,
    //     name: None,
    //     pub_count: self.pub_count,
    // }
    // }

    /// Provides a name to this subscription, which enables lag logging.
    ///
    /// If the `Publisher` that accepts this `Subscription` cannot push
    /// new messages into this `Subscription` without deleting old message,
    /// we say that the `Subscription` is lagging. Catching lagging is important
    /// as it indicates data loss and a lack of processing speed. With a name,
    /// these lags will be logged as warnings in the standard log file (`.log`).
    #[must_use]
    fn set_name(mut self, name: impl Into<Cow<'static, str>>) -> Self
    where
        Self: Sized,
    {
        self.set_name_mut(name.into());
        self
    }

    /// Analagous to `set_name`, except that mutation is done through a mutable reference.
    fn set_name_mut(&mut self, name: Cow<'static, str>);

    /// Increments the publisher count of the `Subscriber`, which is important for it to know
    /// when no more publishers are connected to it.
    fn increment_publishers(&self, token: PublisherToken);

    /// Decrements the publisher count of the `Subscriber`, which is important for it to know
    /// when no more publishers are connected to it.
    fn decrement_publishers(&self, token: PublisherToken);

    fn into_mono_pub(self) -> MonoPublisher<Self::Item, Self>
    where
        Self: Sized,
    {
        MonoPublisher::from(self)
    }

    fn into_pub(self) -> Publisher<Self::Item>
    where
        Self: Sized + Send + 'static,
    {
        let publisher = Publisher::default();
        publisher.accept_subscription(self);
        publisher
    }
}

/// An object that must be passed to a `Publisher`, enabling the `Subscriber`
/// that created the subscription to receive messages from that `Publisher`.
///
/// If dropped, no change will occur to the `Subscriber` and no resources will be leaked.
#[derive(Debug)]
pub struct DirectSubscription<T> {
    pub(super) sub: Weak<SubscriberInner<T>>,
    pub(super) name: Option<Cow<'static, str>>,
    pub(super) lag: usize,
}

impl<T> Clone for DirectSubscription<T> {
    fn clone(&self) -> Self {
        Self {
            sub: self.sub.clone(),
            name: self.name.clone(),
            lag: self.lag,
        }
    }
}

impl<T> Subscription for DirectSubscription<T> {
    type Item = T;

    fn push(&mut self, value: Self::Item, _token: PublisherToken) -> bool {
        if let Some(sub) = self.sub.upgrade() {
            if sub.queue.force_push(value).is_some() {
                self.lag += 1;
                if let Some(name) = &self.name {
                    warn!(target: "publishers", "{name} lagging by {} messages", self.lag);
                }
            } else {
                self.lag = 0;
                sub.notify.notify_one();
            }
            true
        } else {
            false
        }
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>)
    where
        Self: Sized,
    {
        self.name = Some(name);
    }

    fn increment_publishers(&self, _token: PublisherToken) {
        if let Some(sub) = self.sub.upgrade() {
            sub.pub_count.fetch_add(1, Ordering::AcqRel);
        }
    }

    fn decrement_publishers(&self, _token: PublisherToken) {
        if let Some(sub) = self.sub.upgrade() {
            sub.pub_count.fetch_sub(1, Ordering::AcqRel);
            sub.notify.notify_one();
        }
    }
}

#[derive(Debug)]
pub struct Map<I, F, O> {
    inner: I,
    map: F,
    _phantom: PhantomData<O>,
}

impl<O, I, F> Subscription for Map<I, F, O>
where
    I: Subscription,
    F: FnMut(O) -> I::Item,
{
    type Item = O;

    fn push(&mut self, value: Self::Item, token: PublisherToken) -> bool {
        self.inner.push((self.map)(value), token)
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>) {
        self.inner.set_name_mut(name);
    }

    fn increment_publishers(&self, token: PublisherToken) {
        self.inner.increment_publishers(token);
    }

    fn decrement_publishers(&self, token: PublisherToken) {
        self.inner.decrement_publishers(token);
    }
}

impl<I: Clone, F: Clone, O> Clone for Map<I, F, O> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            map: self.map.clone(),
            _phantom: PhantomData,
        }
    }
}

#[derive(Debug)]
pub struct FilterMap<I, F, O> {
    inner: I,
    map: F,
    _phantom: PhantomData<O>,
}

impl<O, I, F> Subscription for FilterMap<I, F, O>
where
    I: Subscription,
    F: FnMut(O) -> Option<I::Item>,
{
    type Item = O;

    fn push(&mut self, value: Self::Item, token: PublisherToken) -> bool {
        if let Some(value) = (self.map)(value) {
            self.inner.push(value, token)
        } else {
            true
        }
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>) {
        self.inner.set_name_mut(name);
    }

    fn increment_publishers(&self, token: PublisherToken) {
        self.inner.increment_publishers(token);
    }

    fn decrement_publishers(&self, token: PublisherToken) {
        self.inner.decrement_publishers(token);
    }
}

impl<I: Clone, F: Clone, O> Clone for FilterMap<I, F, O> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            map: self.map.clone(),
            _phantom: PhantomData,
        }
    }
}

pub type BoxedSubscription<T> = Box<dyn Subscription<Item = T> + Send>;

impl<T> Subscription for BoxedSubscription<T> {
    type Item = T;

    fn push(&mut self, value: Self::Item, token: PublisherToken) -> bool {
        self.deref_mut().push(value, token)
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>) {
        self.deref_mut().set_name_mut(name);
    }

    fn increment_publishers(&self, token: PublisherToken) {
        self.deref().increment_publishers(token);
    }

    fn decrement_publishers(&self, token: PublisherToken) {
        self.deref().decrement_publishers(token);
    }
}

enum DetachedCommand<T> {
    NewValue(T),
    Increment,
    Decrement,
    SetName(Cow<'static, str>),
}

pub struct UnorderedDetached<T> {
    sender: SyncSender<DetachedCommand<T>>,
}

impl<T> Clone for UnorderedDetached<T> {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
        }
    }
}

impl<T> Subscription for UnorderedDetached<T> {
    type Item = T;

    fn push(&mut self, value: Self::Item, _token: PublisherToken) -> bool {
        self.sender.send(DetachedCommand::NewValue(value)).is_ok()
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>) {
        let _ = self.sender.send(DetachedCommand::SetName(name));
    }

    fn increment_publishers(&self, _token: PublisherToken) {
        let _ = self.sender.send(DetachedCommand::Increment);
    }

    fn decrement_publishers(&self, _token: PublisherToken) {
        let _ = self.sender.send(DetachedCommand::Decrement);
    }
}

pub struct OrderedDetached<T> {
    sender: SyncSender<DetachedCommand<(usize, T)>>,
    index: Arc<AtomicUsize>,
}

impl<T> Clone for OrderedDetached<T> {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
            index: self.index.clone(),
        }
    }
}

impl<T> Subscription for OrderedDetached<T> {
    type Item = T;

    fn push(&mut self, value: Self::Item, _token: PublisherToken) -> bool {
        let index = self.index.fetch_add(1, Ordering::AcqRel);
        self.sender
            .send(DetachedCommand::NewValue((index, value)))
            .is_ok()
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>) {
        let _ = self.sender.send(DetachedCommand::SetName(name));
    }

    fn increment_publishers(&self, _token: PublisherToken) {
        let _ = self.sender.send(DetachedCommand::Increment);
    }

    fn decrement_publishers(&self, _token: PublisherToken) {
        let _ = self.sender.send(DetachedCommand::Decrement);
    }
}

pub struct SequencedDetached<T> {
    sender: SyncSender<DetachedCommand<(usize, T)>>,
    index: Arc<AtomicUsize>,
}

impl<T> Clone for SequencedDetached<T> {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
            index: self.index.clone(),
        }
    }
}

impl<T> Subscription for SequencedDetached<T> {
    type Item = T;

    fn push(&mut self, value: Self::Item, _token: PublisherToken) -> bool {
        let index = self.index.fetch_add(1, Ordering::AcqRel);
        self.sender
            .send(DetachedCommand::NewValue((index, value)))
            .is_ok()
    }

    fn set_name_mut(&mut self, name: Cow<'static, str>) {
        let _ = self.sender.send(DetachedCommand::SetName(name));
    }

    fn increment_publishers(&self, _token: PublisherToken) {
        let _ = self.sender.send(DetachedCommand::Increment);
    }

    fn decrement_publishers(&self, _token: PublisherToken) {
        let _ = self.sender.send(DetachedCommand::Decrement);
    }
}
