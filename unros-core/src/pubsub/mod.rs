//! Publishers and Subscribers are an essential component of many frameworks in many different disciplines.
//!
//! This is one of the aspects taken from `ROS` that have been greatly improved. We offer
//! an `API` for pubsub that is more similar to `Rust`'s iterators. Signals are analagous
//! to `Rust`'s channels in that they do not trigger code, unlike `ROS` subscriber callbacks.

use std::{
    collections::VecDeque,
    io::Write,
    ops::{Deref, DerefMut},
    path::Path,
    sync::{Arc, Weak},
};

use crossbeam::queue::ArrayQueue;
use log::warn;
use tokio::sync::Notify;

use crate::logging::{dump::DataDump, START_TIME};

/// An essential component that promotes separation of concerns, and is
/// an intrinsic element of the ROS framework.
///
/// Publishers provide a simple way to send a message to receivers, much
/// like Rust's channels. These are analagous to single-producer-multi-consumer
/// channels.
///
/// Signals make numerous clones of the values it will send, so you should
/// use a type `T` that is cheap to clone with this signal. A good default is
/// `Arc`. Since Nodes will often be used from different threads, the type `T`
/// should also be `Send`.
pub struct Publisher<T> {
    subs: VecDeque<Subscription<T>>,
}

impl<T> Default for Publisher<T> {
    fn default() -> Self {
        Self {
            subs: VecDeque::default(),
        }
    }
}

impl<T: Clone> Publisher<T> {
    /// Sets a value into this signal, allowing it to be received by Subscribers.
    ///
    /// Only the node that owns this signal should call this method.
    pub fn set(&mut self, value: T) {
        self.subs
            .retain_mut(|sub| match sub.queue.push(value.clone()) {
                EnqueueResult::Ok => {
                    sub.lag = 0;
                    if let Some(notify) = sub.notify.upgrade() {
                        notify.notify_one();
                        true
                    } else {
                        false
                    }
                }
                EnqueueResult::Full => {
                    sub.lag += 1;
                    if let Some(name) = &sub.name {
                        warn!(target: "publishers", "{name} lagging by {} messages", sub.lag);
                    }
                    if let Some(notify) = sub.notify.upgrade() {
                        notify.notify_one();
                        true
                    } else {
                        false
                    }
                }
                EnqueueResult::Closed => false,
            });
    }

    /// Accepts a given subscription, allowing the corresponding `Subscriber` to
    /// receive new messages.
    pub fn accept_subscription(&mut self, sub: Subscription<T>) {
        self.subs.push_back(sub);
    }
}

impl<T> Drop for Publisher<T> {
    fn drop(&mut self) {
        for sub in self.subs.drain(..) {
            drop(sub.queue);
            if let Some(notify) = sub.notify.upgrade() {
                notify.notify_one();
            };
        }
    }
}

trait Queue<T>: Send + Sync {
    fn push(&self, value: T) -> EnqueueResult;
}

#[derive(PartialEq, Eq)]
enum EnqueueResult {
    Ok,
    Full,
    Closed,
}

impl<T: Send> Queue<T> for Weak<ArrayQueue<T>> {
    fn push(&self, value: T) -> EnqueueResult {
        if let Some(queue) = self.upgrade() {
            if queue.force_push(value).is_some() {
                EnqueueResult::Full
            } else {
                EnqueueResult::Ok
            }
        } else {
            EnqueueResult::Closed
        }
    }
}

impl<T, F: Fn(T) -> EnqueueResult + Send + Sync> Queue<T> for F {
    fn push(&self, value: T) -> EnqueueResult {
        self(value)
    }
}

/// An essential companion to the `Publisher`.
///
/// Subscribers are bounded queues that can receive messages `T`
/// from multiple Publishers concurrently. To subscribe to a `Publisher`,
/// a `Subscriber` must create a subscription and pass that to the `Publisher`.
pub struct Subscriber<T> {
    queue: Arc<ArrayQueue<T>>,
    notify: Arc<Notify>,
}

/// An object that must be passed to a `Publisher`, enabling the `Subscriber`
/// that created the subscription to receive messages from that `Publisher`.
///
/// If dropped, no change will occur to the `Subscriber` and no resources will be leaked.
pub struct Subscription<T> {
    // subscriber: Box<dyn FnOnce(watch::Receiver<()>, Option<Box<str>>)>,
    queue: Box<dyn Queue<T>>,
    notify: Weak<Notify>,
    name: Option<Box<str>>,
    lag: usize,
}

impl<T: Clone + Send + 'static> Subscriber<T> {
    #[must_use]
    pub fn new(size: usize) -> Self {
        Self {
            queue: Arc::new(ArrayQueue::new(size)),
            notify: Arc::new(Notify::new()),
        }
    }

    /// Receive some message (waiting if none are available), or `None` if all `Publishers` have been dropped.
    pub async fn recv_or_closed(&mut self) -> Option<T> {
        loop {
            if let Some(value) = self.queue.pop() {
                return Some(value);
            }

            if Arc::weak_count(&self.queue) == 0 {
                return None;
            }

            self.notify.notified().await;
        }
    }

    /// Try to receive a message if one is available.
    pub fn try_recv(&mut self) -> Option<T> {
        self.queue.pop()
    }

    /// Wait until a message is received, even if all `Publisher`s have been dropped.
    pub async fn recv(&mut self) -> T {
        if let Some(x) = self.recv_or_closed().await {
            x
        } else {
            std::future::pending().await
        }
    }

    /// Convert this `Subscriber` into a logger that logs
    /// all received messages formatted using the `display` function.
    ///
    /// Logs are saved to `path` using a `DataDump`.
    pub async fn into_logger(
        mut self,
        mut display: impl FnMut(T) -> String + Send + 'static,
        path: impl AsRef<Path>,
    ) -> std::io::Result<()> {
        let mut dump = DataDump::new_file(path).await?;
        tokio::spawn(async move {
            loop {
                let Some(value) = self.recv_or_closed().await else {
                    break;
                };
                let secs = START_TIME.get().unwrap().elapsed().as_secs_f32();
                writeln!(
                    dump,
                    "[{:0>1}:{:.2}] {}",
                    (secs / 60.0).floor(),
                    secs % 60.0,
                    display(value)
                )
                .unwrap();
            }
        });
        Ok(())
    }

    /// Creates a `Subscription` that needs to be passed to a `Publisher`.
    #[must_use]
    pub fn create_subscription(&self) -> Subscription<T> {
        Subscription {
            queue: Box::new(Arc::downgrade(&self.queue)),
            notify: Arc::downgrade(&self.notify),
            name: None,
            lag: 0,
        }
    }

    /// Converts this `Subscriber` into a `WatchSubscriber`.
    ///
    /// Wait until a message is received, even if all `Publisher`s have been dropped.
    pub async fn into_watch(mut self) -> WatchSubscriber<T> {
        WatchSubscriber {
            value: self.recv().await,
            inner: self,
        }
    }

    /// Tries to convert this `Subscriber` into a `WatchSubscriber`.
    ///
    /// If no message is available, this `Subscriber` will be returned.
    pub fn try_into_watch(mut self) -> Result<WatchSubscriber<T>, Self> {
        if let Some(value) = self.try_recv() {
            Ok(WatchSubscriber { value, inner: self })
        } else {
            Err(self)
        }
    }

    /// Convert this `Subscriber` into a `WatchSubscriber`.
    ///
    /// Wait until a message is received. If all `Publisher`s have been dropped this `Subscriber` will be returned.
    pub async fn into_watch_or_closed(mut self) -> Result<WatchSubscriber<T>, Self> {
        if let Some(value) = self.recv_or_closed().await {
            Ok(WatchSubscriber { value, inner: self })
        } else {
            Err(self)
        }
    }
}

impl<T: 'static> Subscription<T> {
    /// Changes the generic type of this `Subscription` using the given `map` function.
    pub fn map<V>(self, map: impl Fn(V) -> T + Send + Sync + 'static) -> Subscription<V> {
        Subscription {
            queue: Box::new(move |x| self.queue.push(map(x))),
            notify: self.notify,
            lag: 0,
            name: None,
        }
    }

    /// Provides a name to this subscription, which enables lag logging.
    ///
    /// If the `Publisher` that accepts this `Subscription` cannot push
    /// new messages into this `Subscription` without deleting old message,
    /// we say that the `Subscription` is lagging. Catching lagging is important
    /// as it indicates data loss and a lack of processing speed. With a name,
    /// these lags will be logged as warnings in the standard log file (`.log`).
    #[must_use]
    pub fn set_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into().into_boxed_str());
        self
    }
}

/// A `Subscriber` that always stores the last received message, acting as a smart pointer for `T`.
///
/// Users must regularly call `update`, `update_or_closed`, or `try_update` to receive newer messages.
pub struct WatchSubscriber<T> {
    inner: Subscriber<T>,
    value: T,
}

impl<T> Deref for WatchSubscriber<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

impl<T> DerefMut for WatchSubscriber<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

impl<T: Clone + Send + 'static> WatchSubscriber<T> {
    /// Creates a new `WatchSubscriber` initialized with the given value and no subscriptions.
    pub fn new(value: T) -> Self {
        Self {
            inner: Subscriber::new(1),
            value,
        }
    }

    /// Wait until a message is received, even if all `Publisher`s have been dropped.
    pub async fn update(sub: &mut Self) {
        if !Self::try_update(sub) {
            sub.value = sub.inner.recv().await;
        }
    }

    /// Wait for a new message (returning `true`), or return `false` if all `Publishers` have been dropped.
    pub async fn update_or_closed(sub: &mut Self) -> bool {
        if Self::try_update(sub) {
            true
        } else if let Some(x) = sub.inner.recv_or_closed().await {
            sub.value = x;
            true
        } else {
            false
        }
    }

    /// Try to receive a new message (returning `true`), or return `false` if no messages are available.
    pub fn try_update(sub: &mut Self) -> bool {
        if Self::try_update_inner(sub) {
            while Self::try_update_inner(sub) {}
            true
        } else {
            false
        }
    }

    fn try_update_inner(sub: &mut Self) -> bool {
        if let Some(x) = sub.inner.try_recv() {
            sub.value = x;
            true
        } else {
            false
        }
    }

    /// Creates a `Subscription` with a size of 1.
    ///
    /// There is no benefit to having a queue size of more than 1.
    pub fn create_subscription(&self) -> Subscription<T> {
        self.inner.create_subscription()
    }

    /// Convert this `WatchSubscriber` into a logger that logs
    /// all received messages formatted using the `display` function.
    ///
    /// Logs are saved to `path` using a `DataDump`.
    pub async fn into_logger(
        self,
        display: impl FnMut(T) -> String + Send + 'static,
        path: impl AsRef<Path>,
    ) -> std::io::Result<()> {
        self.inner.into_logger(display, path).await
    }
}
