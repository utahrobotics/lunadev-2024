//! Publishers and Subscribers are an essential component of many frameworks in many different disciplines.
//!
//! This is one of the aspects taken from `ROS` that have been greatly improved. We offer
//! an `API` for pubsub that is more similar to `Rust`'s iterators. Signals are analagous
//! to `Rust`'s channels in that they do not trigger code, unlike `ROS` subscriber callbacks.

use std::{
    io::Write,
    ops::{Deref, DerefMut},
    path::Path,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc, Weak,
    },
};

use crossbeam::{
    queue::{ArrayQueue, SegQueue},
    utils::Backoff,
};
use tokio::sync::Notify;

pub mod subs;

use crate::logging::{dump::DataDump, START_TIME};

use self::subs::{DirectSubscription, PublisherToken, Subscription};

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
    subs: Arc<SegQueue<Box<dyn Subscription<Item = T> + Send>>>,
}

impl<T> Default for Publisher<T> {
    fn default() -> Self {
        Self {
            subs: Arc::default(),
        }
    }
}

impl<T: Clone> Publisher<T> {
    /// Sets a value into this signal, allowing it to be received by Subscribers.
    ///
    /// Only the node that owns this signal should call this method.
    pub fn set(&mut self, value: T) {
        for _ in 0..self.subs.len() {
            // This is thte only place we pop from subs, so we are guaranteed
            // to always have at least self.subs.len() to elements to pop
            let mut sub = self.subs.pop().unwrap();
            if sub.push(value.clone()) {
                self.subs.push(sub);
            }
        }
    }
}

impl<T> Publisher<T> {
    /// Accepts a given subscription, allowing the corresponding `Subscriber` to
    /// receive new messages.
    pub fn accept_subscription(&self, sub: impl Subscription<Item = T> + Send + 'static) {
        sub.increment_publishers(PublisherToken(()));
        self.subs.push(Box::new(sub));
    }

    pub fn get_ref(&self) -> PublisherRef<T> {
        PublisherRef {
            subs: Arc::downgrade(&self.subs),
        }
    }
}

impl<T> Drop for Publisher<T> {
    fn drop(&mut self) {
        let mut subs = std::mem::take(&mut self.subs);

        let backoff = Backoff::new();
        let subs = loop {
            match Arc::try_unwrap(subs) {
                Ok(x) => break x,
                Err(x) => {
                    backoff.spin();
                    subs = x;
                }
            }
        };

        for sub in subs.into_iter() {
            sub.decrement_publishers(PublisherToken(()));
        }
    }
}

pub struct PublisherRef<T> {
    subs: Weak<SegQueue<Box<dyn Subscription<Item = T> + Send>>>,
}

impl<T> PublisherRef<T> {
    /// Accepts a given subscription, allowing the corresponding `Subscriber` to
    /// receive new messages.
    pub fn accept_subscription(&self, sub: impl Subscription<Item = T> + Send + 'static) {
        self.accept_subscription_or_closed(sub);
    }

    /// Accepts a given subscription, allowing the corresponding `Subscriber` to
    /// receive new messages.
    ///
    /// Returns true iff the original `Publisher` has not been dropped
    pub fn accept_subscription_or_closed(
        &self,
        sub: impl Subscription<Item = T> + Send + 'static,
    ) -> bool {
        let Some(subs) = self.subs.upgrade() else {
            return false;
        };
        sub.increment_publishers(PublisherToken(()));
        subs.push(Box::new(sub));
        true
    }
}

impl<T> Clone for PublisherRef<T> {
    fn clone(&self) -> Self {
        Self {
            subs: self.subs.clone(),
        }
    }
}

#[derive(PartialEq, Eq)]
pub enum EnqueueResult {
    Ok,
    Full,
    Closed,
}

// impl<T: Send + 'static> Subscription<T> for Weak<ArrayQueue<T>> {
//     fn push(&mut self, value: T) -> EnqueueResult {
//         if let Some(queue) = self.upgrade() {
//             if queue.force_push(value).is_some() {
//                 EnqueueResult::Full
//             } else {
//                 EnqueueResult::Ok
//             }
//         } else {
//             EnqueueResult::Closed
//         }
//     }
//     fn clone(&self) -> Box<dyn Subscription<T>> {
//         Box::new(Clone::clone(self))
//     }
// }

// impl<T, F: FnMut(T) -> EnqueueResult + Send + Sync + Clone + 'static> Subscription<T> for F {
//     fn push(&mut self, value: T) -> EnqueueResult {
//         self(value)
//     }
//     fn clone(&self) -> Box<dyn Subscription<T>> {
//         Box::new(Clone::clone(self))
//     }
// }

// impl<T> Clone for Box<dyn Subscription<T>> {
//     fn clone(&self) -> Self {
//         Subscription::clone(self.deref())
//     }
// }

struct SubscriberInner<T> {
    queue: ArrayQueue<T>,
    notify: Notify,
    pub_count: AtomicUsize,
}

/// An essential companion to the `Publisher`.
///
/// Subscribers are bounded queues that can receive messages `T`
/// from multiple Publishers concurrently. To subscribe to a `Publisher`,
/// a `Subscriber` must create a subscription and pass that to the `Publisher`.
pub struct Subscriber<T> {
    inner: Arc<SubscriberInner<T>>,
}

impl<T: Clone + Send + 'static> Subscriber<T> {
    #[must_use]
    pub fn new(size: usize) -> Self {
        Self {
            inner: Arc::new(SubscriberInner {
                queue: ArrayQueue::new(size),
                notify: Notify::default(),
                pub_count: AtomicUsize::default(),
            }),
        }
    }

    pub fn get_size(&self) -> usize {
        self.inner.queue.capacity()
    }

    /// Receive some message (waiting if none are available), or `None` if all `Publishers` have been dropped.
    pub async fn recv_or_closed(&mut self) -> Option<T> {
        loop {
            if let Some(value) = self.inner.queue.pop() {
                return Some(value);
            }

            if self.inner.pub_count.load(Ordering::Acquire) == 0 {
                return None;
            }

            self.inner.notify.notified().await;
        }
    }

    /// Try to receive a message if one is available.
    pub fn try_recv(&mut self) -> Option<T> {
        self.inner.queue.pop()
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
    pub fn create_subscription(&self) -> DirectSubscription<T> {
        DirectSubscription {
            sub: Arc::downgrade(&self.inner),
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
    pub fn create_subscription(&self) -> DirectSubscription<T> {
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
