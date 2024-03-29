//! Publishers and Subscribers are an essential component of many frameworks in many different disciplines.
//!
//! This is one of the aspects taken from `ROS` that have been greatly improved. We offer
//! an `API` for pubsub that is more similar to `Rust`'s iterators. Signals are analagous
//! to `Rust`'s channels in that they do not trigger code, unlike `ROS` subscriber callbacks.

use std::{
    io::Write,
    marker::PhantomData,
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

use self::subs::{BoxedSubscription, DirectSubscription, PublisherToken, Subscription};

/// An essential component that promotes separation of concerns, and is
/// an intrinsic element of the ROS framework.
///
/// Publishers provide a simple way to send a message to receivers, much
/// like Rust's channels. These are analagous to single-producer-multi-consumer
/// channels, except that a `Publisher` can be a producer to many different consumers.
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
    /// Sets a value into this `Publisher`, allowing it to be received by Subscribers.
    ///
    /// Only the node that owns this `Publisher` should call this method for hygiene.
    pub fn set(&self, value: T) -> bool {
        let count = self.subs.len();
        if count == 0 {
            return false;
        }
        for _ in 0..count {
            // This is the only place we pop from subs, so we are guaranteed
            // to always have at least self.subs.len() to elements to pop
            let mut sub = self.subs.pop().unwrap();
            if sub.push(value.clone(), PublisherToken(PhantomData)) {
                self.subs.push(sub);
            }
        }
        true
    }
}

impl<T> Publisher<T> {
    /// Accepts a given subscription, allowing the corresponding `Subscriber` to
    /// receive new messages.
    pub fn accept_subscription(&self, sub: impl Subscription<Item = T> + Send + 'static) {
        sub.increment_publishers(PublisherToken(PhantomData));
        self.subs.push(Box::new(sub));
    }

    /// Gets a reference to this `Publisher` that is safe to share publicly since
    /// users of that reference cannot set values into it.
    pub fn get_ref(&self) -> PublisherRef<T> {
        PublisherRef {
            subs: Arc::downgrade(&self.subs),
        }
    }

    pub fn get_sub_count(&self) -> usize {
        self.subs.len()
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
            sub.decrement_publishers(PublisherToken(PhantomData));
        }
    }
}

/// Similar to a `Publisher`, except that it can only publish to one `Subscriber`,
/// eliminating the need for `T` to implement `Clone`.
///
/// As such, MonoPublishers must be created directly from Subscriptions using `From`
/// and `Into`.
pub struct MonoPublisher<T, S: Subscription<Item = T> = BoxedSubscription<T>> {
    sub: Option<S>,
}

impl<T, S: Subscription<Item = T>> From<S> for MonoPublisher<T, S> {
    fn from(sub: S) -> Self {
        sub.increment_publishers(PublisherToken(PhantomData));
        Self { sub: Some(sub) }
    }
}

impl<T, S: Subscription<Item = T>> MonoPublisher<T, S> {
    /// Sets a value into this `MonoPublisher`, allowing it to be received by a Subscriber.
    ///
    /// Only the node that owns this `MonoPublisher` should call this method for hygiene.
    pub fn set(&mut self, value: T) {
        if let Some(sub) = &mut self.sub {
            if !sub.push(value, PublisherToken(PhantomData)) {
                self.sub = None;
            }
        }
    }

    pub fn get_sub_count(&self) -> usize {
        if self.sub.is_some() {
            1
        } else {
            0
        }
    }

    /// Creates a new `MonoPublisher` that is not connected to any `Subscriber`.
    ///
    /// Since `MonoPublishers` cannot accept new subscriptions, this `MonoPublisher` will
    /// never publish any values ever. This can be used in scenarios where you have a `Option<MonoPublisher>`
    /// but would rather work with just a `MonoPublisher`.
    pub fn new() -> Self {
        Self { sub: None }
    }
}

impl<T, S: Subscription<Item = T>> Drop for MonoPublisher<T, S> {
    fn drop(&mut self) {
        if let Some(sub) = &self.sub {
            sub.decrement_publishers(PublisherToken(PhantomData));
        }
    }
}

/// A non-owning reference to a `Publisher` that cannot be used to set values into
/// the `Publisher`.
///
/// Sharing `&Publisher` publicly allows users of the reference to set a value into it
/// which is in violation of how pubsub should work. Only one "thing" should ever be able
/// to provide values.
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
        sub.increment_publishers(PublisherToken(PhantomData));
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

impl<T: Send + 'static> Subscriber<T> {
    /// Creates a new `Subscriber` that can store at most `size` elements inside.
    ///
    /// # Panics
    /// Panics if `size` is 0.
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

    /// Gets the size of the `Subscriber`. This corresponds to the number of elements this
    /// `Subscriber` can store, and *not* the number of elements currently in the `Subscriber`.
    pub fn get_size(&self) -> usize {
        self.inner.queue.capacity()
    }

    pub fn get_pub_count(&self) -> usize {
        self.inner.pub_count.load(Ordering::Acquire)
    }

    /// Receive some message (waiting if none are available), or `None` if all `Publishers` have been dropped.
    pub async fn recv_or_closed(&self) -> Option<T> {
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
    pub fn try_recv(&self) -> Option<T> {
        self.inner.queue.pop()
    }

    /// Wait until a message is received, even if all `Publisher`s have been dropped.
    pub async fn recv(&self) -> T {
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
        self,
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
    pub async fn into_watch(self) -> WatchSubscriber<T> {
        WatchSubscriber {
            value: self.recv().await,
            inner: self,
        }
    }

    /// Tries to convert this `Subscriber` into a `WatchSubscriber`.
    ///
    /// If no message is available, this `Subscriber` will be returned.
    pub fn try_into_watch(self) -> Result<WatchSubscriber<T>, Self> {
        if let Some(value) = self.try_recv() {
            Ok(WatchSubscriber { value, inner: self })
        } else {
            Err(self)
        }
    }

    /// Convert this `Subscriber` into a `WatchSubscriber`.
    ///
    /// Wait until a message is received. If all `Publisher`s have been dropped this `Subscriber` will be returned.
    pub async fn into_watch_or_closed(self) -> Result<WatchSubscriber<T>, Self> {
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
