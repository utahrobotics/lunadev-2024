//! Signals are an essential component of many frameworks in many different disciplines.
//!
//! This is one of the aspects taken from `ROS` that have been greatly improved. We offer
//! an `API` for signals that is more similar to `Rust`'s iterators. Signals are analagous
//! to `Rust`'s channels in that they do not trigger code, unlike `ROS` subscriber callbacks.
//! Signals in this crate also differ by having 3 different ways that they can be subscribed
//! to depending on the needs of the code using it.

use std::{sync::{Arc, Weak}, path::Path, io::Write, ops::{Deref, DerefMut}};

use crossbeam::queue::ArrayQueue;
use futures::{stream::FuturesUnordered, StreamExt};
use rand::{rngs::SmallRng, seq::SliceRandom, SeedableRng};
use tokio::sync::watch;

use crate::logging::{dump::DataDump, START_TIME};

/// An essential component that promotes separation of concerns, and is
/// an intrinsic element of the ROS framework.
///
/// Signals provide a simple way to send a message to receivers, much
/// like Rust's channels. However, signals provide 3 different forms
/// of subscriptions:
///
/// 1. **Unbounded**<br>
///    The subscription will contain all sent messages forever and ever
///    if it is never read from. If the receiving code can handle the same
///    throughput that this signal produces, but the buffer size needed is
///    unknown or highly variable, use this subscription.
///
/// 2. ****<br>
///    The subscription will hold a limited number of messages before
///    ignoring future messages until the current ones are read. If the
///    receiving code may not be able to handle the same throughput that
///    this signal produces, and can tolerate lost messages, use this
///    subscription.
///
/// 3. **Watched**<br>
///    The subscription will only keep track of the latest message. If you
///    only need the latest value, use this subscription.
///
/// Signals make numerous clones of the values it will send, so you should
/// use a type `T` that is cheap to clone with this signal. A good default is
/// `Arc`. Since Nodes will often be used from different threads, the type `T`
/// should also be `Send + Sync`, but this is not a requirement.
pub struct Publisher<T> {
    bounded_queues: Vec<Box<dyn Queue<T>>>,
    watch_sender: watch::Sender<()>,
}

impl<T> Default for Publisher<T> {
    fn default() -> Self {
        Self {
            bounded_queues: Default::default(),
            watch_sender: watch::channel(()).0,
        }
    }
}

impl<T: Clone> Publisher<T> {
    /// Sets a value into this signal.
    ///
    /// Unbounded and  Subscriptions will receive this value, and
    /// Watched Subscriptions will replace their current values with this.
    ///
    /// This method takes an immutable reference as a convenience, but this
    /// can be abused by nodes that do not own this signal. As a user of this
    /// signal, ie. you are accessing this signal just to subscribe to it,
    /// do not call this method ever. This will lead to spaghetti code. Only
    /// the node that owns this signal should call this method.
    pub fn set(&mut self, value: T) {
        self.bounded_queues.retain(|queue|
            queue.push(value.clone())
        );
        self.watch_sender.send_replace(());
    }

    pub fn accept_subscription(&mut self, sub: Subscription<T>) {
        self.bounded_queues.push(sub.queue);
        (sub.subscriber)(self.watch_sender.subscribe());
    }
}


trait Queue<T>: Send + Sync {
    fn push(&self, value: T) -> bool;
}


impl<T: Send> Queue<T> for Weak<ArrayQueue<T>> {
    fn push(&self, value: T) -> bool {
        if let Some(queue) = self.upgrade() {
            queue.force_push(value);
            true
        } else {
            false
        }
    }
}


impl<T, F: Fn(T) -> bool + Send + Sync> Queue<T> for F {
    fn push(&self, value: T) -> bool {
        self(value)
    }
}


struct SubscriptionInner<T> {
    queue: Arc<ArrayQueue<T>>,
    watch: watch::Receiver<()>
}


pub struct Subscriber<T> {
    subscriptions: Vec<SubscriptionInner<T>>,
    rng: SmallRng
}


impl<T> Default for Subscriber<T> {
    fn default() -> Self {
        Self { subscriptions: Default::default(), rng: SmallRng::from_entropy() }
    }
}


pub struct Subscription<'a, T> {
    subscriber: Box<dyn FnOnce(watch::Receiver<()>) + 'a>,
    queue: Box<dyn Queue<T>>
}


impl<T: Clone + Send + 'static> Subscriber<T> {
    pub async fn recv_or_empty(&mut self) -> Option<T> {
        let mut futs = FuturesUnordered::new();
        self.subscriptions.shuffle(&mut self.rng);

        for sub in &mut self.subscriptions {
            if let Some(x) = sub.queue.pop() {
                sub.watch.mark_changed();
                return Some(x);
            }
            futs.push(async {
                if sub.watch.changed().await.is_ok() {
                    Some(sub.queue.pop().unwrap())
                } else {
                    None
                }
            });
        }

        while let Some(x) = futs.next().await {
            if x.is_some() {
                return x;
            }
        }
        None
    }

    pub fn try_recv(&mut self) -> Option<T> {
        self.subscriptions.shuffle(&mut self.rng);

        for sub in &mut self.subscriptions {
            if let Some(x) = sub.queue.pop() {
                sub.watch.mark_changed();
                return Some(x);
            }
        }

        None
    }

    pub async fn recv(&mut self) -> T {
        if let Some(x) = self.recv_or_empty().await {
            x
        } else {
            std::future::pending().await
        }
    }

    pub async fn into_logger(mut self, mut display: impl FnMut(T) -> String + Send + 'static, path: impl AsRef<Path>) -> std::io::Result<()> {
        let mut dump = DataDump::new_file(path).await?;
        tokio::spawn(async move {
            loop {
                let Some(value) = self.recv_or_empty().await else { break; };
                let secs = START_TIME.get().unwrap().elapsed().as_secs_f32();
                writeln!(
                    dump,
                    "[{:0>1}:{:.2}] {}",
                    (secs / 60.0).floor(),
                    secs % 60.0,
                    display(value)
                ).unwrap();
            }
        });
        Ok(())
    }

    ///
    /// # Panics
    /// Panics if size is 0.
    pub fn create_subscription(&mut self, size: usize) -> Subscription<T> {
        let queue = Arc::new(ArrayQueue::new(size));
        Subscription {
            queue: Box::new(Arc::downgrade(&queue)),
            subscriber: Box::new(move |watch| self.subscriptions.push(SubscriptionInner { queue, watch })),
        }
    }

    pub async fn into_watch(mut self) -> WatchSubscriber<T> {
        WatchSubscriber {
            value: self.recv().await,
            inner: self
        }
    }

    pub fn try_into_watch(mut self) -> Result<WatchSubscriber<T>, Self> {
        if let Some(value) = self.try_recv() {
            Ok(WatchSubscriber {
                            value,
                            inner: self
                        })
        } else {
            Err(self)
        }
    }

    pub async fn into_watch_or_empty(mut self) -> Result<WatchSubscriber<T>, Self> {
        if let Some(value) = self.recv_or_empty().await {
            Ok(WatchSubscriber {
                            value,
                            inner: self
                        })
        } else {
            Err(self)
        }
    }
}


impl<'a, T: 'static> Subscription<'a, T> {
    pub fn map<V>(self, map: impl Fn(V) -> T + Send + Sync + 'static) -> Subscription<'a, V> {
        Subscription {
            subscriber: self.subscriber,
            queue: Box::new(move |x| self.queue.push(map(x))),
        }
    }
}

pub struct WatchSubscriber<T> {
    inner: Subscriber<T>,
    value: T
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
    pub async fn update(sub: &mut Self) {
        if !Self::try_update(sub) {
            sub.value = sub.inner.recv().await;
        }
    }

    pub async fn update_or_closed(sub: &mut Self) -> bool {
        if Self::try_update(sub) {
            true
        } else if let Some(x) = sub.inner.recv_or_empty().await {
            sub.value = x;
            true
        } else {
            false
        }
    }
    
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
}