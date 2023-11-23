use std::ops::{Add, AddAssign};

use async_trait::async_trait;
use futures::{stream::FuturesUnordered, StreamExt};
use rand::{rngs::SmallRng, seq::SliceRandom, SeedableRng};
use tokio::sync::mpsc;

use super::{watched::WatchedSubscription, ChannelTrait, MappedChannel, Signal, ZippedChannel};

/// A subscription to a signal that stores every message that was received.
pub struct UnboundedSubscription<T> {
    pub(super) receivers: Vec<Box<dyn ChannelTrait<T>>>,
    pub(super) rng: SmallRng,
}

static_assertions::assert_impl_all!(UnboundedSubscription<()>: Send, Sync);

impl<T: Send + 'static> UnboundedSubscription<T> {
    /// Creates a subscription that will never produce a message.
    ///
    /// None subscriptions are considered closed.
    pub fn none() -> Self {
        Self {
            receivers: vec![],
            rng: SmallRng::from_entropy(),
        }
    }

    /// Waits for a message to be sent, unless the `Signal` has been dropped.
    pub async fn recv_or_closed(&mut self) -> Option<T> {
        let mut futures: FuturesUnordered<_> = self
            .receivers
            .iter_mut()
            .map(|x| x.recv_or_closed())
            .collect();

        futures.next().await.unwrap_or_default()
    }

    /// Waits for a message to be sent, regardless of the state of the `Signal`.
    ///
    /// If the `Signal` was dropped, this method will never return. This was
    /// considered to be an acceptable abstraction as `Node`s should have a
    /// limited view of the outside world, which includes not knowing if a
    /// `Signal` is still alive or not.
    pub async fn recv(&mut self) -> T {
        let mut futures: FuturesUnordered<_> =
            self.receivers.iter_mut().map(|x| x.recv()).collect();

        let Some(x) = futures.next().await else {
            std::future::pending::<()>().await;
            unreachable!()
        };
        x
    }

    /// Tries to receive a single value from the `Signal`.
    ///
    /// Returns `None` if there are no messages right now
    /// or the `Signal` is closed.
    pub fn try_recv(&mut self) -> Option<T> {
        self.receivers.shuffle(&mut self.rng);
        self.receivers
            .iter_mut()
            .filter_map(|x| x.try_recv())
            .next()
    }

    /// Changes the generic type of the signal that this subscription is for.
    ///
    /// This is done by applying a mapping function after a message is received.
    /// This mapping function is ran in an asynchronous context, so it should be
    /// non-blocking. Do note that the mapping function itself is not asynchronous
    /// and is multi-thread safe.
    ///
    /// There is also a non-zero cost to mapping on top of the mapping functions,
    /// so avoid having deeply mapped subscriptions. This is due to the lack of
    /// `AsyncFn` traits and/or lending functions.
    pub fn map<V: 'static>(
        mut self,
        mapper: impl FnMut(T) -> V + 'static + Send + Sync,
    ) -> UnboundedSubscription<V> {
        UnboundedSubscription {
            rng: SmallRng::from_rng(&mut self.rng).unwrap(),
            receivers: vec![Box::new(MappedChannel {
                source: Box::new(self),
                mapper: Box::new(mapper),
            })],
        }
    }

    /// Converts this subscription to a `WatchedSubscription`.
    ///
    /// This is a useful technique to simulate adding `WatchedSubscription`s
    /// together, since `WatchedSubscription`s cannot be added together once
    /// created. You can first add multiple `UnboundedSubscriptions` together,
    /// then call this method.
    pub async fn to_watched(mut self) -> WatchedSubscription<T>
    where
        T: Clone + Sync,
    {
        let mut signal = Signal::default();
        let sub = signal.get_ref().watch();
        tokio::spawn(async move {
            loop {
                let Some(msg) = self.recv_or_closed().await else {
                    break;
                };
                signal.set(msg);
            }
        });
        sub
    }

    /// Zips this subscription with the other given subscription.
    ///
    /// This is equivalent to the zipping iterators.
    pub fn zip<B>(mut self, other: UnboundedSubscription<B>) -> UnboundedSubscription<(T, B)>
    where
        T: Clone + Sync,
        B: Clone + Send + Sync + 'static,
    {
        UnboundedSubscription {
            rng: SmallRng::from_rng(&mut self.rng).unwrap(),
            receivers: vec![Box::new(ZippedChannel {
                source_a: Box::new(self),
                source_b: Box::new(other),
                item_a: None,
                item_b: None,
            })],
        }
    }
}

#[async_trait]
impl<T: Send + 'static> ChannelTrait<T> for mpsc::UnboundedReceiver<T> {
    fn source_count(&self) -> usize {
        1
    }

    async fn recv_or_closed(&mut self) -> Option<T> {
        self.recv().await
    }

    async fn recv(&mut self) -> T {
        match mpsc::UnboundedReceiver::recv(self).await {
            Some(x) => x,
            None => {
                std::future::pending::<()>().await;
                unreachable!()
            }
        }
    }

    fn try_recv(&mut self) -> Option<T> {
        mpsc::UnboundedReceiver::try_recv(self).ok()
    }
}

#[async_trait]
impl<T: Send + 'static> ChannelTrait<T> for UnboundedSubscription<T> {
    fn source_count(&self) -> usize {
        self.receivers.iter().map(|x| x.source_count()).sum()
    }

    async fn recv_or_closed(&mut self) -> Option<T> {
        UnboundedSubscription::recv_or_closed(self).await
    }

    async fn recv(&mut self) -> T {
        UnboundedSubscription::recv(self).await
    }

    fn try_recv(&mut self) -> Option<T> {
        UnboundedSubscription::try_recv(self)
    }
}

impl<T: Send + 'static> Add for UnboundedSubscription<T> {
    type Output = Self;

    fn add(mut self, mut rhs: Self) -> Self::Output {
        self.receivers.append(&mut rhs.receivers);
        self
    }
}

impl<T: Send + 'static> AddAssign for UnboundedSubscription<T> {
    fn add_assign(&mut self, mut rhs: Self) {
        self.receivers.append(&mut rhs.receivers);
    }
}
