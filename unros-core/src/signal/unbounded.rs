use std::ops::{Add, AddAssign};

use async_trait::async_trait;
use futures::{stream::FuturesUnordered, StreamExt};
use rand::{rngs::SmallRng, seq::SliceRandom, SeedableRng};
use tokio::sync::mpsc;
use tokio_rayon::rayon::prelude::{IntoParallelRefMutIterator, ParallelIterator};

use super::{watched::WatchedSubscription, ChannelTrait, MappedChannel, Signal};

pub struct UnboundedSubscription<T> {
    pub(super) receivers: Vec<Box<dyn ChannelTrait<T>>>,
    pub(super) rng: SmallRng,
}

static_assertions::assert_impl_all!(UnboundedSubscription<()>: Send, Sync);

impl<T: Send + 'static> UnboundedSubscription<T> {
    pub fn none() -> Self {
        Self {
            receivers: vec![],
            rng: SmallRng::from_entropy(),
        }
    }

    pub async fn recv_ex(&mut self) -> Option<T> {
        let mut futures: FuturesUnordered<_> =
            self.receivers.iter_mut().map(|x| x.recv_ex()).collect();

        futures.next().await.unwrap_or_default()
    }

    pub async fn recv(&mut self) -> T {
        let mut futures: FuturesUnordered<_> =
            self.receivers.iter_mut().map(|x| x.recv()).collect();

        let Some(x) = futures.next().await else {
            std::future::pending::<()>().await;
            unreachable!()
        };
        x
    }

    pub fn blocking_recv(&mut self) -> Option<T> {
        let mut out: Vec<_> = self
            .receivers
            .par_iter_mut()
            .filter_map(|x| x.blocking_recv())
            .take_any(1)
            .collect();
        out.pop()
    }

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

    pub async fn to_watched(mut self) -> WatchedSubscription<T>
    where
        T: Clone + Sync,
    {
        let mut signal = Signal::default();
        let sub = signal.get_ref().watch();
        tokio::spawn(async move {
            loop {
                let Some(msg) = self.recv_ex().await else {
                    break;
                };
                signal.set(msg);
            }
        });
        sub
    }
}

#[async_trait]
impl<T: Send + 'static> ChannelTrait<T> for mpsc::UnboundedReceiver<T> {
    fn source_count(&self) -> usize {
        1
    }

    async fn recv_ex(&mut self) -> Option<T> {
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

    fn blocking_recv(&mut self) -> Option<T> {
        mpsc::UnboundedReceiver::blocking_recv(self)
    }
}

#[async_trait]
impl<T: Send + 'static> ChannelTrait<T> for UnboundedSubscription<T> {
    fn source_count(&self) -> usize {
        self.receivers.iter().map(|x| x.source_count()).sum()
    }

    async fn recv_ex(&mut self) -> Option<T> {
        UnboundedSubscription::recv_ex(self).await
    }

    async fn recv(&mut self) -> T {
        UnboundedSubscription::recv(self).await
    }

    fn try_recv(&mut self) -> Option<T> {
        UnboundedSubscription::try_recv(self)
    }

    fn blocking_recv(&mut self) -> Option<T> {
        UnboundedSubscription::blocking_recv(self)
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
