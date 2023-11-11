use std::ops::Add;

use async_trait::async_trait;
use futures::{stream::FuturesUnordered, StreamExt};
use rand::{seq::SliceRandom, rngs::SmallRng, SeedableRng};
use tokio::sync::mpsc;

use super::{ChannelTrait, MappedChannel};

pub struct UnboundedSubscription<T> {
    pub(super) receivers: Vec<Box<dyn ChannelTrait<T>>>,
    pub(super) rng: SmallRng
}

static_assertions::assert_impl_all!(UnboundedSubscription<()>: Send, Sync);

impl<T: 'static> UnboundedSubscription<T> {
    pub fn none() -> Self {
        Self {
            receivers: vec![],
            rng: SmallRng::from_entropy()
        }
    }

    pub async fn recv(&mut self) -> T {
        let mut futures: FuturesUnordered<_> = self.receivers.iter_mut().map(|x| x.recv()).collect();

        let Some(x) = futures.next().await else {
                std::future::pending::<()>().await;
                unreachable!()
        };
        x
    }

    pub fn try_recv(&mut self) -> Option<T> {
        self.receivers.shuffle(&mut self.rng);
        self
            .receivers
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
            receivers: vec![Box::new(MappedChannel { source: Box::new(self), mapper: Box::new(mapper) })],
        }
    }
}


#[async_trait]
impl<T: Send + 'static> ChannelTrait<T> for mpsc::UnboundedReceiver<T> {
    fn source_count(&self) -> usize {
        1
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

    fn try_recv(&mut self) -> Option<T>  {
        mpsc::UnboundedReceiver::try_recv(self).ok()
    }
}


#[async_trait]
impl<T: 'static> ChannelTrait<T> for UnboundedSubscription<T> {
    fn source_count(&self) -> usize {
        self.receivers
            .iter()
            .map(|x| x.source_count())
            .sum()
    }

    async fn recv(&mut self) -> T {
        self.recv().await
    }

    fn try_recv(&mut self) -> Option<T>  {
        self.try_recv()
    }
}


impl<T: 'static> Add for UnboundedSubscription<T> {
    type Output = Self;

    fn add(mut self, rhs: Self) -> Self::Output {
        self.receivers.push(Box::new(rhs));
        self
    }
}