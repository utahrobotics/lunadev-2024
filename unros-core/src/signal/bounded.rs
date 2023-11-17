use std::ops::{Add, AddAssign};

use async_trait::async_trait;
use futures::{stream::FuturesUnordered, StreamExt};
use rand::{rngs::SmallRng, seq::SliceRandom, SeedableRng};
use tokio::sync::broadcast;

use super::{watched::WatchedSubscription, ChannelTrait, MappedChannel, Signal};

pub struct BoundedSubscription<T, const SIZE: u32> {
    pub(super) receivers: Vec<Box<dyn ChannelTrait<Result<T, u64>>>>,
    pub(super) rng: SmallRng,
}

static_assertions::assert_impl_all!(BoundedSubscription<(), 1>: Send, Sync);

impl<T: Send + 'static, const SIZE: u32> BoundedSubscription<T, SIZE> {
    pub fn none() -> Self {
        Self {
            receivers: vec![],
            rng: SmallRng::from_entropy(),
        }
    }

    pub async fn recv_ex(&mut self) -> Option<Result<T, u64>> {
        let mut futures: FuturesUnordered<_> =
            self.receivers.iter_mut().map(|x| x.recv_ex()).collect();

        let Some(x) = futures.next().await else {
            std::future::pending::<()>().await;
            unreachable!()
        };
        x
    }

    pub async fn recv(&mut self) -> Result<T, u64> {
        let mut futures: FuturesUnordered<_> =
            self.receivers.iter_mut().map(|x| x.recv()).collect();

        let Some(x) = futures.next().await else {
            std::future::pending::<()>().await;
            unreachable!()
        };
        x
    }

    pub fn try_recv(&mut self) -> Option<Result<T, u64>> {
        self.receivers.shuffle(&mut self.rng);
        self.receivers
            .iter_mut()
            .filter_map(|x| x.try_recv())
            .next()
    }

    // pub fn blocking_recv(&mut self) -> Option<Result<T, u64>> {
    //     let mut out: Vec<_> = self
    //         .receivers
    //         .par_iter_mut()
    //         .filter_map(|x| x.blocking_recv())
    //         .take_any(1)
    //         .collect();
    //     out.pop()
    // }

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
        mut mapper: impl FnMut(T) -> V + 'static + Send + Sync,
    ) -> BoundedSubscription<V, SIZE> {
        BoundedSubscription {
            rng: SmallRng::from_rng(&mut self.rng).unwrap(),
            receivers: vec![Box::new(MappedChannel {
                source: Box::new(self),
                mapper: Box::new(move |x| x.map(|x| mapper(x))),
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
                let Ok(msg) = msg else {
                    continue;
                };
                signal.set(msg);
            }
        });
        sub
    }
}

#[async_trait]
impl<T: Send + 'static, const SIZE: u32> ChannelTrait<Result<T, u64>>
    for BoundedSubscription<T, SIZE>
{
    fn source_count(&self) -> usize {
        self.receivers.iter().map(|x| x.source_count()).sum()
    }

    async fn recv_ex(&mut self) -> Option<Result<T, u64>> {
        BoundedSubscription::recv_ex(self).await
    }

    async fn recv(&mut self) -> Result<T, u64> {
        BoundedSubscription::recv(self).await
    }

    // fn blocking_recv(&mut self) -> Option<Result<T, u64>> {
    //     BoundedSubscription::blocking_recv(self)
    // }

    fn try_recv(&mut self) -> Option<Result<T, u64>> {
        BoundedSubscription::try_recv(self)
    }
}

impl<T: Send + 'static, const SIZE: u32> Add for BoundedSubscription<T, SIZE> {
    type Output = Self;

    fn add(mut self, mut rhs: Self) -> Self::Output {
        self.receivers.append(&mut rhs.receivers);
        self
    }
}

impl<T: Send + 'static, const SIZE: u32> AddAssign for BoundedSubscription<T, SIZE> {
    fn add_assign(&mut self, mut rhs: Self) {
        self.receivers.append(&mut rhs.receivers);
    }
}

#[async_trait]
impl<T: Clone + Send + 'static> ChannelTrait<Result<T, u64>> for broadcast::Receiver<T> {
    fn source_count(&self) -> usize {
        1
    }

    async fn recv_ex(&mut self) -> Option<Result<T, u64>> {
        match broadcast::Receiver::recv(self).await {
            Ok(x) => Some(Ok(x)),
            Err(broadcast::error::RecvError::Lagged(n)) => Some(Err(n)),
            Err(broadcast::error::RecvError::Closed) => None,
        }
    }

    async fn recv(&mut self) -> Result<T, u64> {
        match broadcast::Receiver::recv(self).await {
            Ok(x) => Ok(x),
            Err(broadcast::error::RecvError::Lagged(n)) => Err(n),
            Err(broadcast::error::RecvError::Closed) => {
                std::future::pending::<()>().await;
                unreachable!()
            }
        }
    }

    fn try_recv(&mut self) -> Option<Result<T, u64>> {
        match broadcast::Receiver::try_recv(self) {
            Ok(x) => Some(Ok(x)),
            Err(broadcast::error::TryRecvError::Lagged(n)) => Some(Err(n)),
            Err(_) => None,
        }
    }

    // fn blocking_recv(&mut self) -> Option<Result<T, u64>> {
    //     match broadcast::Receiver::blocking_recv(self) {
    //         Ok(x) => Some(Ok(x)),
    //         Err(broadcast::error::RecvError::Closed) => None,
    //         Err(broadcast::error::RecvError::Lagged(n)) => Some(Err(n)),
    //     }
    // }
}
