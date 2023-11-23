use std::ops::Deref;

use async_trait::async_trait;
use tokio::sync::watch;

#[async_trait]
pub(super) trait WatchTrait<T>: Send + Sync + 'static {
    async fn get(&mut self) -> T;
    async fn wait_for_change(&mut self) -> T;

    fn get_or_empty(&mut self) -> Option<T>;
    async fn changed_or_closed(&mut self) -> Option<T>;
}

struct MappedWatched<T, S> {
    recv: Option<Box<dyn WatchTrait<S>>>,
    mapper: Box<dyn FnMut(S) -> T + Send + Sync>,
}

#[async_trait]
impl<T: 'static, S: 'static> WatchTrait<T> for MappedWatched<T, S> {
    async fn get(&mut self) -> T {
        if let Some(recv) = &mut self.recv {
            (self.mapper)(recv.get().await)
        } else {
            std::future::pending::<()>().await;
            unreachable!()
        }
    }

    async fn wait_for_change(&mut self) -> T {
        if let Some(recv) = &mut self.recv {
            (self.mapper)(recv.wait_for_change().await)
        } else {
            std::future::pending::<()>().await;
            unreachable!()
        }
    }

    fn get_or_empty(&mut self) -> Option<T> {
        self.recv
            .as_mut()
            .and_then(|x| x.get_or_empty())
            .map(|x| (self.mapper)(x))
    }

    async fn changed_or_closed(&mut self) -> Option<T> {
        self.recv
            .as_mut()?
            .changed_or_closed()
            .await
            .map(|x| (self.mapper)(x))
    }
}

struct ZippedWatched<A, B> {
    source_a: Option<Box<dyn WatchTrait<A>>>,
    item_a: Option<A>,
    source_b: Option<Box<dyn WatchTrait<B>>>,
    item_b: Option<B>,
}

#[async_trait]
impl<A: Clone + Send + Sync + 'static, B: Clone + Send + Sync + 'static> WatchTrait<(A, B)>
    for ZippedWatched<A, B>
{
    async fn get(&mut self) -> (A, B) {
        if self.source_a.is_none() || self.source_b.is_none() {
            std::future::pending::<()>().await;
            unreachable!()
        }
        loop {
            tokio::select! {
                new_item_a = self.source_a.as_mut().unwrap().get() => {
                    self.item_a = Some(new_item_a);
                }
                new_item_b = self.source_b.as_mut().unwrap().get() => {
                    self.item_b = Some(new_item_b);
                }
            }
            if let Some(item_a) = &self.item_a {
                if let Some(item_b) = &self.item_b {
                    break (item_a.clone(), item_b.clone());
                }
            }
        }
    }

    async fn wait_for_change(&mut self) -> (A, B) {
        if self.source_a.is_none() || self.source_b.is_none() {
            std::future::pending::<()>().await;
            unreachable!()
        }
        loop {
            tokio::select! {
                new_item_a = self.source_a.as_mut().unwrap().wait_for_change() => {
                    self.item_a = Some(new_item_a);
                }
                new_item_b = self.source_b.as_mut().unwrap().wait_for_change() => {
                    self.item_b = Some(new_item_b);
                }
            }
            if let Some(item_a) = &self.item_a {
                if let Some(item_b) = &self.item_b {
                    break (item_a.clone(), item_b.clone());
                }
            }
        }
    }

    fn get_or_empty(&mut self) -> Option<(A, B)> {
        if self.source_a.is_none() || self.source_b.is_none() {
            return None;
        }
        if let Some(new_item_a) = self.source_a.as_mut().unwrap().get_or_empty() {
            self.item_a = Some(new_item_a);
        }
        if let Some(new_item_b) = self.source_b.as_mut().unwrap().get_or_empty() {
            self.item_b = Some(new_item_b);
        }
        if let Some(item_a) = &self.item_a {
            if let Some(item_b) = &self.item_b {
                return Some((item_a.clone(), item_b.clone()));
            }
        }
        None
    }

    async fn changed_or_closed(&mut self) -> Option<(A, B)> {
        if self.source_a.is_none() || self.source_b.is_none() {
            return None;
        }
        loop {
            tokio::select! {
                new_item_a = self.source_a.as_mut().unwrap().changed_or_closed() => {
                    let new_item_a = new_item_a?;
                    self.item_a = Some(new_item_a);
                }
                new_item_b = self.source_b.as_mut().unwrap().changed_or_closed() => {
                    let new_item_b = new_item_b?;
                    self.item_b = Some(new_item_b);
                }
            }
            if let Some(item_a) = &self.item_a {
                if let Some(item_b) = &self.item_b {
                    break Some((item_a.clone(), item_b.clone()));
                }
            }
        }
    }
}

/// A subscription to a signal that stores just the last message that was received.
pub struct WatchedSubscription<T> {
    pub(super) recv: Option<Box<dyn WatchTrait<T>>>,
}

static_assertions::assert_impl_all!(WatchedSubscription<()>: Send, Sync);

impl<T: 'static> WatchedSubscription<T> {
    /// Creates a subscription that will never be updated.
    ///
    /// None subscriptions are considered closed.
    pub fn none() -> Self {
        Self { recv: None }
    }

    /// Gets the current value in this subscription.
    ///
    /// If the `Signal` has yet to produce a message,
    /// this method will wait until one is produced.
    /// If the `Signal` is dropped before a message is
    /// sent, this method will wait forever.
    pub async fn get(&mut self) -> T {
        if let Some(recv) = &mut self.recv {
            recv.get().await
        } else {
            std::future::pending::<()>().await;
            unreachable!()
        }
    }

    /// Waits for the stored value to change, or for the
    /// `Signal` to be closed.
    pub async fn changed_or_closed(&mut self) -> Option<T> {
        self.recv.as_mut()?.changed_or_closed().await
    }

    /// Waits for the `Signal` to be changed.
    ///
    /// If the `Signal` is dropped, this method will wait
    /// forever.
    pub async fn wait_for_change(&mut self) -> T {
        if let Some(recv) = &mut self.recv {
            recv.wait_for_change().await
        } else {
            std::future::pending::<()>().await;
            unreachable!()
        }
    }

    /// Gets the current value in this subscription.
    ///
    /// If the `Signal` has yet to produce a message,
    /// this method will return `None`.
    /// If the `Signal` is dropped before a message is
    /// sent, this method will return `None`.
    pub fn get_or_empty(&mut self) -> Option<T> {
        self.recv.as_mut().and_then(|x| x.get_or_empty())
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
        self,
        mapper: impl FnMut(T) -> V + 'static + Send + Sync,
    ) -> WatchedSubscription<V> {
        WatchedSubscription {
            recv: Some(Box::new(MappedWatched {
                recv: self.recv,
                mapper: Box::new(mapper),
            })),
        }
    }

    /// Zips this subscription with the other given subscription.
    ///
    /// Only one of the subscriptions have to identify a change for the
    /// whole subscription to be considered changed.
    pub fn zip<B>(self, other: WatchedSubscription<B>) -> WatchedSubscription<(T, B)>
    where
        T: Clone + Send + Sync,
        B: Clone + Send + Sync + 'static,
    {
        WatchedSubscription {
            recv: Some(Box::new(ZippedWatched {
                source_a: self.recv,
                source_b: other.recv,
                item_a: None,
                item_b: None,
            })),
        }
    }
}

#[async_trait]
impl<T: Clone + Send + Sync + 'static> WatchTrait<T> for watch::Receiver<Option<T>> {
    async fn get(&mut self) -> T {
        if let Some(x) = self.borrow_and_update().deref() {
            return x.clone();
        }
        self.wait_for_change().await
    }

    async fn wait_for_change(&mut self) -> T {
        if self.changed().await.is_err() {
            std::future::pending::<()>().await;
            unreachable!()
        } else {
            self.borrow().as_ref().unwrap().clone()
        }
    }

    fn get_or_empty(&mut self) -> Option<T> {
        self.borrow_and_update().as_ref().map(Clone::clone)
    }

    async fn changed_or_closed(&mut self) -> Option<T> {
        if self.changed().await.is_err() {
            None
        } else {
            Some(__self.borrow().as_ref().unwrap().clone())
        }
    }
}
