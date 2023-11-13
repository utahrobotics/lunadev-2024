use std::{collections::hash_map::Entry, num::NonZeroU32};

use async_trait::async_trait;
use fxhash::FxHashMap;
use rand::{rngs::SmallRng, SeedableRng};
use tokio::sync::{broadcast, mpsc, watch};

use self::{
    bounded::BoundedSubscription, unbounded::UnboundedSubscription, watched::WatchedSubscription,
};

pub mod bounded;
pub mod unbounded;
pub mod watched;

#[async_trait]
trait ChannelTrait<T>: Send + Sync + 'static {
    fn source_count(&self) -> usize;

    async fn recv_ex(&mut self) -> Option<T>;
    async fn recv(&mut self) -> T;
    fn blocking_recv(&mut self) -> Option<T>;

    fn try_recv(&mut self) -> Option<T>;
}

struct MappedChannel<T, S> {
    source: Box<dyn ChannelTrait<S>>,
    mapper: Box<dyn FnMut(S) -> T + Send + Sync>,
}

#[async_trait]
impl<T: 'static, S: 'static> ChannelTrait<T> for MappedChannel<T, S> {
    fn source_count(&self) -> usize {
        self.source.source_count()
    }

    async fn recv_ex(&mut self) -> Option<T> {
        self.source.recv_ex().await.map(|x| (self.mapper)(x))
    }

    async fn recv(&mut self) -> T {
        (self.mapper)(self.source.recv().await)
    }

    fn blocking_recv(&mut self) -> Option<T> {
        self.source.blocking_recv().map(|x| (self.mapper)(x))
    }

    fn try_recv(&mut self) -> Option<T> {
        self.source.try_recv().map(|x| (self.mapper)(x))
    }
}

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
/// 2. **Bounded**<br>
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
pub struct Signal<T> {
    unbounded_senders: Vec<mpsc::UnboundedSender<T>>,
    bounded_senders: FxHashMap<NonZeroU32, broadcast::Sender<T>>,
    watch_sender: Option<watch::Sender<Option<T>>>,
}

impl<T> Default for Signal<T> {
    fn default() -> Self {
        Self {
            unbounded_senders: Default::default(),
            bounded_senders: Default::default(),
            watch_sender: None,
        }
    }
}

impl<T: Clone> Signal<T> {
    pub fn get_ref(&mut self) -> SignalRef<T> {
        SignalRef(self)
    }

    /// Sets a value into this signal.
    ///
    /// Unbounded and Bounded Subscriptions will receive this value, and
    /// Watched Subscriptions will replace their current values with this.
    ///
    /// This method takes an immutable reference as a convenience, but this
    /// can be abused by nodes that do not own this signal. As a user of this
    /// signal, ie. you are accessing this signal just to subscribe to it,
    /// do not call this method ever. This will lead to spaghetti code. Only
    /// the node that owns this signal should call this method.
    pub fn set(&self, value: T) {
        for sender in &self.unbounded_senders {
            let _ = sender.send(value.clone());
        }
        for sender in self.bounded_senders.values() {
            let _ = sender.send(value.clone());
        }
        if let Some(watch_sender) = &self.watch_sender {
            watch_sender.send_replace(Some(value));
        }
    }
}

pub struct SignalRef<'a, T>(&'a mut Signal<T>);

impl<'a, T: Clone + Send + Sync + 'static> SignalRef<'a, T> {
    pub fn subscribe_unbounded(&mut self) -> UnboundedSubscription<T> {
        let (sender, recv) = mpsc::unbounded_channel();
        self.0.unbounded_senders.push(sender);

        UnboundedSubscription {
            receivers: vec![Box::new(recv)],
            rng: SmallRng::from_entropy(),
        }
    }

    pub fn subscribe_bounded<const SIZE: u32>(&mut self) -> BoundedSubscription<T, SIZE> {
        let recv = match self.0.bounded_senders.entry(
            NonZeroU32::new(SIZE).expect("Size of BoundedSubscription should be greater than 0"),
        ) {
            Entry::Occupied(x) => x.get().subscribe(),
            Entry::Vacant(x) => {
                let (sender, recv) = broadcast::channel(SIZE as usize);
                x.insert(sender);
                recv
            }
        };
        BoundedSubscription {
            receivers: vec![Box::new(recv)],
            rng: SmallRng::from_entropy(),
        }
    }

    pub fn watch(&mut self) -> WatchedSubscription<T> {
        let watch_sender = self
            .0
            .watch_sender
            .get_or_insert_with(|| watch::channel(None).0);
        WatchedSubscription {
            recv: Some(Box::new(watch_sender.subscribe())),
        }
    }
}
