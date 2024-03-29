use std::{borrow::Cow, hash::Hasher, marker::PhantomData, sync::Arc};

use bitcode::{Decode, Encode};
use fxhash::{FxHashMap, FxHasher};
use unros::pubsub::{
    subs::{DirectSubscription, Subscription},
    Publisher, PublisherRef,
};

use crate::{peer::NetworkPublisher, NetworkPeer};

pub struct Channel<T> {
    channel_id: u8,
    received_packets: PublisherRef<Result<T, Arc<bitcode::Error>>>,
    packets_to_send: DirectSubscription<(Box<[u8]>, PacketMode)>,
}

impl<T> Clone for Channel<T> {
    fn clone(&self) -> Self {
        Self {
            channel_id: self.channel_id,
            received_packets: self.received_packets.clone(),
            packets_to_send: self.packets_to_send.clone(),
        }
    }
}

impl<T: Decode + Send + 'static> Channel<T> {
    pub fn accept_subscription(
        &self,
        sub: impl Subscription<Item = Result<T, Arc<bitcode::Error>>> + Send + 'static,
    ) {
        self.received_packets.accept_subscription(sub);
    }

    pub fn accept_subscription_or_closed(
        &self,
        sub: impl Subscription<Item = Result<T, Arc<bitcode::Error>>> + Send + 'static,
    ) -> bool {
        self.received_packets.accept_subscription_or_closed(sub)
    }
}

impl<T: Encode> Channel<T> {
    pub fn create_reliable_subscription(&self) -> impl Subscription<Item = T> {
        let channel_id = self.channel_id;
        self.packets_to_send.clone().map(move |value| {
            let mut data = bitcode::encode(&value).expect("Failed to serialize value");
            data.push(channel_id);
            (data.into(), PacketMode::ReliableSequenced)
        })
    }

    pub fn create_unreliable_subscription(&self) -> impl Subscription<Item = T> {
        let channel_id = self.channel_id;
        self.packets_to_send.clone().map(move |value| {
            let mut data = bitcode::encode(&value).expect("Failed to serialize value");
            data.push(channel_id);
            (data.into(), PacketMode::UnreliableUnsequenced)
        })
    }
}

#[derive(Debug)]
pub struct NegotiationError {
    old: Cow<'static, str>,
    new: Cow<'static, str>,
}

impl std::fmt::Display for NegotiationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "The given channel name {} conflicts with {}. Either change the name or change the `seed`", self.new, self.old)
    }
}

pub struct Negotiation<T: FromPeer> {
    pub(super) channel_ids: T::Ids,
    _phantom: PhantomData<T>,
}

impl<T: FromPeer> Negotiation<T> {
    pub fn new(negotiation: T, seed: usize) -> Result<Self, NegotiationError> {
        let mut seen = FxHashMap::default();
        Ok(Self {
            channel_ids: negotiation.get_ids(seed, &mut seen)?,
            _phantom: PhantomData,
        })
    }
}

impl<T: FromPeer> Clone for Negotiation<T> {
    fn clone(&self) -> Self {
        Self {
            channel_ids: self.channel_ids,
            _phantom: PhantomData,
        }
    }
}

impl<T: FromPeer> Copy for Negotiation<T> {}

pub trait FromPeer {
    type Ids: Copy;
    type Product;

    fn from_peer(
        peer: &NetworkPeer,
        ids: &Self::Ids,
        pubs: &mut FxHashMap<u8, NetworkPublisher>,
    ) -> Self::Product;
    fn get_ids(
        &self,
        seed: usize,
        seen: &mut FxHashMap<u8, Cow<'static, str>>,
    ) -> Result<Self::Ids, NegotiationError>;
}

pub struct ChannelNegotiation<T> {
    name: Cow<'static, str>,
    _phantom: PhantomData<T>,
}

impl<T> ChannelNegotiation<T> {
    pub fn new(name: impl Into<Cow<'static, str>>) -> Self {
        Self {
            name: name.into(),
            _phantom: PhantomData,
        }
    }
}

impl<T: Decode + Clone + 'static> FromPeer for ChannelNegotiation<T> {
    type Ids = u8;
    type Product = Channel<T>;

    fn from_peer(
        peer: &NetworkPeer,
        ids: &Self::Ids,
        pubs: &mut FxHashMap<u8, NetworkPublisher>,
    ) -> Self::Product {
        let pub_received_packets = Publisher::default();
        let recv_packets_sub = pub_received_packets.get_ref();
        let pub_received_packets = Arc::new(pub_received_packets);
        let pub_received_packets2 = pub_received_packets.clone();

        pubs.insert(
            *ids,
            NetworkPublisher {
                setter: Box::new(move |bytes| {
                    pub_received_packets.set(bitcode::decode(&bytes).map_err(Arc::new));
                }),
                valid: Box::new(move || pub_received_packets2.get_sub_count() > 0),
            },
        );

        Channel {
            channel_id: *ids,
            received_packets: recv_packets_sub,
            packets_to_send: peer.packets_to_send.clone(),
        }
    }

    fn get_ids(
        &self,
        seed: usize,
        seen: &mut FxHashMap<u8, Cow<'static, str>>,
    ) -> Result<u8, NegotiationError> {
        let mut hasher = FxHasher::default();
        hasher.write_usize(seed);
        hasher.write_str(&self.name);
        let channel = (hasher.finish() % u8::MAX as u64) as u8;

        if let Some(old) = seen.insert(channel, self.name.clone()) {
            Err(NegotiationError {
                old,
                new: self.name.clone(),
            })
        } else {
            Ok(channel)
        }
    }
}

impl<A0: FromPeer, A1: FromPeer> FromPeer for (A0, A1) {
    type Ids = (A0::Ids, A1::Ids);
    type Product = (A0::Product, A1::Product);

    fn from_peer(
        peer: &NetworkPeer,
        ids: &Self::Ids,
        pubs: &mut FxHashMap<u8, NetworkPublisher>,
    ) -> Self::Product {
        (
            A0::from_peer(peer, &ids.0, pubs),
            A1::from_peer(peer, &ids.1, pubs),
        )
    }

    fn get_ids(
        &self,
        seed: usize,
        seen: &mut FxHashMap<u8, Cow<'static, str>>,
    ) -> Result<Self::Ids, NegotiationError> {
        Ok((self.0.get_ids(seed, seen)?, self.1.get_ids(seed, seen)?))
    }
}

impl<A0: FromPeer, A1: FromPeer, A2: FromPeer> FromPeer for (A0, A1, A2) {
    type Ids = (A0::Ids, A1::Ids, A2::Ids);
    type Product = (A0::Product, A1::Product, A2::Product);

    fn from_peer(
        peer: &NetworkPeer,
        ids: &Self::Ids,
        pubs: &mut FxHashMap<u8, NetworkPublisher>,
    ) -> Self::Product {
        (
            A0::from_peer(peer, &ids.0, pubs),
            A1::from_peer(peer, &ids.1, pubs),
            A2::from_peer(peer, &ids.2, pubs),
        )
    }

    fn get_ids(
        &self,
        seed: usize,
        seen: &mut FxHashMap<u8, Cow<'static, str>>,
    ) -> Result<Self::Ids, NegotiationError> {
        Ok((
            self.0.get_ids(seed, seen)?,
            self.1.get_ids(seed, seen)?,
            self.2.get_ids(seed, seen)?,
        ))
    }
}

impl<A0: FromPeer, A1: FromPeer, A2: FromPeer, A3: FromPeer> FromPeer for (A0, A1, A2, A3) {
    type Ids = (A0::Ids, A1::Ids, A2::Ids, A3::Ids);
    type Product = (A0::Product, A1::Product, A2::Product, A3::Product);

    fn from_peer(
        peer: &NetworkPeer,
        ids: &Self::Ids,
        pubs: &mut FxHashMap<u8, NetworkPublisher>,
    ) -> Self::Product {
        (
            A0::from_peer(peer, &ids.0, pubs),
            A1::from_peer(peer, &ids.1, pubs),
            A2::from_peer(peer, &ids.2, pubs),
            A3::from_peer(peer, &ids.3, pubs),
        )
    }

    fn get_ids(
        &self,
        seed: usize,
        seen: &mut FxHashMap<u8, Cow<'static, str>>,
    ) -> Result<Self::Ids, NegotiationError> {
        Ok((
            self.0.get_ids(seed, seen)?,
            self.1.get_ids(seed, seen)?,
            self.2.get_ids(seed, seen)?,
            self.3.get_ids(seed, seen)?,
        ))
    }
}

impl<A0: FromPeer, A1: FromPeer, A2: FromPeer, A3: FromPeer, A4: FromPeer> FromPeer
    for (A0, A1, A2, A3, A4)
{
    type Ids = (A0::Ids, A1::Ids, A2::Ids, A3::Ids, A4::Ids);
    type Product = (
        A0::Product,
        A1::Product,
        A2::Product,
        A3::Product,
        A4::Product,
    );

    fn from_peer(
        peer: &NetworkPeer,
        ids: &Self::Ids,
        pubs: &mut FxHashMap<u8, NetworkPublisher>,
    ) -> Self::Product {
        (
            A0::from_peer(peer, &ids.0, pubs),
            A1::from_peer(peer, &ids.1, pubs),
            A2::from_peer(peer, &ids.2, pubs),
            A3::from_peer(peer, &ids.3, pubs),
            A4::from_peer(peer, &ids.4, pubs),
        )
    }

    fn get_ids(
        &self,
        seed: usize,
        seen: &mut FxHashMap<u8, Cow<'static, str>>,
    ) -> Result<Self::Ids, NegotiationError> {
        Ok((
            self.0.get_ids(seed, seen)?,
            self.1.get_ids(seed, seen)?,
            self.2.get_ids(seed, seen)?,
            self.3.get_ids(seed, seen)?,
            self.4.get_ids(seed, seen)?,
        ))
    }
}
