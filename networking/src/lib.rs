#![feature(hasher_prefixfree_extras)]

use std::{
    hash::Hasher,
    net::SocketAddrV4,
    sync::{
        mpsc::{Receiver, Sender, SyncSender},
        Arc, Mutex,
    },
};

use bitcode::{Decode, Encode};
use enet::{BandwidthLimit, Enet, PacketMode};
use fxhash::{FxHashMap, FxHasher};
use unros::{
    anyhow, async_trait, pubsub::{Publisher, PublisherRef, Subscription}, rayon, setup_logging, tokio, Node, NodeIntrinsics, RuntimeContext
};

#[derive(Debug)]
pub struct ChannelCreationError {
    old: Box<str>,
    new: String,
}

impl std::fmt::Display for ChannelCreationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "The given channel name {} conflicts with {}. Either change the name or change the `channel_seed`", self.new, self.old)
    }
}

impl std::error::Error for ChannelCreationError {}

pub struct ChannelMap {
    channel_seed: usize,
    channel_map: FxHashMap<u8, Box<str>>,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ChannelId(u8);

impl ChannelMap {
    pub fn new(channel_seed: usize) -> Self {
        Self {
            channel_seed,
            channel_map: FxHashMap::default(),
        }
    }

    pub fn add_channel(
        &mut self,
        name: impl Into<String>,
    ) -> Result<ChannelId, ChannelCreationError> {
        let mut hasher = FxHasher::default();
        hasher.write_usize(self.channel_seed);
        let new = name.into();
        hasher.write_str(&new);
        let channel = (hasher.finish() % u8::MAX as u64) as u8;

        if let Some(old) = self.channel_map.get(&channel) {
            Err(ChannelCreationError {
                old: old.clone(),
                new,
            })
        } else {
            self.channel_map.insert(channel, new.into());
            Ok(ChannelId(channel))
        }
    }
}

pub struct NetworkPeer {
    remote_addr: SocketAddrV4,
    packets_to_send: Subscription<(Box<[u8]>, PacketMode, u8)>,
    packets_router: Arc<Mutex<FxHashMap<u8, SyncSender<Box<[u8]>>>>>,
}

impl NetworkPeer {
    pub fn create_channel<T>(&self, channel_id: ChannelId) -> Channel<T>
    where
        T: Decode + Clone + 'static,
    {
        let (sender, receiver) = std::sync::mpsc::sync_channel(0);
        self.packets_router
            .lock()
            .unwrap()
            .insert(channel_id.0, sender);

        let mut pub_received_packets = Publisher::default();
        let recv_packets_sub = pub_received_packets.get_ref();

        rayon::spawn(move || loop {
            let Ok(bytes) = receiver.recv() else {
                break;
            };
            pub_received_packets.set(bitcode::decode(&bytes).map_err(Arc::new));
        });

        Channel {
            channel_id: channel_id.0,
            received_packets: recv_packets_sub,
            packets_to_send: self.packets_to_send.clone(),
        }
    }

    pub fn get_remote_addr(&self) -> SocketAddrV4 {
        self.remote_addr
    }
}

pub struct NetworkConnector {
    address_sender: Sender<(SocketAddrV4, tokio::sync::oneshot::Sender<NetworkPeer>)>,
}

impl NetworkConnector {
    pub async fn connect_to(&mut self, addr: SocketAddrV4) -> Option<NetworkPeer> {
        let (sender, receiver) = tokio::sync::oneshot::channel();
        self.address_sender.send((addr, sender)).ok()?;
        receiver.await.ok()
    }
}

pub struct NetworkPeerReceiver(tokio::sync::mpsc::UnboundedReceiver<NetworkPeer>);

impl NetworkPeerReceiver {
    pub async fn recv(&mut self) -> Option<NetworkPeer> {
        self.0.recv().await
    }
}

pub struct NetworkNode {
    intrinsics: NodeIntrinsics<Self>,
    binding: Option<(
        SocketAddrV4,
        tokio::sync::mpsc::UnboundedSender<NetworkPeer>,
    )>,
    address_receiver: Receiver<(SocketAddrV4, tokio::sync::oneshot::Sender<NetworkPeer>)>,
}

impl NetworkNode {
    pub fn new_client() -> (Self, NetworkConnector) {
        let (address_sender, address_receiver) = std::sync::mpsc::channel();
        (
            Self {
                binding: None,
                intrinsics: NodeIntrinsics::default(),
                address_receiver,
            },
            NetworkConnector { address_sender },
        )
    }

    pub fn new_server(bind_address: SocketAddrV4) -> (Self, NetworkPeerReceiver, NetworkConnector) {
        let (address_sender, address_receiver) = std::sync::mpsc::channel();
        let (peer_sender, peer_receiver) = tokio::sync::mpsc::unbounded_channel();
        (
            Self {
                binding: Some((bind_address, peer_sender)),
                intrinsics: NodeIntrinsics::default(),
                address_receiver,
            },
            NetworkPeerReceiver(peer_receiver),
            NetworkConnector { address_sender },
        )
    }
}

pub struct Channel<T> {
    channel_id: u8,
    received_packets: PublisherRef<Result<T, Arc<bitcode::Error>>>,
    packets_to_send: Subscription<(Box<[u8]>, PacketMode, u8)>,
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

impl<T: Decode> Channel<T> {
    pub fn accept_subscription(&self, sub: Subscription<Result<T, Arc<bitcode::Error>>>) {
        self.received_packets.accept_subscription(sub);
    }

    pub fn accept_subscription_or_closed(
        &self,
        sub: Subscription<Result<T, Arc<bitcode::Error>>>,
    ) -> bool {
        self.received_packets.accept_subscription_or_closed(sub)
    }
}

impl<T: Encode> Channel<T> {
    pub fn create_reliable_subscription(&self) -> Subscription<T> {
        let channel_id = self.channel_id;
        self.packets_to_send.clone().map(move |value| {
            (
                bitcode::encode(&value)
                    .expect("Failed to serialize value")
                    .into(),
                PacketMode::ReliableSequenced,
                channel_id,
            )
        })
    }

    pub fn create_unreliable_subscription(&self) -> Subscription<T> {
        let channel_id = self.channel_id;
        self.packets_to_send.clone().map(move |value| {
            (
                bitcode::encode(&value)
                    .expect("Failed to serialize value")
                    .into(),
                PacketMode::UnreliableUnsequenced,
                channel_id,
            )
        })
    }
}

#[async_trait]
impl Node for NetworkNode {
    const DEFAULT_NAME: &'static str = "networking";

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let enet = Enet::new()?;
        let outgoing_limit = if self.bandwidth_limit == 0 {
            BandwidthLimit::Unlimited
        } else {
            BandwidthLimit::Limited(self.bandwidth_limit)
        };

        let mut drop_check = DropCheck::default();
        let drop_check2 = drop_check.clone();
        let _drop_check = drop_check.clone();
        let mut image_subscriptions = self.image_subscriptions.take().unwrap();
        let mut video_dump = self.video_dump.take().unwrap();
        let sdp = video_dump.generate_sdp().unwrap();

        Ok(())
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}
