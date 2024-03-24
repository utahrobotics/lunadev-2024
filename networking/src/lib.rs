#![feature(hasher_prefixfree_extras)]

use std::{
    hash::Hasher,
    net::SocketAddrV4,
    ops::{Deref, DerefMut},
    sync::{
        mpsc::{Receiver, Sender},
        Arc, Mutex,
    },
};

use bitcode::{Decode, Encode};
pub use bitcode;
use enet::{
    Address, BandwidthLimit, ChannelLimit, Enet, Event, Host, Packet, PacketMode, Peer, PeerState,
};
use fxhash::{FxHashMap, FxHasher};
use unros::{
    anyhow, async_trait, log,
    pubsub::{Publisher, PublisherRef, Subscriber, Subscription},
    setup_logging, tokio, tokio_rayon, DropCheck, Node, NodeIntrinsics, RuntimeContext,
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
    packets_to_send: Subscription<(Box<[u8]>, PacketMode)>,
    packets_router: Arc<Mutex<FxHashMap<u8, Box<dyn FnMut(Box<[u8]>) + Send>>>>,
}

impl NetworkPeer {
    pub fn create_channel<T>(&self, channel_id: ChannelId) -> Channel<T>
    where
        T: Decode + Clone + 'static,
    {
        let mut pub_received_packets = Publisher::default();
        let recv_packets_sub = pub_received_packets.get_ref();

        self.packets_router.lock().unwrap().insert(
            channel_id.0,
            Box::new(move |bytes| {
                pub_received_packets.set(bitcode::decode(&bytes).map_err(Arc::new));
            }),
        );

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
    pub bandwidth_limit: u32,
    pub max_peer_count: usize,
    pub service_millis: u32,
    pub reliable_lanes: u8,
    pub peer_buffer_size: usize,
    intrinsics: NodeIntrinsics<Self>,
    binding: Option<(
        SocketAddrV4,
        tokio::sync::mpsc::UnboundedSender<NetworkPeer>,
    )>,
    address_receiver: Receiver<(SocketAddrV4, tokio::sync::oneshot::Sender<NetworkPeer>)>,
}

impl NetworkNode {
    pub fn new_client(max_peer_count: usize) -> (Self, NetworkConnector) {
        let (address_sender, address_receiver) = std::sync::mpsc::channel();
        (
            Self {
                bandwidth_limit: 0,
                service_millis: 50,
                reliable_lanes: 3,
                max_peer_count,
                peer_buffer_size: 8,
                binding: None,
                intrinsics: NodeIntrinsics::default(),
                address_receiver,
            },
            NetworkConnector { address_sender },
        )
    }

    pub fn new_server(bind_address: SocketAddrV4, max_peer_count: usize) -> (Self, NetworkPeerReceiver, NetworkConnector) {
        let (address_sender, address_receiver) = std::sync::mpsc::channel();
        let (peer_sender, peer_receiver) = tokio::sync::mpsc::unbounded_channel();
        (
            Self {
                bandwidth_limit: 0,
                service_millis: 50,
                reliable_lanes: 3,
                peer_buffer_size: 8,
                max_peer_count,
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
    packets_to_send: Subscription<(Box<[u8]>, PacketMode)>,
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
            let mut data = bitcode::encode(&value).expect("Failed to serialize value");
            data.push(channel_id);
            (data.into(), PacketMode::ReliableSequenced)
        })
    }

    pub fn create_unreliable_subscription(&self) -> Subscription<T> {
        let channel_id = self.channel_id;
        self.packets_to_send.clone().map(move |value| {
            let mut data = bitcode::encode(&value).expect("Failed to serialize value");
            data.push(channel_id);
            (data.into(), PacketMode::UnreliableUnsequenced)
        })
    }
}

struct HostWrapper(Host<()>);

impl Deref for HostWrapper {
    type Target = Host<()>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for HostWrapper {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Drop for HostWrapper {
    fn drop(&mut self) {
        let mut peer_count = 0;
        for mut peer in self.0.peers() {
            if peer.state() == PeerState::Connected {
                peer.disconnect(0);
                peer_count += 1;
            }
        }
        unros::log::info!("{peer_count}");
        while peer_count > 0 {
            let mut event = match self.0.service(1000) {
                Ok(Some(event)) => event,
                Ok(None) => continue,
                Err(e) => {
                    log::error!("Faced the following error while safely dropping ENet host: {e}");
                    break;
                }
            };
            match &mut event {
                Event::Connect(peer) => {
                    peer_count += 1;
                    peer.disconnect(0);
                }
                Event::Disconnect(_, _) => peer_count -= 1,
                _ => {}
            }
        }
        unros::log::info!("fwwf");
    }
}

#[async_trait]
impl Node for NetworkNode {
    const DEFAULT_NAME: &'static str = "networking";

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let enet = Enet::new()?;

        let drop_check = DropCheck::default();
        let _drop_check = drop_check.clone();
        let mut conns: FxHashMap<
            SocketAddrV4,
            (
                Subscriber<(Box<[u8]>, PacketMode)>,
                Arc<Mutex<FxHashMap<u8, Box<dyn FnMut(Box<[u8]>) + Send>>>>,
            ),
        > = FxHashMap::default();
        let mut pending_conns: FxHashMap<SocketAddrV4, tokio::sync::oneshot::Sender<NetworkPeer>> =
            FxHashMap::default();
        let mut reliable_round_robin = 0u8;

        tokio_rayon::spawn(move || {
            let tmp_host = enet.create_host::<()>(
                self.binding
                    .as_ref()
                    .map(|(x, _)| Address::from(*x))
                    .as_ref(),
                self.max_peer_count,
                ChannelLimit::Limited(self.reliable_lanes as usize + 1),
                BandwidthLimit::Unlimited,
                if self.bandwidth_limit == 0 {
                    BandwidthLimit::Unlimited
                } else {
                    BandwidthLimit::Limited(self.bandwidth_limit)
                },
            )?;
            let mut host = HostWrapper(tmp_host);
            loop {
                {
                    let mut option = host.service(self.service_millis)?;
                    if drop_check.has_dropped() {
                        break Ok(());
                    }

                    match option {
                        Some(Event::Connect(ref peer)) => {
                            let addr =
                                SocketAddrV4::new(*peer.address().ip(), peer.address().port());
                            if let Some(sender) = pending_conns.remove(&addr) {
                                let packet_sub = Subscriber::new(self.peer_buffer_size);
                                let packets_to_send = packet_sub.create_subscription();
                                let packets_router: Arc<
                                    Mutex<FxHashMap<u8, Box<dyn FnMut(Box<[u8]>) + Send>>>,
                                > = Arc::default();
                                conns.insert(addr, (packet_sub, packets_router.clone()));
                                let _ = sender.send(NetworkPeer {
                                    remote_addr: addr,
                                    packets_to_send,
                                    packets_router,
                                });
                            } else if let Some((_, sender)) = &self.binding {
                                let packet_sub = Subscriber::new(self.peer_buffer_size);
                                let packets_to_send = packet_sub.create_subscription();
                                let packets_router: Arc<
                                    Mutex<FxHashMap<u8, Box<dyn FnMut(Box<[u8]>) + Send>>>,
                                > = Arc::default();
                                conns.insert(addr, (packet_sub, packets_router.clone()));
                                let _ = sender.send(NetworkPeer {
                                    remote_addr: addr,
                                    packets_to_send,
                                    packets_router,
                                });
                            }
                        }
                        Some(Event::Disconnect(ref peer, _)) => {
                            let addr =
                                SocketAddrV4::new(*peer.address().ip(), peer.address().port());
                            pending_conns.remove(&addr);
                            conns.remove(&addr);
                        }
                        Some(Event::Receive {
                            ref packet,
                            ref mut sender,
                            ..
                        }) => {
                            let channel = *packet.data().last().unwrap();
                            let data = packet
                                .data()
                                .split_at(packet.data().len() - 1)
                                .0
                                .to_vec()
                                .into_boxed_slice();

                            let addr =
                                SocketAddrV4::new(*sender.address().ip(), sender.address().port());

                            let mut drop = false;
                            if let Some((_, packets_router)) = conns.get(&addr) {
                                if Arc::strong_count(&packets_router) == 1 {
                                    drop = true;
                                } else if let Some(packet_sender) =
                                    packets_router.lock().unwrap().get_mut(&channel)
                                {
                                    packet_sender(data);
                                }
                            } else {
                                drop = true;
                            }
                            if drop {
                                conns.remove(&addr);
                                sender.disconnect(0);
                            }
                        }
                        None => {}
                    }
                }

                while let Ok((addr, sender)) = self.address_receiver.try_recv() {
                    pending_conns.insert(addr, sender);
                    host.connect(&addr.into(), self.reliable_lanes as usize + 1, 0)?;
                }

                let mut peers: FxHashMap<SocketAddrV4, Peer<()>> = host
                    .peers()
                    .map(|peer| {
                        let addr = SocketAddrV4::new(*peer.address().ip(), peer.address().port());
                        (addr, peer)
                    })
                    .collect();

                conns.retain(|addr, (packets_to_send, packets_router)| {
                    if Arc::strong_count(&packets_router) == 1 {
                        return false;
                    }
                    let Some(peer) = peers.get_mut(addr) else {
                        return false;
                    };
                    while let Some((data, mode)) = packets_to_send.try_recv() {
                        if mode == PacketMode::ReliableSequenced {
                            peer.send_packet(
                                Packet::new(&data, PacketMode::ReliableSequenced).unwrap(),
                                reliable_round_robin,
                            )
                            .expect("Failed to send packet");
                            reliable_round_robin = (reliable_round_robin + 1) % self.reliable_lanes;
                        } else {
                            peer.send_packet(
                                Packet::new(&data, mode).unwrap(),
                                self.reliable_lanes,
                            )
                            .expect("Failed to send packet");
                        }
                    }
                    true
                });
            }
        })
        .await
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}
