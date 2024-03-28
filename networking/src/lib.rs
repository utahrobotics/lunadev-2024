#![feature(hasher_prefixfree_extras)]

use std::{
    marker::PhantomData,
    net::SocketAddrV4,
    ops::{Deref, DerefMut},
    sync::{
        mpsc::{Receiver, Sender},
        Arc, OnceLock, Weak,
    },
};

pub use bitcode;
use bitcode::{Decode, Encode};
use enet::{
    Address, BandwidthLimit, ChannelLimit, Enet, Event, Host, Packet, PacketMode, Peer, PeerState,
};
use fxhash::FxHashMap;
use negotiation::{FromPeer, Negotiation};
use peer::NetworkPublisher;
use unros::{
    anyhow, async_trait, asyncify_run, log,
    pubsub::{subs::DirectSubscription, Subscriber},
    setup_logging, tokio, DropCheck, Node, NodeIntrinsics, RuntimeContext,
};

use crate::peer::ENetPeer;

pub mod negotiation;
pub mod peer;

pub struct NetworkPeer {
    remote_addr: SocketAddrV4,
    packets_to_send: DirectSubscription<(Box<[u8]>, PacketMode)>,
    packets_router: Weak<OnceLock<FxHashMap<u8, NetworkPublisher>>>,
}

impl NetworkPeer {
    pub fn negotiate<T: FromPeer>(self, negotiation: &Negotiation<T>) -> Option<T::Product> {
        let mut map = FxHashMap::default();
        let negotation = T::from_peer(&self, &negotiation.channel_ids, &mut map);
        if let Some(packets_router) = self.packets_router.upgrade() {
            assert!(packets_router.set(map).is_ok());
            Some(negotation)
        } else {
            None
        }
    }

    pub fn get_remote_addr(&self) -> SocketAddrV4 {
        self.remote_addr
    }
}

pub struct NetworkConnector {
    address_sender: Sender<(
        SocketAddrV4,
        tokio::sync::oneshot::Sender<NetworkPeer>,
        Box<[u8]>,
    )>,
}

impl NetworkConnector {
    pub async fn connect_to<T: Encode>(
        &mut self,
        addr: SocketAddrV4,
        init_data: &T,
    ) -> Option<NetworkPeer> {
        let (sender, receiver) = tokio::sync::oneshot::channel();
        self.address_sender
            .send((addr, sender, bitcode::encode(init_data).unwrap().into()))
            .ok()?;
        receiver.await.ok()
    }
}

pub struct NetworkPeerReceiver<T>(
    tokio::sync::mpsc::UnboundedReceiver<(Box<[u8]>, NetworkPeer)>,
    PhantomData<T>,
);

impl<T: Decode> NetworkPeerReceiver<T> {
    pub async fn recv(&mut self) -> Option<Result<(T, NetworkPeer), bitcode::Error>> {
        self.0
            .recv()
            .await
            .map(|(x, peer)| bitcode::decode(&x).map(|x| (x, peer)))
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
        tokio::sync::mpsc::UnboundedSender<(Box<[u8]>, NetworkPeer)>,
    )>,
    address_receiver: Receiver<(
        SocketAddrV4,
        tokio::sync::oneshot::Sender<NetworkPeer>,
        Box<[u8]>,
    )>,
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

    pub fn new_server<T>(
        bind_address: SocketAddrV4,
        max_peer_count: usize,
    ) -> (Self, NetworkPeerReceiver<T>, NetworkConnector) {
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
            NetworkPeerReceiver(peer_receiver, PhantomData),
            NetworkConnector { address_sender },
        )
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
        let mut conns: FxHashMap<SocketAddrV4, ENetPeer> = FxHashMap::default();
        let mut reliable_round_robin = 0u8;

        asyncify_run(move || {
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
                        Some(Event::Connect(ref mut peer)) => {
                            let addr =
                                SocketAddrV4::new(*peer.address().ip(), peer.address().port());
                            if let Some(enet_peer) = conns.remove(&addr) {
                                if let ENetPeer::Connecting { peer_sender, init_data } = enet_peer {
                                    peer.send_packet(Packet::new(&init_data, PacketMode::ReliableSequenced).unwrap(), reliable_round_robin).unwrap();
                                    reliable_round_robin = (reliable_round_robin + 1) % self.reliable_lanes;

                                    let to_remote = Subscriber::new(self.peer_buffer_size);
                                    let packets_to_send = to_remote.create_subscription();
                                    let packets_router: Arc<OnceLock<FxHashMap<u8, NetworkPublisher>>> =
                                        Arc::default();
                                    let peer = NetworkPeer {
                                        remote_addr: addr,
                                        packets_to_send,
                                        packets_router: Arc::downgrade(&packets_router),
                                    };
                                    let _ = peer_sender.send(peer);

                                    conns.insert(addr, ENetPeer::Connected { to_remote, packets_router });
                                } else {
                                    error!("Connected to a peer that already exists! {addr}");
                                }
                            } else if self.binding.is_some() {
                                conns.insert(addr, ENetPeer::AwaitingInitData);
                            } else {
                                unreachable!("Received unprovoked connection from peer even though we were in client mode");
                            }
                        }
                        Some(Event::Disconnect(ref peer, _)) => {
                            let addr =
                                SocketAddrV4::new(*peer.address().ip(), peer.address().port());
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

                            if let Some(peer) = conns.get_mut(&addr) {
                                if let Some(peer) = peer.put_data(addr, data, channel, self.peer_buffer_size) {
                                    let _ = self.binding.as_ref().unwrap().1.send(peer);
                                }
                            } else {
                                error!("[{addr}] Received data from a peer we are not connected to!")
                            }
                        }
                        None => {}
                    }
                }

                while let Ok((addr, peer_sender, init_data)) = self.address_receiver.try_recv() {
                    conns.insert(addr, ENetPeer::Connecting { peer_sender, init_data });
                    host.connect(&addr.into(), self.reliable_lanes as usize + 1, 0)?;
                }

                let mut peers: FxHashMap<SocketAddrV4, Peer<()>> = host
                    .peers()
                    .map(|peer| {
                        let addr = SocketAddrV4::new(*peer.address().ip(), peer.address().port());
                        (addr, peer)
                    })
                    .collect();

                conns.retain(|addr, enet_peer| {
                    let Some(peer) = peers.get_mut(addr) else {
                        return false;
                    };

                    if enet_peer.retain() {
                        while let Some((data, mode)) = enet_peer.get_data() {
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
                    } else {
                        peer.disconnect(0);
                        false
                    }
                });
            }
        })
        .await
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}
