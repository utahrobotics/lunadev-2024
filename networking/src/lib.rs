#![feature(hasher_prefixfree_extras)]

use std::{
    net::{SocketAddr, ToSocketAddrs},
    sync::{
        mpsc::{Receiver, Sender},
        Arc, OnceLock, Weak,
    },
    time::{Duration, Instant},
};

pub use bitcode;
use bitcode::{Decode, Encode};
use crossbeam::queue::SegQueue;
use fxhash::FxHashMap;
use laminar::{Packet, Socket};
use negotiation::{FromPeer, Negotiation};
use peer::NetworkPublisher;
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait, asyncify_run, pubsub::{
        subs::Subscription,
        MonoPublisher,
    },
    setup_logging, tokio,
    utils::DropWrapper,
    DropCheck, Node, NodeIntrinsics, RuntimeContext,
};

use crate::peer::Peer;

pub mod negotiation;
pub mod peer;

pub struct NetworkPeer {
    remote_addr: SocketAddr,
    packets_to_send: Weak<SegQueue<Packet>>,
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

    pub fn get_remote_addr(&self) -> SocketAddr {
        self.remote_addr
    }
}

pub struct NetworkConnector {
    address_sender: Sender<(Packet, tokio::sync::oneshot::Sender<NetworkPeer>)>,
}

pub enum ConnectionError {
    Timeout,
    ServerDropped,
}

impl NetworkConnector {
    pub async fn connect_to<T: Encode>(
        &mut self,
        addr: SocketAddr,
        init_data: &T,
    ) -> Result<NetworkPeer, ConnectionError> {
        let (sender, receiver) = tokio::sync::oneshot::channel();
        self.address_sender
            .send((
                Packet::reliable_sequenced(addr, bitcode::encode(init_data).unwrap(), None),
                sender,
            ))
            .map_err(|_| ConnectionError::ServerDropped)?;
        receiver.await.map_err(|_| ConnectionError::Timeout)
    }
}

pub struct NetworkNode {
    pub service_duration: Duration,
    pub peer_buffer_size: usize,
    intrinsics: NodeIntrinsics<Self>,
    socket: Socket,
    peer_pub: MonoPublisher<(Packet, NetworkPeer)>,
    address_receiver: Receiver<(Packet, tokio::sync::oneshot::Sender<NetworkPeer>)>,
}

pub fn new_client() -> laminar::Result<(NetworkNode, NetworkConnector)> {
    let (address_sender, address_receiver) = std::sync::mpsc::channel();
    Ok((
        NetworkNode {
            socket: Socket::bind_any()?,
            service_duration: Duration::from_millis(50),
            peer_buffer_size: 8,
            peer_pub: MonoPublisher::new(),
            intrinsics: NodeIntrinsics::default(),
            address_receiver,
        },
        NetworkConnector { address_sender },
    ))
}

pub fn new_server<T: Decode>(
    bind_address: impl ToSocketAddrs,
    peer_sub: impl Subscription<Item = Result<(T, NetworkPeer), bitcode::Error>> + Send,
) -> laminar::Result<(NetworkNode, NetworkConnector)> {
    let (address_sender, address_receiver) = std::sync::mpsc::channel();
    Ok((
        NetworkNode {
            socket: Socket::bind(bind_address)?,
            service_duration: Duration::from_millis(50),
            peer_buffer_size: 8,
            peer_pub: MonoPublisher::from(
                peer_sub
                    .map(|(packet, peer): (Packet, NetworkPeer)| {
                        Ok((bitcode::decode(&packet.payload())?, peer))
                    })
                    .boxed(),
            ),
            intrinsics: NodeIntrinsics::default(),
            address_receiver,
        },
        NetworkConnector { address_sender },
    ))
}

#[async_trait]
impl Node for NetworkNode {
    const DEFAULT_NAME: &'static str = "networking";

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let drop_check = DropCheck::default();
        let _drop_check = drop_check.clone();
        let mut conns: FxHashMap<SocketAddr, Peer> = FxHashMap::default();
        let packet_queue = Arc::new(SegQueue::new());
        let mut socket = DropWrapper::new(self.socket, |_| {});
        let mut spin_sleeper = SpinSleeper::default();
        let mut now = Instant::now();

        asyncify_run(move || loop {
            socket.manual_poll(now);
            if drop_check.has_dropped() {
                break Ok(());
            }

            while let Some(event) = socket.recv() {
                let dc_addr = match event {
                    laminar::SocketEvent::Packet(packet) => {
                        if let Some(peer) = conns.get(&packet.addr()) {
                            match peer {
                                Peer::Connecting { peer_sender } => {
                                    let packets_router: Arc<
                                        OnceLock<FxHashMap<u8, NetworkPublisher>>,
                                    > = Arc::default();
                                    let peer = NetworkPeer {
                                        remote_addr: packet.addr(),
                                        packets_to_send: Arc::downgrade(&packet_queue),
                                        packets_router: Arc::downgrade(&packets_router),
                                    };
                                    let _ = peer_sender.send(peer);
                                }
                                Peer::Connected { packets_router } => {
                                    let channel = *packet.payload().last().unwrap();
                                    let data = packet
                                        .payload()
                                        .split_at(packet.payload().len() - 1)
                                        .0
                                        .to_vec()
                                        .into_boxed_slice();

                                    let Some(netpub) =
                                        packets_router.get().map(|x| x.get(&channel)).flatten()
                                    else {
                                        error!(
                                            "[{}] Unrecognized channel: {channel}",
                                            packet.addr()
                                        );
                                        continue;
                                    };
                                    (netpub.setter)(data);
                                }
                            }
                        } else if let Err(e) =
                            socket.send(Packet::reliable_sequenced(packet.addr(), vec![], None))
                        {
                            error!("Failed to reply to {}: {e}", packet.addr());
                        }
                        continue;
                    }
                    laminar::SocketEvent::Connect(addr) => {
                        continue;
                    }
                    laminar::SocketEvent::Timeout(addr) => addr,
                    laminar::SocketEvent::Disconnect(addr) => addr,
                };
            }

            while let Ok((packet, peer_sender)) = self.address_receiver.try_recv() {
                conns.insert(packet.addr(), Peer::Connecting { peer_sender });

                if let Err(e) = socket.send(packet) {
                    error!("Failed to connect to {}: {e}", packet.addr());
                }
            }

            conns.retain(|addr, peer| {
                let retain = match peer {
                    Peer::Connecting { peer_sender } => !peer_sender.is_closed(),
                    Peer::Connected { packets_router } => {
                        Some(true)
                            == packets_router.get().map(|packets_router| {
                                packets_router.iter().any(|(_, netpub)| (netpub.valid)())
                            })
                    }
                };

                if !retain {
                    // disconnect
                }

                retain
            });
            spin_sleeper.sleep(self.service_duration.saturating_sub(now.elapsed()));
            now += now.elapsed();
        })
        .await
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}
