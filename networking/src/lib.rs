#![feature(hasher_prefixfree_extras)]

use std::{
    collections::hash_map::Entry,
    net::{SocketAddr, ToSocketAddrs},
    num::NonZeroU8,
    sync::{
        mpsc::{Receiver, Sender},
        Arc, Weak,
    },
    time::{Duration, Instant},
};

pub use bitcode;
use bitcode::{Decode, Encode};
use fxhash::FxHashMap;
use laminar::{Packet, Socket};
use negotiation::{FromPeer, Negotiation};
use peer::{AwaitingNegotiationReq, NetworkPublisher};
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait, asyncify_run,
    pubsub::{
        subs::{DirectSubscription, Subscription},
        MonoPublisher, Subscriber,
    },
    setup_logging,
    tokio::{self, sync::oneshot},
    utils::DropWrapper,
    DropCheck, Node, NodeIntrinsics, RuntimeContext,
};

use crate::peer::{PeerStateMachine, Retention};

pub mod negotiation;
pub mod peer;

#[derive(Encode, Decode, PartialEq, Eq, Debug)]
enum SpecialMessage {
    Connect(Vec<u8>),
    Disconnect,
    Negotiate,
    Ack,
}

#[derive(Debug)]
enum PeerQuirk {
    ServerSide {
        received_client_negotiation: oneshot::Receiver<()>,
    },
    ClientSide,
}

#[derive(Debug)]
pub struct NetworkPeer {
    remote_addr: SocketAddr,
    packets_to_send: DirectSubscription<Packet>,
    packets_router: oneshot::Sender<(FxHashMap<NonZeroU8, NetworkPublisher>, Weak<()>)>,
    quirk: PeerQuirk,
}

#[derive(Debug)]
pub enum NegotiationError {
    ClientDidNotNegotiate,
    ServerDropped,
}

impl NetworkPeer {
    pub async fn negotiate<T: FromPeer>(
        mut self,
        negotiation: &Negotiation<T>,
    ) -> Result<T::Product, NegotiationError> {
        let mut map = FxHashMap::default();
        let channel_count = Arc::new(());
        let channel_count_weak = Arc::downgrade(&channel_count);
        let negotiation = T::from_peer(&self, &negotiation.channel_ids, &mut map, channel_count);
        self.packets_router
            .send((map, channel_count_weak))
            .map_err(|_| NegotiationError::ServerDropped)?;

        match std::mem::replace(&mut self.quirk, PeerQuirk::ClientSide) {
            PeerQuirk::ServerSide {
                received_client_negotiation,
            } => received_client_negotiation
                .await
                .map_err(|_| NegotiationError::ClientDidNotNegotiate)?,

            PeerQuirk::ClientSide => {}
        }
        Ok(negotiation)
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
                Packet::reliable_ordered(
                    addr,
                    bitcode::encode(&SpecialMessage::Connect(
                        bitcode::encode(init_data).unwrap(),
                    ))
                    .unwrap(),
                    None,
                ),
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
            socket: Socket::bind("0.0.0.0:0")?,
            service_duration: Duration::from_millis(20),
            peer_buffer_size: 8,
            peer_pub: MonoPublisher::new(),
            intrinsics: NodeIntrinsics::default(),
            address_receiver,
        },
        NetworkConnector { address_sender },
    ))
}

pub fn new_server<T, S>(
    bind_address: impl ToSocketAddrs,
    peer_sub: S,
) -> laminar::Result<(NetworkNode, NetworkConnector)>
where
    T: Decode,
    S: Subscription<Item = Result<(T, NetworkPeer), bitcode::Error>> + Send + 'static,
{
    let (address_sender, address_receiver) = std::sync::mpsc::channel();
    Ok((
        NetworkNode {
            socket: Socket::bind(bind_address)?,
            service_duration: Duration::from_millis(20),
            peer_buffer_size: 8,
            peer_pub: MonoPublisher::from(
                peer_sub
                    .filter_map(|(packet, peer): (Packet, NetworkPeer)| {
                        let msg: SpecialMessage = match bitcode::decode(packet.payload()) {
                            Ok(x) => x,
                            Err(e) => return Some(Err(e)),
                        };
                        let SpecialMessage::Connect(data) = msg else {
                            return None;
                        };
                        Some(bitcode::decode(&data).map(|data| (data, peer)))
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

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let drop_check = DropCheck::default();
        let _drop_check = drop_check.clone();
        let mut conns: FxHashMap<SocketAddr, PeerStateMachine> = FxHashMap::default();
        let mut socket = DropWrapper::new(self.socket, |_| {});
        let spin_sleeper = SpinSleeper::default();
        let mut now = Instant::now();

        asyncify_run(move || loop {
            now += now.elapsed();
            socket.manual_poll(now);
            if drop_check.has_dropped() {
                break Ok(());
            }

            while let Some(event) = socket.recv() {
                match event {
                    laminar::SocketEvent::Packet(packet) => match conns.entry(packet.addr()) {
                        Entry::Occupied(mut entry) => {
                            let addr = packet.addr();
                            if Retention::Drop
                                == entry.get_mut().provide_data(
                                    packet,
                                    &context,
                                    self.peer_buffer_size,
                                )
                            {
                                info!("Received disconnect from {}", addr);
                                entry.remove();
                            }
                        }
                        Entry::Vacant(entry) => {
                            if self.peer_pub.get_sub_count() == 0 {
                                error!("Received packet from unknown address: {}", packet.addr());
                            } else if let Err(e) = socket.send(Packet::reliable_ordered(
                                packet.addr(),
                                bitcode::encode(&SpecialMessage::Ack).unwrap(),
                                None,
                            )) {
                                error!("Failed to send ack to {}: {e}", packet.addr());
                            } else {
                                let packets_sub = Subscriber::new(self.peer_buffer_size);
                                let (packets_router_sender, negotiation_recv) = oneshot::channel();
                                let (client_negotiation_sender, received_client_negotiation_recv) =
                                    oneshot::channel();

                                let remote_addr = packet.addr();
                                self.peer_pub.set((
                                    packet,
                                    NetworkPeer {
                                        remote_addr,
                                        packets_to_send: packets_sub.create_subscription(),
                                        packets_router: packets_router_sender,
                                        quirk: PeerQuirk::ServerSide {
                                            received_client_negotiation:
                                                received_client_negotiation_recv,
                                        },
                                    },
                                ));
                                entry.insert(PeerStateMachine::AwaitingNegotiation {
                                    packets_sub: Some(packets_sub),
                                    req: AwaitingNegotiationReq::ServerNegotiation {
                                        negotiation_recv,
                                        client_negotiation_sender,
                                    },
                                });
                            }
                        }
                    },
                    laminar::SocketEvent::Connect(addr) => {
                        info!("Connected to {addr}");
                        continue;
                    }
                    laminar::SocketEvent::Timeout(addr) => {
                        info!("Timed out from {addr}");
                        conns.remove(&addr);
                    }
                    laminar::SocketEvent::Disconnect(addr) => {
                        info!("Disconnected from {addr}");
                        conns.remove(&addr);
                    }
                }
            }

            conns.retain(|addr, peer| match peer.poll(&mut socket, *addr, &context) {
                Retention::Drop => false,
                Retention::Retain => true,
            });

            while let Ok((packet, peer_sender)) = self.address_receiver.try_recv() {
                let addr = packet.addr();

                match conns.entry(packet.addr()) {
                    Entry::Occupied(_) => {
                        error!("Already connected to {}", packet.addr());
                        continue;
                    }
                    Entry::Vacant(entry) => {
                        entry.insert(PeerStateMachine::Connecting { peer_sender });

                        if let Err(e) = socket.send(packet) {
                            error!("Failed to connect to {addr}: {e}");
                        }
                    }
                }
            }

            spin_sleeper.sleep(self.service_duration.saturating_sub(now.elapsed()));
        })
        .await
    }

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }
}
