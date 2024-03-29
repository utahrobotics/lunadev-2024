#![feature(hasher_prefixfree_extras)]

use std::{
    collections::hash_map::Entry,
    net::{SocketAddr, ToSocketAddrs},
    num::NonZeroU8,
    sync::mpsc::{Receiver, Sender},
    time::{Duration, Instant},
};

pub use bitcode;
use bitcode::{Decode, Encode};
use fxhash::FxHashMap;
use laminar::{Packet, Socket};
use negotiation::{FromPeer, Negotiation};
use peer::{NetworkPublisher, NetworkRole};
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
    Disconnect,
    Negotiate,
}

enum PeerQuirk {
    ServerSide {
        received_client_negotiation: oneshot::Receiver<()>,
    },
    ClientSide,
}

pub struct NetworkPeer {
    remote_addr: SocketAddr,
    packets_to_send: DirectSubscription<Packet>,
    packets_router: oneshot::Sender<FxHashMap<NonZeroU8, NetworkPublisher>>,
    quirk: PeerQuirk,
}

impl NetworkPeer {
    pub async fn negotiate<T: FromPeer>(
        mut self,
        negotiation: &Negotiation<T>,
    ) -> Option<T::Product> {
        match self.quirk {
            PeerQuirk::ServerSide {
                received_client_negotiation,
            } => received_client_negotiation.await.ok()?,

            PeerQuirk::ClientSide => {
                let mut pubber = MonoPublisher::from(self.packets_to_send.clone());
                pubber.set(Packet::reliable_ordered(
                    self.remote_addr,
                    bitcode::encode(&SpecialMessage::Negotiate).unwrap(),
                    None,
                ));
            }
        }

        self.quirk = PeerQuirk::ClientSide;

        let mut map = FxHashMap::default();
        let negotiation = T::from_peer(&self, &negotiation.channel_ids, &mut map);
        self.packets_router.send(map).ok()?;
        Some(negotiation)
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
        let mut conns: FxHashMap<SocketAddr, PeerStateMachine> = FxHashMap::default();
        let mut socket = DropWrapper::new(self.socket, |_| {});
        let mut spin_sleeper = SpinSleeper::default();
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
                        Entry::Occupied(entry) => {
                            if Retention::Drop
                                == entry.get_mut().provide_data(
                                    packet,
                                    &context,
                                    self.peer_buffer_size,
                                )
                            {
                                entry.remove();
                            }
                        }
                        Entry::Vacant(entry) => {
                            if self.peer_pub.get_sub_count() == 0 {
                                error!("Received packet from unknown address: {}", packet.addr());
                            } else {
                                let packets_sub = Subscriber::new(self.peer_buffer_size);
                                let (packets_router_sender, packets_router_recv) =
                                    oneshot::channel();
                                let (
                                    received_client_negotiation_sender,
                                    received_client_negotiation_recv,
                                ) = oneshot::channel();

                                self.peer_pub.set((
                                    packet,
                                    NetworkPeer {
                                        remote_addr: packet.addr(),
                                        packets_to_send: packets_sub.create_subscription(),
                                        packets_router: packets_router_sender,
                                        quirk: PeerQuirk::ServerSide {
                                            received_client_negotiation:
                                                received_client_negotiation_recv,
                                        },
                                    },
                                ));
                                entry.insert(PeerStateMachine::AwaitingNegotiation {
                                    packets_sub: Subscriber::new(self.peer_buffer_size),
                                    req: peer::AwaitingNegotiationReq::Negotiation(
                                        packets_router_recv,
                                    ),
                                    role: NetworkRole::Server,
                                });
                            }
                        } // if let Some(peer) =  {
                          //     match peer.provide_data(packet, &context, self.peer_buffer_size) {
                          //         ProvideDataResponse::GeneratePeer(peer) => {
                          //             conns.insert(packet.addr(), PeerStateMachine::Connected { peer });
                          //         }
                          //         ProvideDataResponse::Drop => {
                          //             conns.remove(&packet.addr());
                          //         }
                          //         ProvideDataResponse::Retain => {}
                          //     }
                          // } else if self.peer_pub.get_sub_count() == 0 {
                          //     error!("Received packet from unknown address: {}", packet.addr());
                          // } else if let Err(e) =
                          //     socket.send(Packet::reliable_sequenced(packet.addr(), vec![], None))
                          // {
                          //     error!("Failed to reply to {}: {e}", packet.addr());
                          // } else {
                          //     conns.insert(packet.addr(), PeerStateMachine::C)
                          // }
                          // continue;
                    },
                    laminar::SocketEvent::Connect(addr) => {
                        // todo
                        continue;
                    }
                    laminar::SocketEvent::Timeout(addr) => {
                        conns.remove(&addr);
                    }
                    laminar::SocketEvent::Disconnect(addr) => {
                        conns.remove(&addr);
                    }
                }
            }

            conns.retain(|addr, peer| match peer.poll(&socket, *addr, &context) {
                Retention::Drop => false,
                Retention::Retain => true,
            });

            while let Ok((packet, peer_sender)) = self.address_receiver.try_recv() {
                conns.insert(packet.addr(), PeerStateMachine::Connecting { peer_sender });

                if let Err(e) = socket.send(packet) {
                    error!("Failed to connect to {}: {e}", packet.addr());
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
