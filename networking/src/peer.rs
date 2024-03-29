use std::{collections::hash_map::Entry, net::SocketAddr, num::NonZeroU8};

use fxhash::FxHashMap;
use laminar::{Packet, Socket};
use unros::{
    pubsub::Subscriber,
    setup_logging,
    tokio::sync::oneshot::{self, error::TryRecvError},
    RuntimeContext,
};

use crate::{NetworkPeer, PeerQuirk, SpecialMessage};

pub struct NetworkPublisher {
    pub(crate) setter: Box<dyn Fn(Box<[u8]>) + Send + Sync>,
    pub(crate) valid: Box<dyn Fn() -> bool + Send + Sync>,
}

pub(super) enum NetworkRole {
    Server,
    Client,
}

pub(super) enum AwaitingNegotiationReq {
    Negotiation(oneshot::Receiver<FxHashMap<NonZeroU8, NetworkPublisher>>),
    NegotiateResponse(FxHashMap<NonZeroU8, NetworkPublisher>),
}

pub(super) enum PeerStateMachine {
    /// Variant only on the client side, created after the init data has been sent to the server
    /// but before a `Negotiate` has been received from the server.
    Connecting {
        peer_sender: oneshot::Sender<NetworkPeer>,
    },

    /// Variant on both the client and server side.
    ///
    /// Server - Server is waiting for code to negotiate channels internally before sending `Negotiate` to client.
    /// Waits for `Negotiate` from client before transitioning to `Connected`.  
    /// Client - Client is waiting for code to negotiate channels internally before sending `Negotiate` to server.
    /// Instantly transitions to `Connected` after sending `Negotiate`.
    AwaitingNegotiation {
        packets_sub: Subscriber<Packet>,
        req: AwaitingNegotiationReq,
        role: NetworkRole,
    },

    /// Variant on both the client and server side.
    ///
    /// Channels have been negotiated so it is safe to send and receive packets.
    Connected {
        packets_sub: Subscriber<Packet>,
        packets_router: FxHashMap<NonZeroU8, NetworkPublisher>,
    },
}

#[derive(PartialEq, Eq)]
pub(super) enum Retention {
    Drop,
    Retain,
}

impl PeerStateMachine {
    pub fn provide_data(
        &mut self,
        packet: Packet,
        context: &RuntimeContext,
        peer_buffer_size: usize,
    ) -> Retention {
        let data = packet.payload();
        let addr = packet.addr();
        setup_logging!(context);

        match self {
            PeerStateMachine::Connecting { peer_sender } => {
                match bitcode::decode::<SpecialMessage>(data) {
                    Ok(SpecialMessage::Disconnect) => return Retention::Drop,
                    Ok(SpecialMessage::Negotiate) => {
                        let (packets_router_sender, packets_router_recv) = oneshot::channel();
                        let packets_sub = Subscriber::new(peer_buffer_size);
                        let peer = NetworkPeer {
                            remote_addr: addr,
                            packets_to_send: packets_sub.create_subscription(),
                            packets_router: packets_router_sender,
                            quirk: PeerQuirk::ClientSide,
                        };
                        *self = PeerStateMachine::AwaitingNegotiation {
                            packets_sub,
                            req: AwaitingNegotiationReq::Negotiation(packets_router_recv),
                            role: NetworkRole::Client,
                        };
                        if peer_sender.send(peer).is_ok() {
                            Retention::Retain
                        } else {
                            Retention::Drop
                        }
                    }
                    Err(e) => error!("Failed to parse special_msg from {addr}: {e}"),
                }
            }

            PeerStateMachine::AwaitingNegotiation {
                req,
                role,
                packets_sub,
            } => match bitcode::decode::<SpecialMessage>(data) {
                Ok(SpecialMessage::Disconnect) => return Retention::Drop,
                Ok(SpecialMessage::Negotiate) => match role {
                    NetworkRole::Server => match req {
                        AwaitingNegotiationReq::Negotiation(_) => {
                            warn!("Unexpected Negotiate from {addr}")
                        }
                        AwaitingNegotiationReq::NegotiateResponse(packets_router) => {
                            *self = PeerStateMachine::Connected {
                                packets_sub: std::mem::replace(packets_sub, Subscriber::new(1)),
                                packets_router: std::mem::take(packets_router),
                            };

                            Retention::Retain
                        }
                    },
                    NetworkRole::Client => warn!("Unexpected Negotiate from {addr}"),
                },
                Err(e) => error!("Failed to parse special_msg from {addr}: {e}"),
            },

            PeerStateMachine::Connected {
                packets_router,
                packets_sub,
            } => {
                let channel = *data.last().unwrap();
                let data = data.split_at(data.len() - 1).0;

                let Some(channel) = NonZeroU8::new(channel) else {
                    match bitcode::decode::<SpecialMessage>(data) {
                        Ok(SpecialMessage::Disconnect) => return Retention::Drop,

                        Ok(x) => error!("Unexpected special_msg from {addr}: {x:?}"),
                        Err(e) => error!("Failed to parse special_msg from {addr}: {e}"),
                    }
                    return Retention::Retain;
                };

                match packets_router.entry(channel) {
                    Entry::Occupied(entry) => {
                        let publisher = entry.get();
                        if (publisher.valid)() {
                            (publisher.setter)(data.into());
                        } else {
                            entry.remove();
                        }
                    }
                    Entry::Vacant(_) => {
                        error!("Unrecognized channel: {}", channel);
                    }
                }

                Retention::Retain
            }
        }
    }

    pub fn poll(
        &mut self,
        socket: &Socket,
        addr: SocketAddr,
        context: &RuntimeContext,
    ) -> Retention {
        setup_logging!(context);

        match self {
            PeerStateMachine::Connecting { peer_sender } => {
                if peer_sender.is_closed() {
                    Retention::Drop
                } else {
                    Retention::Retain
                }
            }
            PeerStateMachine::AwaitingNegotiation {
                req,
                role,
                packets_sub,
            } => match req {
                AwaitingNegotiationReq::Negotiation(recv) => match recv.try_recv() {
                    Ok(packets_router) => {
                        match role {
                            NetworkRole::Server => {
                                *req = AwaitingNegotiationReq::NegotiateResponse(packets_router)
                            }

                            NetworkRole::Client => {
                                *self = PeerStateMachine::Connected {
                                    packets_sub: std::mem::replace(packets_sub, Subscriber::new(1)),
                                    packets_router,
                                }
                            }
                        }
                        if let Err(e) = socket.send(Packet::reliable_ordered(
                            addr,
                            bitcode::encode(&SpecialMessage::Negotiate).unwrap(),
                            None,
                        )) {
                            error!("Failed to send Negotiate to {addr}: {e}");
                        }
                        Retention::Retain
                    }
                    Err(TryRecvError::Closed) => Retention::Drop,
                    Err(TryRecvError::Empty) => Retention::Retain,
                },
                AwaitingNegotiationReq::NegotiateResponse(_) => todo!(),
            },
            PeerStateMachine::Connected {
                packets_router,
                packets_sub,
            } => {
                while let Some(packet) = packets_sub.try_recv() {
                    if let Err(e) = socket.send(packet) {
                        error!("Failed to send packet to {addr}: {e}");
                    }
                }
                if packets_router.is_empty() && packets_sub.get_pub_count() == 0 {
                    Retention::Drop
                } else {
                    Retention::Retain
                }
            }
        }
    }
}
