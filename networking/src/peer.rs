use std::{
    collections::hash_map::Entry, net::SocketAddr, num::NonZeroU8, sync::Weak, time::Instant,
};

use fxhash::FxHashMap;
use laminar::{Packet, Socket};
use unros::{
    pubsub::Subscriber,
    runtime::RuntimeContext,
    setup_logging,
    tokio::sync::oneshot::{self, error::TryRecvError},
};

use crate::{NetworkPeer, PeerQuirk, SpecialMessage};

pub struct NetworkPublisher {
    pub(crate) setter: Box<dyn Fn(Box<[u8]>) + Send + Sync>,
    pub(crate) valid: Box<dyn Fn() -> bool + Send + Sync>,
}

impl std::fmt::Debug for NetworkPublisher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NetworkPublisher")
            .field("valid", &(self.valid)())
            .finish()
    }
}

#[derive(Debug)]
pub(super) enum AwaitingNegotiationReq {
    ServerNegotiation {
        negotiation_recv: oneshot::Receiver<(FxHashMap<NonZeroU8, NetworkPublisher>, Weak<()>)>,
        client_negotiation_sender: oneshot::Sender<()>,
    },
    ServerAwaitNegotiateResponse {
        packets_router: FxHashMap<NonZeroU8, NetworkPublisher>,
        channel_count: Weak<()>,
        client_negotiation_sender: oneshot::Sender<()>,
    },
    ClientNegotiation {
        negotiation_recv: oneshot::Receiver<(FxHashMap<NonZeroU8, NetworkPublisher>, Weak<()>)>,
    },
}

#[derive(Debug)]
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
        packets_sub: Option<Subscriber<Packet>>,
        req: AwaitingNegotiationReq,
    },

    /// Variant on both the client and server side.
    ///
    /// Channels have been negotiated so it is safe to send and receive packets.
    Connected {
        packets_sub: Subscriber<Packet>,
        channel_count: Weak<()>,
        packets_router: FxHashMap<NonZeroU8, NetworkPublisher>,
        last_received: Instant,
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
        socket: &mut Socket,
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
                        let peer_sender = std::mem::replace(peer_sender, oneshot::channel().0);
                        *self = PeerStateMachine::AwaitingNegotiation {
                            packets_sub: Some(packets_sub),
                            req: AwaitingNegotiationReq::ClientNegotiation {
                                negotiation_recv: packets_router_recv,
                            },
                        };
                        if peer_sender.send(peer).is_ok() {
                            Retention::Retain
                        } else {
                            Retention::Drop
                        }
                    }
                    Ok(x) => {
                        if let SpecialMessage::Connect(_) = x {
                            error!("Unexpected Connect from {addr}");
                        }
                        if peer_sender.is_closed() {
                            Retention::Drop
                        } else {
                            Retention::Retain
                        }
                    }
                    Err(e) => {
                        error!("Failed to parse special_msg from {addr} while connecting: {e}");
                        Retention::Retain
                    }
                }
            }

            PeerStateMachine::AwaitingNegotiation { req, packets_sub } => {
                match bitcode::decode::<SpecialMessage>(data) {
                    Ok(SpecialMessage::Disconnect) => return Retention::Drop,
                    Ok(SpecialMessage::Negotiate) => match req {
                        AwaitingNegotiationReq::ServerNegotiation { .. } => {
                            warn!("Unexpected Negotiate from {addr}");
                            Retention::Retain
                        }
                        AwaitingNegotiationReq::ServerAwaitNegotiateResponse {
                            packets_router,
                            channel_count,
                            client_negotiation_sender,
                        } => {
                            if std::mem::replace(client_negotiation_sender, oneshot::channel().0)
                                .send(())
                                .is_ok()
                            {
                                *self = PeerStateMachine::Connected {
                                    packets_sub: std::mem::take(packets_sub).unwrap(),
                                    channel_count: std::mem::take(channel_count),
                                    packets_router: std::mem::take(packets_router),
                                    last_received: Instant::now(),
                                };
                                Retention::Retain
                            } else {
                                Retention::Drop
                            }
                        }
                        AwaitingNegotiationReq::ClientNegotiation { .. } => {
                            warn!("Unexpected Negotiate from {addr}");
                            Retention::Retain
                        }
                    },
                    Ok(x) => {
                        if let SpecialMessage::Connect(_) = x {
                            error!("Unexpected Connect from {addr}");
                        }
                        if let AwaitingNegotiationReq::ServerAwaitNegotiateResponse {
                            client_negotiation_sender,
                            ..
                        } = req
                        {
                            if client_negotiation_sender.is_closed() {
                                Retention::Drop
                            } else {
                                Retention::Retain
                            }
                        } else {
                            Retention::Retain
                        }
                    }
                    Err(e) => {
                        error!("Failed to parse special_msg from {addr} while awaiting negotiation: {e}");
                        Retention::Retain
                    }
                }
            }

            PeerStateMachine::Connected {
                packets_router,
                channel_count,
                packets_sub: _,
                last_received,
            } => {
                *last_received += last_received.elapsed();
                let channel = *data.last().unwrap();
                let data = data.split_at(data.len() - 1).0;

                let Some(channel) = NonZeroU8::new(channel) else {
                    match bitcode::decode::<SpecialMessage>(data) {
                        Ok(SpecialMessage::Disconnect) => return Retention::Drop,
                        Ok(SpecialMessage::Ack) => {}
                        Ok(SpecialMessage::Ping) => {
                            let mut payload = bitcode::encode(&SpecialMessage::Ping).unwrap();
                            payload.push(0);
                            if let Err(e) = socket.send(Packet::reliable_unordered(addr, payload)) {
                                error!("Failed to send ack packet to {addr}: {e}");
                            }
                        }

                        Ok(x) => error!("Unexpected special_msg from {addr}: {x:?}"),
                        Err(e) => {
                            error!("Failed to parse special_msg from {addr} while connected: {e}")
                        }
                    }
                    return Retention::Retain;
                };

                match packets_router.entry(channel) {
                    Entry::Occupied(entry) => {
                        let publisher = entry.get();
                        if (publisher.valid)() {
                            (publisher.setter)(data.into());
                        } else if channel_count.strong_count() == 0 {
                            entry.remove();
                        } else {
                            error!("Publisher is invalid");
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
        socket: &mut Socket,
        addr: SocketAddr,
        context: &RuntimeContext,
    ) -> Retention {
        setup_logging!(context);

        match self {
            PeerStateMachine::Connecting { peer_sender } => {
                if peer_sender.is_closed() {
                    debug!("{addr} dropped while connecting");
                    Retention::Drop
                } else {
                    Retention::Retain
                }
            }
            PeerStateMachine::AwaitingNegotiation { req, packets_sub } => match req {
                AwaitingNegotiationReq::ServerNegotiation {
                    negotiation_recv,
                    client_negotiation_sender,
                } => match negotiation_recv.try_recv() {
                    Ok((packets_router, channel_count)) => {
                        if let Err(e) = socket.send(Packet::reliable_unordered(
                            addr,
                            bitcode::encode(&SpecialMessage::Negotiate).unwrap(),
                        )) {
                            error!("Failed to send Negotiate to {addr}: {e}");
                        }

                        *req = AwaitingNegotiationReq::ServerAwaitNegotiateResponse {
                            packets_router,
                            channel_count,
                            client_negotiation_sender: std::mem::replace(
                                client_negotiation_sender,
                                oneshot::channel().0,
                            ),
                        };

                        Retention::Retain
                    }
                    Err(TryRecvError::Closed) => {
                        debug!("{addr} dropped while awaiting server negotiation");
                        Retention::Drop
                    }
                    Err(TryRecvError::Empty) => Retention::Retain,
                },
                AwaitingNegotiationReq::ServerAwaitNegotiateResponse {
                    packets_router: _,
                    channel_count: _,
                    client_negotiation_sender,
                } => {
                    if client_negotiation_sender.is_closed() {
                        debug!("{addr} dropped while awaiting client response from server");
                        Retention::Drop
                    } else {
                        Retention::Retain
                    }
                }
                AwaitingNegotiationReq::ClientNegotiation { negotiation_recv } => {
                    match negotiation_recv.try_recv() {
                        Ok((packets_router, channel_count)) => {
                            if let Err(e) = socket.send(Packet::reliable_unordered(
                                addr,
                                bitcode::encode(&SpecialMessage::Negotiate).unwrap(),
                            )) {
                                error!("Failed to send Negotiate to {addr}: {e}");
                            }

                            *self = PeerStateMachine::Connected {
                                packets_sub: std::mem::take(packets_sub).unwrap(),
                                channel_count,
                                packets_router,
                                last_received: Instant::now(),
                            };

                            Retention::Retain
                        }
                        Err(TryRecvError::Closed) => {
                            debug!("{addr} dropped while awaiting client negotiation");
                            Retention::Drop
                        }
                        Err(TryRecvError::Empty) => Retention::Retain,
                    }
                }
            },
            PeerStateMachine::Connected {
                packets_router,
                channel_count,
                packets_sub,
                last_received,
            } => {
                while let Some(packet) = packets_sub.try_recv() {
                    if let Err(e) = socket.send(packet) {
                        error!("Failed to send packet to {addr}: {e}");
                    }
                }

                let elapsed = last_received.elapsed();
                if elapsed.as_millis() >= 3000 {
                    let mut payload = bitcode::encode(&SpecialMessage::Ping).unwrap();
                    payload.push(0);
                    if let Err(e) = socket.send(Packet::reliable_unordered(addr, payload)) {
                        error!("Failed to send ack packet to {addr}: {e}");
                    }
                    *last_received += elapsed;
                }

                if channel_count.strong_count() == 0 {
                    packets_router.retain(|_, netpub| (netpub.valid)());

                    if packets_router.is_empty() && packets_sub.get_pub_count() == 0 {
                        let mut payload = bitcode::encode(&SpecialMessage::Disconnect).unwrap();
                        payload.push(0);
                        if let Err(e) = socket.send(Packet::reliable_unordered(addr, payload)) {
                            error!("Failed to send disconnect packet to {addr}: {e}");
                        }
                        debug!("Disconnected from {addr} as pubsub are dropped");
                        Retention::Drop
                    } else {
                        Retention::Retain
                    }
                } else {
                    Retention::Retain
                }
            }
        }
    }
}
