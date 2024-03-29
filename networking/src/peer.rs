use std::{
    net::SocketAddrV4,
    sync::{Arc, OnceLock},
};

use enet::PacketMode;
use fxhash::FxHashMap;
use unros::{log::error, pubsub::Subscriber, tokio::sync::oneshot};

use crate::NetworkPeer;

pub struct NetworkPublisher {
    pub(crate) setter: Box<dyn Fn(Box<[u8]>) + Send + Sync>,
    pub(crate) valid: Box<dyn Fn() -> bool + Send + Sync>,
}

pub(super) enum ENetPeer {
    Connecting {
        peer_sender: oneshot::Sender<NetworkPeer>,
        init_data: Box<[u8]>,
    },
    AwaitingInitData,
    Connected {
        to_remote: Subscriber<(Box<[u8]>, PacketMode)>,
        packets_router: Arc<OnceLock<FxHashMap<u8, NetworkPublisher>>>,
    },
}

impl ENetPeer {
    pub(super) fn put_data(
        &mut self,
        addr: SocketAddrV4,
        data: Box<[u8]>,
        channel: u8,
        peer_buffer_size: usize,
    ) -> Option<(Box<[u8]>, NetworkPeer)> {
        match self {
            ENetPeer::Connected { packets_router, .. } => {
                let Some(netpub) = packets_router.get().map(|x| x.get(&channel)).flatten() else {
                    error!("[{addr}] Unrecognized channel: {channel}");
                    return None;
                };
                (netpub.setter)(data);
                None
            }
            ENetPeer::AwaitingInitData => {
                let to_remote = Subscriber::new(peer_buffer_size);
                let packets_to_send = to_remote.create_subscription();
                let packets_router: Arc<OnceLock<FxHashMap<u8, NetworkPublisher>>> = Arc::default();
                let peer = NetworkPeer {
                    remote_addr: addr,
                    packets_to_send,
                    packets_router: Arc::downgrade(&packets_router),
                };
                *self = ENetPeer::Connected {
                    to_remote,
                    packets_router,
                };

                Some((data, peer))
            }
            _ => None,
        }
    }

    pub(super) fn get_data(&self) -> Option<(Box<[u8]>, PacketMode)> {
        match self {
            ENetPeer::Connecting { .. } => None,
            ENetPeer::AwaitingInitData => None,
            ENetPeer::Connected { to_remote, .. } => to_remote.try_recv(),
        }
    }

    pub(super) fn retain(&mut self) -> bool {
        match self {
            ENetPeer::Connecting { peer_sender, .. } => !peer_sender.is_closed(),
            ENetPeer::AwaitingInitData => true,
            ENetPeer::Connected {
                to_remote,
                packets_router,
            } => {
                to_remote.get_pub_count() > 0
                    || packets_router
                        .get()
                        .map(|packets_router| {
                            packets_router.iter().any(|(_, netpub)| (netpub.valid)())
                        })
                        .unwrap_or(false)
            }
        }
    }
}
