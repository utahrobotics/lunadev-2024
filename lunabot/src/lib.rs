use std::sync::Arc;

use networking::{bitcode::{self, Decode, Encode}, Channel, ChannelMap, NetworkPeer};

#[derive(Clone)]
pub struct Channels {
    pub important: Channel<ImportantMessage>,
    pub camera: Channel<Arc<str>>,
    pub odometry: Channel<u8>,
    pub controls: Channel<(f32, f32)>,
    pub logs: Channel<Arc<str>>
}

#[derive(Debug, Eq, PartialEq, Encode, Decode, Clone, Copy)]
#[repr(u8)]
pub enum ImportantMessage {
    EnableCamera,
    DisableCamera,
    Ping,
}

impl Channels {
    pub fn new(peer: &NetworkPeer) -> Self {
        let mut channel_map = ChannelMap::new(2342);

        Channels {
            important: peer.create_channel(channel_map.add_channel("important").unwrap()),
            camera: peer.create_channel(channel_map.add_channel("camera").unwrap()),
            odometry: peer.create_channel(channel_map.add_channel("odometry").unwrap()),
            controls: peer.create_channel(channel_map.add_channel("controls").unwrap()),
            logs: peer.create_channel(channel_map.add_channel("logs").unwrap()),
        }
    }
}