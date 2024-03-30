use std::sync::Arc;

use networking::{
    bitcode::{self, Decode, Encode},
    negotiation::{ChannelNegotiation, Negotiation},
};

pub fn make_negotiation() -> Negotiation<(
    ChannelNegotiation<ImportantMessage>,
    ChannelNegotiation<Arc<str>>,
    ChannelNegotiation<u8>,
    ChannelNegotiation<ControlsPacket>,
    ChannelNegotiation<Arc<str>>,
)> {
    Negotiation::new(
        (
            ChannelNegotiation::new("important"),
            ChannelNegotiation::new("camera"),
            ChannelNegotiation::new("odometry"),
            ChannelNegotiation::new("controls"),
            ChannelNegotiation::new("logs"),
        ),
        46432,
    )
    .unwrap()
}

#[derive(Clone, Copy, Encode, Decode, PartialEq, Eq, Default, Debug)]
pub struct ControlsPacket {
    pub drive: i8,
    pub steering: i8,
    pub arm_vel: i8,
}

#[derive(Debug, Eq, PartialEq, Encode, Decode, Clone, Copy)]
#[repr(u8)]
pub enum ImportantMessage {
    EnableCamera,
    DisableCamera,
}
