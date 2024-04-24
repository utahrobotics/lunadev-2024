use std::sync::Arc;

use networking::{
    bitcode::{self, Decode, Encode},
    negotiation::{ChannelNegotiation, Negotiation},
};

pub const VIDEO_WIDTH: u32 = 1280;
pub const VIDEO_HEIGHT: u32 = 720;

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

#[derive(Clone, Copy, Encode, Decode, PartialEq, Eq, Debug)]
pub enum ArmParameters {
    TiltUp,
    TiltDown,
    Stop,
    LiftArm,
    LowerArm,
    SetArm { tilt_frac: u8, lift_frac: u8 },
}

impl Default for ArmParameters {
    fn default() -> Self {
        ArmParameters::Stop
    }
}

#[derive(Clone, Copy, Encode, Decode, PartialEq, Eq, Default, Debug)]
pub struct ControlsPacket {
    pub drive: i8,
    pub steering: i8,
    pub arm_params: ArmParameters,
}

#[derive(Debug, Eq, PartialEq, Encode, Decode, Clone, Copy)]
#[repr(u8)]
pub enum ImportantMessage {
    EnableCamera,
    DisableCamera,
}
