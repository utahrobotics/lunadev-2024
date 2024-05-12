use networking::{
    bitcode::{self, Decode, Encode},
    negotiation::{ChannelNegotiation, Negotiation},
};
use ordered_float::{Float, FloatCore, NotNan};
use simba::scalar::SupersetOf;
use std::{ops::Deref, sync::Arc};

pub type LunaNegotiation = Negotiation<(
    ChannelNegotiation<ImportantMessage>,
    ChannelNegotiation<CameraMessage>,
    ChannelNegotiation<Odometry>,
    ChannelNegotiation<ControlsPacket>,
    ChannelNegotiation<Arc<str>>,
    ChannelNegotiation<Audio>,
)>;

pub fn make_negotiation() -> LunaNegotiation {
    Negotiation::new(
        (
            ChannelNegotiation::new("important"),
            ChannelNegotiation::new("camera"),
            ChannelNegotiation::new("odometry"),
            ChannelNegotiation::new("controls"),
            ChannelNegotiation::new("logs"),
            ChannelNegotiation::new("audio"),
        ),
        46432,
    )
    .unwrap()
}

#[derive(Clone, Copy, Encode, Decode, PartialEq, Debug)]
pub struct Odometry {
    pub arm_angle: f32,
    pub acceleration: [f32; 3],
    pub front_elevation: f32,
    pub back_elevation: f32,
}

#[derive(Clone, Copy, Encode, Decode, PartialEq, Eq, Debug)]
pub struct ArmParameters {
    pub lift: ArmAction,
    pub tilt: ArmAction,
}

#[derive(Clone, Copy, Encode, Decode, PartialEq, Eq, Debug)]
pub enum ArmAction {
    Extend,
    Retract,
    Stop,
    SetValue(u8),
    Home,
}

impl Default for ArmParameters {
    fn default() -> Self {
        ArmParameters {
            lift: ArmAction::Stop,
            tilt: ArmAction::Stop,
        }
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

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub struct Steering<N: Float + FloatCore = f32> {
    pub left: NotNan<N>,
    pub right: NotNan<N>,
}

impl<N: Float + FloatCore + std::fmt::Debug> std::fmt::Debug for Steering<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Steering")
            .field("left", self.left.deref())
            .field("right", self.right.deref())
            .finish()
    }
}

impl<N: Float + FloatCore + std::ops::MulAssign + SupersetOf<f32>> Steering<N> {
    /// Shorthand to make this struct if you know the given values are not `NaN`.
    ///
    /// # Panics
    /// Panics if either left or right are `NaN`. To handle this possibility gracefully,
    /// you should just construct this struct normally as the fields are public.
    pub fn new(left: N, right: N) -> Self {
        Self {
            left: NotNan::new(left).unwrap(),
            right: NotNan::new(right).unwrap(),
        }
    }

    pub fn from_drive_and_steering(drive: NotNan<N>, steering: NotNan<N>) -> Self {
        let mut left = drive;
        let mut right = drive;

        if steering.into_inner() > N::zero() {
            right *= (N::from_subset(&0.5) - steering.into_inner()) * N::from_subset(&2.0);
        } else {
            left *= (N::from_subset(&0.5) + steering.into_inner()) * N::from_subset(&2.0);
        }

        Self { left, right }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode)]
pub enum Audio {
    Play,
    Pause,
}

#[derive(Clone, Encode, Decode, PartialEq, Eq, Debug)]
pub enum CameraMessage {
    Sdp(Arc<str>),
}
