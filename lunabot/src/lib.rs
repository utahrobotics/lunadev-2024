use std::{ops::Deref, sync::Arc};

use localization::frames::IMUFrame;
use nalgebra::Vector3;
use networking::{
    bitcode::{self, Decode, Encode},
    Channel, ChannelMap, NetworkPeer,
};
use rig::{euler_to_quat, RobotElementRef, RotationSequence, RotationType};
use serial::SerialConnection;
use unros::{bytes::Bytes, log};

#[derive(Clone, Copy, Encode, Decode, PartialEq, Eq, Default)]
pub struct ControlsPacket {
    pub drive: i8,
    pub steering: i8,
    pub arm_vel: i8,
}

#[derive(Clone)]
pub struct Channels {
    pub important: Channel<ImportantMessage>,
    pub camera: Channel<Arc<str>>,
    pub odometry: Channel<u8>,
    pub controls: Channel<ControlsPacket>,
    pub logs: Channel<Arc<str>>,
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

pub async fn open_imu(
    path: impl Into<String>,
    robot_element: RobotElementRef,
) -> SerialConnection<Bytes, IMUFrame> {
    let serial = SerialConnection::new(path.into(), 115200, true).await;
    let mut buf = vec![];
    let imu_mapper = move |x: Bytes| {
        buf.extend_from_slice(x.deref());
        let Ok(msg) = std::str::from_utf8(&buf) else {
            return None;
        };

        let Some(idx) = msg.find('\n') else {
            return None;
        };

        if idx == 0 {
            buf.remove(0);
            return None;
        }

        let line = msg.split_at(idx - 1).0;
        if line.is_empty() {
            buf.drain(0..=idx);
            return None;
        }

        let mut numbers = line.split(' ');
        let ax: f32 = numbers.next().unwrap().parse().unwrap();
        let az: f32 = numbers.next().unwrap().parse().unwrap();
        let mut ay: f32 = numbers.next().unwrap().parse().unwrap();
        let rx: f32 = numbers.next().unwrap().parse().unwrap();
        let ry: f32 = numbers.next().unwrap().parse().unwrap();
        let rz: f32 = numbers.next().unwrap().parse().unwrap();

        ay -= 9.96 * 2.0;

        if numbers.next().is_some() {
            log::warn!("Extra number from IMU!")
        }

        buf.drain(0..idx);

        Some(IMUFrame {
            acceleration: Vector3::new(ax, ay, az),
            acceleration_variance: 0.001,
            angular_velocity: euler_to_quat(
                [rx, ry, rz],
                RotationType::Extrinsic,
                RotationSequence::XYZ,
            ),
            angular_velocity_variance: 0.0001,
            robot_element: robot_element.clone(),
        })
    };
    serial.map_output(imu_mapper)
}
