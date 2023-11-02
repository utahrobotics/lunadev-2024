use std::sync::Arc;

use enet::{Address, BandwidthLimit, ChannelLimit, Enet, Event};
use global_msgs::Steering;
use num_enum::{TryFromPrimitive, IntoPrimitive};
use unros_core::{anyhow, async_trait, Node, tokio_rayon, node_error, node_warn, node_info, tokio, Signal};

#[derive(Debug, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum Channels {
    Important,
    Camera,
    Odometry,
    Controls,
    Max,
}

#[repr(u8)]
enum ImportantMessage {
    EnableCamera,
    DisableCamera,
    Ping,
}

pub struct Telemetry {
    name: String,
    bandwidth_limit: u32,
    server_addr: Address,
    steering_signal: Signal<Steering>
}

impl Telemetry {
    pub fn get_steering_signal(&mut self) -> &mut Signal<Steering> {
        &mut self.steering_signal
    }

    async fn receive_packet(&self, channel: u8, packet: Box<[u8]>) {
        let Ok(channel) = Channels::try_from(channel) else {
            node_error!(self, "Received invalid channel: {channel}");
            return;
        };
        match channel {
            Channels::Important => todo!(),
            Channels::Camera => todo!(),
            Channels::Odometry => todo!(),
            Channels::Controls => todo!(),
            Channels::Max => node_error!(self, "Received invalid channel: {}", channel as u8)
        }
    }
}

#[async_trait]
impl Node for Telemetry {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    async fn run(self) -> anyhow::Result<()> {
        let self = Arc::new(self);
        let enet = Enet::new()?;
        let outgoing_limit = if self.bandwidth_limit == 0 {
            BandwidthLimit::Unlimited
        } else {
            BandwidthLimit::Limited(self.bandwidth_limit)
        };

        tokio_rayon::spawn(move || {
            let mut host = enet.create_host::<()>(
                None,
                1,
                ChannelLimit::Maximum,
                BandwidthLimit::Unlimited,
                outgoing_limit,
            )?;
            let mut peer = host.connect(&self.server_addr, Channels::Max as usize, 0)?;

            loop {
                node_info!(self, "Connecting to lunabase...");
                peer = loop {
                    let Some(event) = host.service(100)? else { continue };
                    match event {
                        Event::Connect(ref peer) => break peer.clone(),
                        Event::Disconnect(_, _) => node_warn!(self, "Somehow disconnected to a peer!"),
                        Event::Receive { .. } => todo!(),
                    }
                };
                node_info!(self, "Connected to lunabase!");
                loop {
                    let Some(event) = host.service(100)? else { continue };
                    match event {
                        Event::Connect(ref new_peer) => {
                            node_error!(self, "Somehow connected to another peer, switching...");
                            peer = new_peer.clone();
                        }
                        Event::Disconnect(_, _) => {
                            node_error!(self, "Disconnected from lunabase!");
                            break
                        }
                        Event::Receive { channel_id, ref packet, .. } => {
                            let self = self.clone();
                            let packet = packet.data().to_vec().into_boxed_slice();
                            tokio::spawn(async move {
                                self.receive_packet(channel_id, packet).await;
                            });
                        }
                    }
                }
            }
        }).await
    }
}
