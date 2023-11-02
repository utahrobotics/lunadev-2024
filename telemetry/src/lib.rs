use std::{
    ops::{Deref, DerefMut},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    }, time::Duration,
};

use crossbeam::queue::SegQueue;
use enet::{Address, BandwidthLimit, ChannelLimit, Enet, Event, Host, Packet, PacketMode};
use global_msgs::Steering;
use image::{DynamicImage, EncodableLayout};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use ordered_float::NotNan;
use unros_core::{
    anyhow, async_trait,
    log::{warn, error},
    node_error, node_info, node_warn,
    tokio::runtime::Handle,
    tokio_rayon::{self},
    Node, Signal,
};

#[derive(Debug, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum Channels {
    Important,
    Camera,
    Odometry,
    Controls,
    Max,
}

#[derive(Debug, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum ImportantMessage {
    EnableCamera,
    DisableCamera,
    Ping,
}

pub struct Telemetry {
    name: String,
    pub bandwidth_limit: u32,
    pub server_addr: Address,
    steering_signal: Signal<Steering>,
    image_queue: Arc<SegQueue<Arc<DynamicImage>>>,
    packet_queue: SegQueue<(Box<[u8]>, PacketMode, Channels)>,
}

impl Telemetry {
    pub fn new(server_addr: impl Into<Address>) -> Self {
        Self {
            name: "telemetry".into(),
            bandwidth_limit: 0,
            server_addr: server_addr.into(),
            steering_signal: Default::default(),
            image_queue: Arc::new(SegQueue::new()),
            packet_queue: SegQueue::new(),
        }
    }

    pub fn subscribe_to_image(&self, signal: &mut Signal<Arc<DynamicImage>>) {
        let queue = self.image_queue.clone();
        signal.connect_to(move |img| {
            queue.push(img);
        });
    }

    pub fn get_steering_signal(&mut self) -> &mut Signal<Steering> {
        &mut self.steering_signal
    }

    async fn receive_packet(&self, channel: u8, packet: Box<[u8]>) {
        let Ok(channel) = Channels::try_from(channel) else {
            node_error!(self, "Received invalid channel: {channel}");
            return;
        };
        match channel {
            Channels::Important => {
                let Ok(msg) = ImportantMessage::try_from(packet[0]) else {
                    node_error!(self, "Received invalid ImportantMessage: {}", packet[0]);
                    return;
                };
                match msg {
                    ImportantMessage::EnableCamera => todo!(),
                    ImportantMessage::DisableCamera => todo!(),
                    ImportantMessage::Ping => self.packet_queue.push((
                        packet,
                        PacketMode::ReliableSequenced,
                        Channels::Important,
                    )),
                }
            }
            Channels::Camera => todo!(),
            Channels::Odometry => todo!(),
            Channels::Controls => {
                let drive = i8::from_le_bytes([packet[0]]) as f32;
                let steering = i8::from_le_bytes([packet[1]]) as f32;

                self.steering_signal
                    .emit(Steering {
                        drive: NotNan::new(drive / 127.0).unwrap(),
                        steering: NotNan::new(steering / 127.0).unwrap(),
                    })
                    .await;

                self.packet_queue.push((
                    packet,
                    PacketMode::UnreliableUnsequenced,
                    Channels::Controls,
                ));
            }
            Channels::Max => node_error!(self, "Received invalid channel: {}", channel as u8),
        }
    }
}

struct HostWrapper(Host<()>);

impl Deref for HostWrapper {
    type Target = Host<()>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for HostWrapper {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Drop for HostWrapper {
    fn drop(&mut self) {
        let mut peer_count = 0;
        for mut peer in self.0.peers() {
            peer.disconnect(0);
            peer_count += 1;
        }
        while peer_count > 0 {
            let mut event = match self.0.service(1000) {
                Ok(Some(event)) => event,
                Ok(None) => continue,
                Err(e) => {
                    error!("Faced the following error while safely dropping ENet host: {e}");
                    break;
                }
            };
            match &mut event {
                Event::Connect(peer) => {
                    peer_count += 1;
                    peer.disconnect(0);
                }
                Event::Disconnect(_, _) => peer_count -= 1,
                _ => {}
            }
        }
    }
}

struct DropCheck(Arc<AtomicBool>, std::sync::mpsc::Receiver<()>);

impl Drop for DropCheck {
    fn drop(&mut self) {
        self.0.store(true, Ordering::SeqCst);
        let _ = self.1.recv();
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
        let handle = Handle::current();

        let drop_check_bool = Arc::new(AtomicBool::new(false));
        let (sender, receiver) = std::sync::mpsc::sync_channel(0);
        let _drop_check = DropCheck(drop_check_bool.clone(), receiver);

        tokio_rayon::spawn(move || {
            let _sender = sender;
            let host = enet.create_host::<()>(
                None,
                1,
                ChannelLimit::Maximum,
                BandwidthLimit::Unlimited,
                outgoing_limit,
            )?;
            let mut host = HostWrapper(host);
            host.connect(&self.server_addr, Channels::Max as usize, 0)?;

            loop {
                node_info!(self, "Connecting to lunabase...");
                loop {
                    let Some(event) = host.service(300)? else {
                        continue;
                    };
                    match event {
                        Event::Connect(_) => break,
                        Event::Disconnect(_, _) => {
                            node_warn!(self, "Somehow disconnected to a peer!")
                        }
                        Event::Receive { .. } => todo!(),
                    }
                }
                while let Some(_) = self.image_queue.pop() {}
                node_info!(self, "Connected to lunabase!");
                loop {
                    {
                        let option = host.service(50)?;
                        if drop_check_bool.load(Ordering::Relaxed) {
                            return Ok(());
                        }
                        match option {
                            Some(Event::Connect(_)) => {
                                node_error!(self, "Somehow connected to another peer, ignoring...");
                            }
                            Some(Event::Disconnect(_, _)) => {
                                node_error!(self, "Disconnected from lunabase!");
                                break;
                            }
                            Some(Event::Receive {
                                channel_id,
                                ref packet,
                                ..
                            }) => {
                                let self = self.clone();
                                let packet = packet.data().to_vec().into_boxed_slice();
                                handle.spawn(async move {
                                    self.receive_packet(channel_id, packet).await;
                                });
                            }
                            None => {}
                        }
                    }
                    let mut peer = host.peers().next().unwrap();
                    while let Some((body, mode, channel)) = self.packet_queue.pop() {
                        peer.send_packet(Packet::new(&body, mode)?, channel as u8)?;
                    }
                    while let Some(img) = self.image_queue.pop() {
                        let img = webp::Encoder::from_image(&img)
                            .map_err(|e| {
                                anyhow::anyhow!("Failed to encode image frame to webp: {e}")
                            })?
                            .encode(35.0);
                        peer.send_packet(
                            Packet::new(img.as_bytes(), enet::PacketMode::UnreliableUnsequenced)?,
                            Channels::Camera as u8,
                        )?;
                    }
                }
            }
        })
        .await
    }
}
