use std::{
    net::SocketAddrV4,
    ops::{Deref, DerefMut},
    sync::Arc,
    time::{Duration, Instant},
};

use crossbeam::queue::SegQueue;
use enet::{BandwidthLimit, ChannelLimit, Enet, Event, Host, Packet, PacketMode, PeerState};
use global_msgs::Steering;
use image::DynamicImage;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use ordered_float::NotNan;
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait, log,
    logging::{
        dump::{ScalingFilter, VideoDataDump},
        log_accept_subscription,
    },
    pubsub::{Publisher, Subscriber, Subscription},
    setup_logging, tokio, tokio_rayon, DropCheck, Node, NodeIntrinsics, RuntimeContext,
};

#[derive(Debug, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum Channels {
    Important,
    Camera,
    Odometry,
    Controls,
    Logs,
    Max,
}

#[derive(Debug, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum ImportantMessage {
    EnableCamera,
    DisableCamera,
    Ping,
}

/// A remote connection to `Lunabase`
pub struct Telemetry {
    pub bandwidth_limit: u32,
    pub server_addr: SocketAddrV4,
    pub max_image_chunk_width: u32,
    pub camera_delta: Duration,
    steering_signal: Publisher<Steering>,
    image_subscriptions: Option<Subscriber<Arc<DynamicImage>>>,
    packet_queue: SegQueue<(Box<[u8]>, PacketMode, Channels)>,
    video_dump: Option<VideoDataDump>,
    intrinsics: NodeIntrinsics<Self>,
}

impl Telemetry {
    pub async fn new(
        server_addr: impl Into<SocketAddrV4>,
        cam_width: u32,
        cam_height: u32,
        cam_fps: usize,
    ) -> anyhow::Result<Self> {
        let server_addr = server_addr.into();
        let mut video_addr = server_addr;
        video_addr.set_port(video_addr.port() + 1);

        let video_dump = VideoDataDump::new_rtp(
            cam_width,
            cam_height,
            1280,
            720,
            ScalingFilter::FastBilinear,
            video_addr,
            cam_fps,
        )?;

        Ok(Self {
            bandwidth_limit: 0,
            server_addr,
            steering_signal: Default::default(),
            image_subscriptions: Some(Subscriber::new(1)),
            camera_delta: Duration::from_millis((1000 / cam_fps) as u64),
            packet_queue: SegQueue::new(),
            max_image_chunk_width: 32,
            video_dump: Some(video_dump),
            intrinsics: Default::default(),
        })
    }

    pub fn accept_steering_sub(&mut self, sub: Subscription<Steering>) {
        self.steering_signal.accept_subscription(sub);
    }

    pub fn create_image_subscription(&self) -> Subscription<Arc<DynamicImage>> {
        self.image_subscriptions
            .as_ref()
            .unwrap()
            .create_subscription()
    }

    fn receive_packet(
        &mut self,
        channel: u8,
        packet: Box<[u8]>,
        context: &RuntimeContext,
        sdp: &String,
        send_logs: &mut bool,
    ) {
        setup_logging!(context);
        let Ok(channel) = Channels::try_from(channel) else {
            error!("Received invalid channel: {channel}");
            return;
        };
        match channel {
            Channels::Important => {
                let Ok(msg) = ImportantMessage::try_from(packet[0]) else {
                    error!("Received invalid ImportantMessage: {}", packet[0]);
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
            Channels::Camera => match packet[0] {
                1 => {
                    info!("Resending SDP");
                    self.packet_queue.push((
                        sdp.as_bytes().iter().copied().collect(),
                        PacketMode::ReliableSequenced,
                        Channels::Camera,
                    ));
                }
                x => {
                    error!("Received invalid CameraMessage: {x}");
                }
            },
            Channels::Odometry => todo!(),
            Channels::Controls => {
                let drive = i8::from_le_bytes([packet[0]]) as f32;
                let steering = i8::from_le_bytes([packet[1]]) as f32;

                self.steering_signal.set(Steering::from_drive_and_steering(
                    NotNan::new(drive / 127.0).unwrap(),
                    NotNan::new(steering / 127.0).unwrap(),
                ));

                self.packet_queue.push((
                    packet,
                    PacketMode::UnreliableUnsequenced,
                    Channels::Controls,
                ));
            }
            Channels::Max => error!("Received invalid channel: {}", channel as u8),
            Channels::Logs => {
                *send_logs = !*send_logs;
            }
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
            if peer.state() == PeerState::Connected {
                peer.disconnect(0);
                peer_count += 1;
            }
        }
        while peer_count > 0 {
            let mut event = match self.0.service(1000) {
                Ok(Some(event)) => event,
                Ok(None) => continue,
                Err(e) => {
                    log::error!("Faced the following error while safely dropping ENet host: {e}");
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

#[async_trait]
impl Node for Telemetry {
    const DEFAULT_NAME: &'static str = "telemetry";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let mut logs_sub = Subscriber::new(32);
        log_accept_subscription(logs_sub.create_subscription());

        let enet = Enet::new()?;
        let outgoing_limit = if self.bandwidth_limit == 0 {
            BandwidthLimit::Unlimited
        } else {
            BandwidthLimit::Limited(self.bandwidth_limit)
        };

        let mut drop_check = DropCheck::default();
        let drop_check2 = drop_check.clone();
        let _drop_check = drop_check.clone();
        let mut image_subscriptions = self.image_subscriptions.take().unwrap();
        let mut video_dump = self.video_dump.take().unwrap();
        let sdp = video_dump.generate_sdp().unwrap();

        let cam_fut = tokio_rayon::spawn(move || {
            let mut start_service = Instant::now();
            let sleeper = SpinSleeper::default();
            drop_check.dont_update_on_drop();

            loop {
                if drop_check.has_dropped() {
                    return Ok(());
                }
                if let Some(img) = image_subscriptions.try_recv() {
                    video_dump.write_frame(img.deref().clone())?;
                }

                let elapsed = start_service.elapsed();
                start_service += elapsed;
                sleeper.sleep(self.camera_delta.saturating_sub(elapsed));
            }
        });

        info!("Connecting to lunabase...");
        let enet_fut = tokio_rayon::spawn(move || loop {
            let tmp_host = enet.create_host::<()>(
                None,
                1,
                ChannelLimit::Maximum,
                BandwidthLimit::Unlimited,
                outgoing_limit,
            )?;
            let mut host = HostWrapper(tmp_host);
            host.connect(&self.server_addr.into(), Channels::Max as usize, 0)?;
            loop {
                if drop_check2.has_dropped() {
                    return Ok(());
                }
                let Some(event) = host.service(50)? else {
                    continue;
                };
                match event {
                    Event::Connect(_) => break,
                    Event::Disconnect(_, _) => {
                        drop(event);
                        host.connect(&self.server_addr.into(), Channels::Max as usize, 0)?;
                        continue;
                    }

                    Event::Receive { ref sender, .. } => {
                        warn!(
                            "Somehow received from a peer ({:?}:{})! ignoring...",
                            sender.address().ip(),
                            sender.address().port()
                        );
                        continue;
                    }
                }
            }
            let mut peer = host.peers().next().unwrap();
            peer.send_packet(
                Packet::new(sdp.as_bytes(), PacketMode::ReliableSequenced)?,
                Channels::Camera as u8,
            )?;
            let mut send_logs = true;

            info!("Connected to lunabase!");
            loop {
                {
                    let option = host.service(50)?;
                    if drop_check2.has_dropped() {
                        return Ok(());
                    }
                    match option {
                        Some(Event::Connect(_)) => {
                            error!("Somehow connected to another peer, ignoring...");
                        }
                        Some(Event::Disconnect(_, _)) => {
                            error!("Disconnected from lunabase!");
                            break;
                        }
                        Some(Event::Receive {
                            channel_id,
                            ref packet,
                            ..
                        }) => {
                            let packet = packet.data().to_vec().into_boxed_slice();
                            self.receive_packet(channel_id, packet, &context, &sdp, &mut send_logs);
                        }
                        None => {}
                    }
                }
                let mut peer = host.peers().next().unwrap();
                while let Some((body, mode, channel)) = self.packet_queue.pop() {
                    peer.send_packet(Packet::new(&body, mode)?, channel as u8)?;
                }
                if send_logs {
                    while let Some(log) = logs_sub.try_recv() {
                        peer.send_packet(
                            Packet::new(log.as_bytes(), PacketMode::ReliableSequenced)?,
                            Channels::Logs as u8,
                        )?;
                    }
                }
            }
            info!("Connecting to lunabase...");
        });

        tokio::select! {
            res = enet_fut => res,
            res = cam_fut => res,
        }
    }
}
