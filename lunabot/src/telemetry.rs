use std::{
    net::SocketAddrV4,
    ops::{Deref, DerefMut},
    sync::Arc,
    time::{Duration, Instant},
};

use global_msgs::Steering;
use image::DynamicImage;
use networking::{bitcode::{self, Decode, Encode}, ChannelId, ChannelMap, NetworkConnector, NetworkNode};
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


struct Channels {
    important: ChannelId,
    camera: ChannelId,
    odometry: ChannelId,
    controls: ChannelId,
    logs: ChannelId
}

#[derive(Debug, Eq, PartialEq, Encode, Decode, Clone, Copy)]
enum ImportantMessage {
    EnableCamera,
    DisableCamera,
    Ping,
}

/// A remote connection to `Lunabase`
pub struct Telemetry {
    network_node: NetworkNode,
    network_connector: NetworkConnector,
    channels: Channels,
    pub server_addr: SocketAddrV4,
    pub camera_delta: Duration,
    steering_signal: Publisher<Steering>,
    image_subscriptions: Option<Subscriber<Arc<DynamicImage>>>,
    // packet_queue: SegQueue<(Box<[u8]>, PacketMode, Channels)>,
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

        let mut channel_map = ChannelMap::new(2342);

        let channels = Channels {
            important: channel_map.add_channel("important").unwrap(),
            camera: channel_map.add_channel("camera").unwrap(),
            odometry: channel_map.add_channel("odometry").unwrap(),
            controls: channel_map.add_channel("controls").unwrap(),
            logs: channel_map.add_channel("logs").unwrap(),
        };
        
        let (network_node, network_connector) = NetworkNode::new_client();

        Ok(Self {
            network_node,
            network_connector,
            channels,
            server_addr,
            steering_signal: Default::default(),
            image_subscriptions: Some(Subscriber::new(1)),
            camera_delta: Duration::from_millis((1000 / cam_fps) as u64),
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
}


#[async_trait]
impl Node for Telemetry {
    const DEFAULT_NAME: &'static str = "telemetry";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        let context2 = context.clone();
        let fut = async {
            loop {
                let peer = loop {
                    let Some(peer) = self.network_connector.connect_to(self.server_addr).await else { continue; };
                    break peer;
                };
                let important_channel = peer.create_channel::<ImportantMessage>(self.channels.important);
                let camera_channel = peer.create_channel::<bool>(self.channels.camera);
                // let odometry_channel = peer.create_channel::<u8>(self.channels.odometry);
                let controls_channel = peer.create_channel::<(f32, f32)>(self.channels.controls);
                let logs_channel = peer.create_channel::<Arc<str>>(self.channels.logs);
                log_accept_subscription(logs_channel.create_reliable_subscription());

                let context3 = context2.clone();
                let steering_fut = async {
                    setup_logging!(context3);
                    let mut steering_sub = Subscriber::new(1);
                    controls_channel.accept_subscription(steering_sub.create_subscription());
                    loop {
                        let Some(result) = steering_sub.recv_or_closed().await else { break; };
                        let (drive, steering) = match result {
                            Ok(x) => x,
                            Err(e) => {
                                error!("Error receiving steering: {e}");
                                continue;
                            }
                        };
                        let Ok(drive) = NotNan::new(drive) else {
                            error!("drive is NaN");
                            continue;
                        };
                        let Ok(steering) = NotNan::new(steering) else {
                            error!("steering is NaN");
                            continue;
                        };
                        self.steering_signal.set(Steering::from_drive_and_steering(drive, steering));
                    }
                };

                tokio::select! {
                    _ = steering_fut => 
                }
            }
        };
        tokio::select! {
            res = fut => res,
            res = self.network_node.run(context) => res,
        }
    }
}
