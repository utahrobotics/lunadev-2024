use std::{
    net::SocketAddrV4, ops::Deref, sync::Arc, time::{Duration, Instant}
};

use global_msgs::Steering;
use image::DynamicImage;
use networking::{bitcode::{self, Decode, Encode}, ChannelId, ChannelMap, NetworkConnector, NetworkNode};
use ordered_float::NotNan;
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait,
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
    image_subscriptions: Subscriber<Arc<DynamicImage>>,
    video_dump: VideoDataDump,
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
        
        let (network_node, network_connector) = NetworkNode::new_client(1);

        Ok(Self {
            network_node,
            network_connector,
            channels,
            server_addr,
            steering_signal: Default::default(),
            image_subscriptions: Subscriber::new(1),
            camera_delta: Duration::from_millis((1000 / cam_fps) as u64),
            video_dump,
            intrinsics: Default::default(),
        })
    }

    pub fn accept_steering_sub(&mut self, sub: Subscription<Steering>) {
        self.steering_signal.accept_subscription(sub);
    }

    pub fn create_image_subscription(&self) -> Subscription<Arc<DynamicImage>> {
        self.image_subscriptions
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
        self.network_node.get_intrinsics().manually_run(context.get_name().clone());

        let context2 = context.clone();
        setup_logging!(context2);
        let sdp: Arc<str> = Arc::from(self.video_dump.generate_sdp().unwrap().into_boxed_str());
        
        let drop_check = DropCheck::default();
        let drop_observe = drop_check.get_observing();

        let cam_fut = tokio_rayon::spawn(move || {
            let mut start_service = Instant::now();
            let sleeper = SpinSleeper::default();

            loop {
                if drop_observe.has_dropped() {
                    return Ok(());
                }
                if let Some(img) = self.image_subscriptions.try_recv() {
                    self.video_dump.write_frame(img.deref().clone())?;
                }

                let elapsed = start_service.elapsed();
                start_service += elapsed;
                sleeper.sleep(self.camera_delta.saturating_sub(elapsed));
            }
        });

        let peer_fut = async {
            loop {
                info!("Connecting to lunabase...");
                let peer = loop {
                    let Some(peer) = self.network_connector.connect_to(self.server_addr).await else { continue; };
                    break peer;
                };
                info!("Connected to lunabase!");
                let important_channel = peer.create_channel::<ImportantMessage>(self.channels.important);
                let camera_channel = peer.create_channel::<Arc<str>>(self.channels.camera);
                let _odometry_channel = peer.create_channel::<u8>(self.channels.odometry);
                let controls_channel = peer.create_channel::<(f32, f32)>(self.channels.controls);
                let logs_channel = peer.create_channel::<Arc<str>>(self.channels.logs);
                log_accept_subscription(logs_channel.create_reliable_subscription());

                let important_fut = async {
                    let mut important_pub = Publisher::default();
                    let mut important_sub = Subscriber::new(8);
                    important_channel.accept_subscription(important_sub.create_subscription());
                    important_pub.accept_subscription(important_channel.create_reliable_subscription());

                    loop {
                        let Some(result) = important_sub.recv_or_closed().await else { break; };
                        let msg = match result {
                            Ok(x) => x,
                            Err(e) => {
                                error!("Error receiving important msg: {e}");
                                continue;
                            }
                        };
                        match msg {
                            ImportantMessage::EnableCamera => todo!(),
                            ImportantMessage::DisableCamera => todo!(),
                            ImportantMessage::Ping => important_pub.set(ImportantMessage::Ping),
                        }
                    }
                };

                let steering_fut = async {
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

                let camera_fut = async {
                    let mut camera_pub = Publisher::default();
                    let mut camera_sub = Subscriber::new(1);
                    camera_channel.accept_subscription(camera_sub.create_subscription());
                    camera_pub.accept_subscription(camera_channel.create_reliable_subscription());
                    camera_pub.set(sdp.clone());

                    loop {
                        let Some(result) = camera_sub.recv_or_closed().await else { break; };
                        let _ = match result {
                            Ok(x) => x,
                            Err(e) => {
                                error!("Error receiving camera msg: {e}");
                                continue;
                            }
                        };
                        
                        info!("Resending SDP");
                        camera_pub.set(sdp.clone());
                    }
                };

                tokio::select! {
                    _ = steering_fut => {}
                    _ = camera_fut => {}
                    _ = important_fut => {}
                }
                error!("Disconnected from lunabase!");
            }
        };

        tokio::select! {
            res = cam_fut => res,
            res = peer_fut => res,
            res = self.network_node.run(context) => res,
        }
    }
}
