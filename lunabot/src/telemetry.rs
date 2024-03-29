use std::{
    net::SocketAddrV4,
    sync::Arc,
    time::{Duration, Instant},
};

use global_msgs::Steering;
use image::DynamicImage;
use lunabot::{make_negotiation, ControlsPacket, ImportantMessage};
use networking::{
    negotiation::{ChannelNegotiation, Negotiation}, ConnectionError, NetworkConnector, NetworkNode
};
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait, asyncify_run,
    logging::{
        dump::{ScalingFilter, VideoDataDump},
        get_log_pub,
    },
    pubsub::{subs::DirectSubscription, Publisher, PublisherRef, Subscriber},
    setup_logging, tokio, DropCheck, Node, NodeIntrinsics, RuntimeContext,
};

/// A remote connection to `Lunabase`
pub struct Telemetry {
    network_node: NetworkNode,
    network_connector: NetworkConnector,
    pub server_addr: SocketAddrV4,
    pub camera_delta: Duration,
    steering_signal: Publisher<Steering>,
    image_subscriptions: Subscriber<Arc<DynamicImage>>,
    video_dump: VideoDataDump,
    intrinsics: NodeIntrinsics<Self>,
    negotiation: Negotiation<(
        ChannelNegotiation<ImportantMessage>,
        ChannelNegotiation<Arc<str>>,
        ChannelNegotiation<u8>,
        ChannelNegotiation<ControlsPacket>,
        ChannelNegotiation<Arc<str>>,
    )>,
}
// #[derive(Clone)]
// pub struct Channels {
//     pub important: Channel<ImportantMessage>,
//     pub camera: Channel<Arc<str>>,
//     pub odometry: Channel<u8>,
//     pub controls: Channel<ControlsPacket>,
//     pub logs: Channel<Arc<str>>,
// }

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

        let (network_node, network_connector) = NetworkNode::new_client(1);

        Ok(Self {
            network_node,
            network_connector,
            server_addr,
            steering_signal: Publisher::default(),
            image_subscriptions: Subscriber::new(1),
            camera_delta: Duration::from_millis((1000 / cam_fps) as u64),
            video_dump,
            intrinsics: Default::default(),
            negotiation: make_negotiation(),
        })
    }

    pub fn steering_pub(&self) -> PublisherRef<Steering> {
        self.steering_signal.get_ref()
    }

    pub fn create_image_subscription(&self) -> DirectSubscription<Arc<DynamicImage>> {
        self.image_subscriptions.create_subscription()
    }
}

#[async_trait]
impl Node for Telemetry {
    const DEFAULT_NAME: &'static str = "telemetry";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        self.network_node
            .get_intrinsics()
            .manually_run(context.get_name().clone());

        let context2 = context.clone();
        setup_logging!(context2);
        let sdp: Arc<str> = Arc::from(self.video_dump.generate_sdp().unwrap().into_boxed_str());

        let drop_check = DropCheck::default();
        let drop_observe = drop_check.get_observing();

        let cam_fut = asyncify_run(move || {
            let mut start_service = Instant::now();
            let sleeper = SpinSleeper::default();

            loop {
                if drop_observe.has_dropped() {
                    return Ok(());
                }
                if let Some(img) = self.image_subscriptions.try_recv() {
                    self.video_dump.write_frame(img.clone())?;
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
                    match self
                        .network_connector
                        .connect_to(self.server_addr, &12u8)
                        .await
                    {
                        Ok(x) => break x,
                        Err(ConnectionError::ServerDropped) => return Ok(()),
                        Err(ConnectionError::Timeout) => {}
                    };
                };
                let Some((important, camera, _odometry, controls, logs)) =
                    peer.negotiate(&self.negotiation)
                else {
                    error!("Failed to negotiate with lunabase!");
                    continue;
                };
                info!("Connected to lunabase!");
                get_log_pub().accept_subscription(logs.create_reliable_subscription());

                let important_fut = async {
                    let important_pub = Publisher::default();
                    let important_sub = Subscriber::new(8);
                    important.accept_subscription(important_sub.create_subscription());
                    important_pub.accept_subscription(important.create_reliable_subscription());

                    loop {
                        let Some(result) = important_sub.recv_or_closed().await else {
                            break;
                        };
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
                            ImportantMessage::Ping => {
                                important_pub.set(ImportantMessage::Ping);
                            }
                        }
                    }
                };

                let steering_fut = async {
                    let controls_pub = Publisher::default();
                    let controls_sub = Subscriber::new(1);
                    controls_pub.accept_subscription(controls.create_unreliable_subscription());
                    controls.accept_subscription(controls_sub.create_subscription());
                    loop {
                        let Some(result) = controls_sub.recv_or_closed().await else {
                            break;
                        };
                        let controls = match result {
                            Ok(x) => x,
                            Err(e) => {
                                error!("Error receiving steering: {e}");
                                continue;
                            }
                        };
                        controls_pub.set(controls);
                        self.steering_signal.set(Steering::new(
                            controls.drive as f32 / 127.0,
                            controls.steering as f32 / 127.0,
                        ));
                    }
                };

                let camera_fut = async {
                    let camera_pub = Publisher::default();
                    let camera_sub = Subscriber::new(1);
                    camera.accept_subscription(camera_sub.create_subscription());
                    camera_pub.accept_subscription(camera.create_reliable_subscription());
                    camera_pub.set(sdp.clone());

                    loop {
                        let Some(result) = camera_sub.recv_or_closed().await else {
                            break;
                        };
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
