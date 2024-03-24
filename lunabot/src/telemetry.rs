use std::{
    net::SocketAddrV4, ops::Deref, sync::Arc, time::{Duration, Instant}
};

use global_msgs::Steering;
use image::DynamicImage;
use lunabot::{Channels, ImportantMessage};
use networking::{NetworkConnector, NetworkNode};
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait, asyncify_run, logging::{
        dump::{ScalingFilter, VideoDataDump},
        log_accept_subscription,
    }, pubsub::{Publisher, Subscriber, Subscription}, setup_logging, tokio, DropCheck, Node, NodeIntrinsics, RuntimeContext
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
        })
    }

    pub fn accept_steering_sub(&self, sub: Subscription<Steering>) {
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

        let cam_fut = asyncify_run(move || {
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
                let channels = Channels::new(&peer);
                log_accept_subscription(channels.logs.create_reliable_subscription());

                let important_fut = async {
                    let mut important_pub = Publisher::default();
                    let mut important_sub = Subscriber::new(8);
                    channels.important.accept_subscription(important_sub.create_subscription());
                    important_pub.accept_subscription(channels.important.create_reliable_subscription());

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
                    let mut controls_pub = Publisher::default();
                    let mut controls_sub = Subscriber::new(1);
                    controls_pub.accept_subscription(channels.controls.create_unreliable_subscription());
                    channels.controls.accept_subscription(controls_sub.create_subscription());
                    loop {
                        let Some(result) = controls_sub.recv_or_closed().await else { break; };
                        let controls = match result {
                            Ok(x) => x,
                            Err(e) => {
                                error!("Error receiving steering: {e}");
                                continue;
                            }
                        };
                        controls_pub.set(controls);
                        self.steering_signal.set(Steering::new(controls.drive as f32 / 127.0, controls.steering as f32 / 127.0));
                    }
                };

                let camera_fut = async {
                    let mut camera_pub = Publisher::default();
                    let mut camera_sub = Subscriber::new(1);
                    channels.camera.accept_subscription(camera_sub.create_subscription());
                    camera_pub.accept_subscription(channels.camera.create_reliable_subscription());
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
