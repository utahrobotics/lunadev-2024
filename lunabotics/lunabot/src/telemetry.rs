use std::{
    net::SocketAddrV4,
    ops::Deref,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use image::RgbImage;
use lunabot_lib::{
    make_negotiation, ArmParameters, Audio, CameraMessage, ControlsPacket, ImportantMessage,
    Steering,
};
use networking::{
    negotiation::{ChannelNegotiation, Negotiation},
    new_client, ConnectionError, NetworkConnector, NetworkNode,
};
use ordered_float::NotNan;
use serde::Deserialize;
use unros::{
    anyhow,
    logging::{
        dump::{ScalingFilter, VideoDataDump},
        get_log_pub,
    },
    node::{AsyncNode, SyncNode},
    pubsub::{MonoPublisher, Publisher, PublisherRef, Subscriber, WatchSubscriber},
    runtime::RuntimeContext,
    setup_logging,
    tokio::{self, task::spawn_blocking},
    DontDrop, ShouldNotDrop,
};

use crate::{
    audio::{pause_buzz, play_buzz},
    CAMERA_HEIGHT, CAMERA_WIDTH, EMPTY_ROW, MAX_CAMERA_COUNT, ROW_DATA_LENGTH, ROW_LENGTH,
};

#[derive(Deserialize)]
struct TelemetryConfig {
    server_addr: SocketAddrV4,
}

/// A remote connection to `Lunabase`
#[derive(ShouldNotDrop)]
pub struct Telemetry {
    network_node: NetworkNode,
    network_connector: NetworkConnector,
    pub server_addr: SocketAddrV4,
    pub camera_delta: Duration,
    steering_signal: Publisher<Steering>,
    arm_signal: Publisher<ArmParameters>,
    dont_drop: DontDrop<Self>,
    negotiation: Negotiation<(
        ChannelNegotiation<ImportantMessage>,
        ChannelNegotiation<CameraMessage>,
        ChannelNegotiation<u8>,
        ChannelNegotiation<ControlsPacket>,
        ChannelNegotiation<Arc<str>>,
        ChannelNegotiation<Audio>,
    )>,
    video_addr: SocketAddrV4,
    cam_width: u32,
    cam_height: u32,
    cam_fps: usize,
    camera_subs: Vec<WatchSubscriber<RgbImage>>,
}

impl Telemetry {
    pub async fn new(
        cam_fps: usize,
        camera_subs: Vec<WatchSubscriber<RgbImage>>,
    ) -> anyhow::Result<Self> {
        assert!(camera_subs.len() < MAX_CAMERA_COUNT, "Too many cameras!");
        let config: TelemetryConfig = unros::get_env()?;
        let mut video_addr = config.server_addr;
        video_addr.set_port(video_addr.port() + 1);

        let (network_node, network_connector) = new_client()?;

        Ok(Self {
            network_node,
            network_connector,
            server_addr: config.server_addr,
            steering_signal: Publisher::default(),
            arm_signal: Publisher::default(),
            camera_delta: Duration::from_millis((1000 / cam_fps) as u64),
            dont_drop: DontDrop::new("telemetry"),
            negotiation: make_negotiation(),
            cam_width: CAMERA_WIDTH * ROW_LENGTH as u32,
            cam_height: CAMERA_HEIGHT * MAX_CAMERA_COUNT.div_ceil(ROW_LENGTH) as u32,
            video_addr,
            cam_fps,
            camera_subs,
        })
    }

    pub fn steering_pub(&self) -> PublisherRef<Steering> {
        self.steering_signal.get_ref()
    }

    pub fn arm_pub(&self) -> PublisherRef<ArmParameters> {
        self.arm_signal.get_ref()
    }
}

impl AsyncNode for Telemetry {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        // self.network_node
        //     .get_intrinsics()
        //     .manually_run(context.get_name().clone());

        self.dont_drop.ignore_drop = true;
        let sdp: Arc<str> =
            Arc::from(VideoDataDump::generate_sdp(self.video_addr).into_boxed_str());
        let enable_camera = Arc::new(AtomicBool::default());
        let enable_camera2 = enable_camera.clone();

        let context2 = context.clone();

        let cam_fut = async {
            loop {
                let mut video_dump;
                loop {
                    if context2.is_runtime_exiting() {
                        return Ok(());
                    }
                    if enable_camera.load(Ordering::Relaxed) {
                        loop {
                            match VideoDataDump::new_rtp(
                                self.cam_width,
                                self.cam_height,
                                self.cam_width,
                                self.cam_height,
                                ScalingFilter::Neighbor,
                                self.video_addr,
                                self.cam_fps,
                                &context2,
                            ) {
                                Ok(x) => {
                                    video_dump = x;
                                    break;
                                }
                                Err(e) => error!("Failed to create video dump: {e}"),
                            }
                            let start_service = Instant::now();
                            while start_service.elapsed().as_millis() < 2000 {
                                if context2.is_runtime_exiting() {
                                    return Ok(());
                                }
                                tokio::time::sleep(self.camera_delta).await;
                            }
                        }
                        break;
                    }
                    tokio::time::sleep(self.camera_delta).await;
                }
                let mut start_service = Instant::now();
                loop {
                    if context2.is_runtime_exiting() {
                        return Ok(());
                    }
                    if !enable_camera.load(Ordering::Relaxed) {
                        drop(video_dump);
                        break;
                    }
                    let mut updated = false;
                    self.camera_subs
                        .iter_mut()
                        .for_each(|sub| updated |= WatchSubscriber::try_update(sub));
                    if updated {
                        for row in self.camera_subs.chunks(ROW_LENGTH) {
                            for y in 0..CAMERA_HEIGHT as usize {
                                for i in 0..ROW_LENGTH {
                                    let row_data = if let Some(img) = row.get(i) {
                                        img.deref()
                                            .split_at(ROW_DATA_LENGTH * y)
                                            .1
                                            .split_at(ROW_DATA_LENGTH)
                                            .0
                                    } else {
                                        &EMPTY_ROW
                                    };
                                    if let Err(e) = video_dump.write_raw(row_data).await {
                                        error!("Failed to write camera data: {e}");
                                    }
                                }
                            }
                        }
                        // println!("{}", MAX_CAMERA_COUNT
                        // .next_multiple_of(ROW_LENGTH)
                        // .saturating_sub(self.camera_subs.len()));
                        for _ in 0..MAX_CAMERA_COUNT
                            .next_multiple_of(ROW_LENGTH)
                            .saturating_sub(self.camera_subs.len().next_multiple_of(ROW_LENGTH))
                        {
                            for _ in 0..CAMERA_HEIGHT as usize {
                                if let Err(e) = video_dump.write_raw(&EMPTY_ROW).await {
                                    error!("Failed to write camera data: {e}");
                                }
                            }
                        }
                    }

                    let elapsed = start_service.elapsed();
                    start_service += elapsed;
                    tokio::time::sleep(self.camera_delta).await;
                }
            }
        };
        let enable_camera = enable_camera2;

        let peer_fut = async {
            loop {
                info!("Connecting to lunabase...");
                let peer = loop {
                    match self
                        .network_connector
                        .connect_to(self.server_addr.into(), &12u8)
                        .await
                    {
                        Ok(x) => break x,
                        Err(ConnectionError::ServerDropped) => return Ok(()),
                        Err(ConnectionError::Timeout) => {}
                    };
                };
                let (important, camera, _odometry, controls, logs, audio) =
                    match peer.negotiate(&self.negotiation).await {
                        Ok(x) => x,
                        Err(e) => {
                            error!("Failed to negotiate with lunabase!: {e:?}");
                            continue;
                        }
                    };
                enable_camera.store(true, Ordering::Relaxed);
                info!("Connected to lunabase!");
                get_log_pub().accept_subscription(logs.create_reliable_subscription());

                let important_fut = async {
                    let mut _important_pub =
                        MonoPublisher::from(important.create_reliable_subscription());
                    let important_sub = Subscriber::new(8);
                    important.accept_subscription(important_sub.create_subscription());

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
                            ImportantMessage::EnableCamera => {
                                enable_camera.store(true, Ordering::Relaxed)
                            }
                            ImportantMessage::DisableCamera => {
                                enable_camera.store(false, Ordering::Relaxed)
                            }
                        }
                    }
                };

                let steering_fut = async {
                    let mut controls_pub =
                        MonoPublisher::from(controls.create_unreliable_subscription());
                    let controls_sub = Subscriber::new(1);
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
                        self.steering_signal.set(Steering::from_drive_and_steering(
                            NotNan::new(controls.drive as f32 / 127.0).unwrap(),
                            NotNan::new(controls.steering as f32 / 127.0).unwrap(),
                        ));
                        self.arm_signal.set(controls.arm_params);
                    }
                };

                let camera_fut = async {
                    let camera_pub = Publisher::default();
                    let camera_sub = Subscriber::new(1);
                    camera.accept_subscription(camera_sub.create_subscription());
                    camera_pub.accept_subscription(camera.create_reliable_subscription());
                    camera_pub.set(CameraMessage::Sdp(sdp.clone()));

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
                    }
                };

                let audio_fut = async {
                    let audio_sub = Subscriber::new(1);
                    audio.accept_subscription(audio_sub.create_subscription());

                    loop {
                        let Some(result) = audio_sub.recv_or_closed().await else {
                            break;
                        };
                        let msg = match result {
                            Ok(x) => x,
                            Err(e) => {
                                error!("Error receiving audio msg: {e}");
                                continue;
                            }
                        };

                        match msg {
                            Audio::Play => play_buzz(),
                            Audio::Pause => pause_buzz(),
                        }
                    }
                };

                tokio::select! {
                    _ = steering_fut => {}
                    _ = camera_fut => {}
                    _ = important_fut => {}
                    _ = audio_fut => {}
                }
                self.steering_signal.set(Steering::default());
                self.arm_signal.set(ArmParameters::default());
                error!("Disconnected from lunabase!");
                enable_camera.store(false, Ordering::Relaxed);
            }
        };
        let context = context.clone();

        tokio::select! {
            res = cam_fut => res,
            res = peer_fut => res,
            res = spawn_blocking(|| self.network_node.run(context)) => res.unwrap(),
        }
    }
}
