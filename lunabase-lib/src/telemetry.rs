use std::{
    net::{Ipv4Addr, SocketAddrV4},
    process::Stdio,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use crossbeam::{atomic::AtomicCell, queue::SegQueue};
use godot::{engine::notify::NodeNotification, obj::BaseMut, prelude::*};
use lunabot::{make_negotiation, ControlsPacket, ImportantMessage};
use networking::new_server;
use unros::{
    default_run_options,
    pubsub::{MonoPublisher, Subscriber},
    setup_logging, start_unros_runtime, tokio, Application,
};

struct LunabotShared {
    base_mut_queue: SegQueue<Box<dyn FnOnce(BaseMut<LunabotConn>) + Send>>,
    controls_data: AtomicCell<ControlsPacket>,
    echo_controls: AtomicBool,
    connected: AtomicBool,
    enable_camera: AtomicBool,
}

#[derive(GodotClass)]
#[class(base=Node)]
struct LunabotConn {
    shared: Option<Arc<LunabotShared>>,
    base: Base<Node>,
    thr: Option<JoinHandle<()>>,
}

#[godot_api]
impl INode for LunabotConn {
    fn init(base: Base<Node>) -> Self {
        let shared = LunabotShared {
            base_mut_queue: SegQueue::default(),
            controls_data: AtomicCell::default(),
            echo_controls: AtomicBool::default(),
            connected: AtomicBool::default(),
            enable_camera: AtomicBool::new(true),
        };
        let shared = Arc::new(shared);

        Self {
            shared: Some(shared),
            base,
            thr: None,
        }
    }

    fn process(&mut self, _delta: f64) {
        if let Some(shared) = self.shared.as_ref().cloned() {
            while let Some(func) = shared.base_mut_queue.pop() {
                func(self.base_mut());
            }
        }
    }

    fn ready(&mut self) {
        self.base().get_tree().unwrap().set_auto_accept_quit(false);
        std::fs::File::create("camera-ffmpeg.log").expect("camera-ffmpeg.log should be writable");
        let shared = self.shared.clone().unwrap();

        let main = |mut app: Application| async move {
            let peer_sub = Subscriber::new(1);
            let (network_node, _) = new_server::<u8, _>(
                SocketAddrV4::new(Ipv4Addr::from_bits(0), LunabotConn::RECV_FROM),
                peer_sub.create_subscription(),
            )?;

            app.add_task(|mut context| async move {
                context.set_quit_on_drop(true);
                setup_logging!(context);
                let negotiation = make_negotiation();

                loop {
                    let peer;
                    tokio::select! {
                        result = peer_sub.recv_or_closed() => {
                            let Some(result) = result else {
                                error!("Server closed itself");
                                godot_error!("Server closed itself");
                                break Ok(());
                            };

                            peer = match result {
                                Ok((_, x)) => x,
                                Err(e) => {
                                    error!("Invalid init data: {e}");
                                    godot_error!("Invalid init data: {e}");
                                    continue;
                                }
                            };
                        }
                        _ = async {
                            loop {
                                tokio::time::sleep(Duration::from_millis(100)).await;
                                if Arc::strong_count(&shared) == 1 {
                                    break;
                                }
                            }
                        } => {
                            break Ok(());
                        }
                    }

                    let (important, camera, _odometry, controls, logs) = match peer.negotiate(&negotiation).await {
                        Ok(x) => x,
                        Err(e) => {
                            error!("Failed to negotiate with lunabot! {e:?}");
                            godot_error!("Failed to negotiate with lunabot! {e:?}");
                            continue;
                        }
                    };

                    shared.base_mut_queue.push(Box::new(|mut base| {
                        base.emit_signal("connected".into(), &[]);
                    }));
                    shared.echo_controls.store(false, Ordering::Relaxed);
                    shared.connected.store(true, Ordering::Relaxed);
                    let mut last_receive_time = Instant::now();
                    let mut ffplay: Option<std::process::Child> = None;
                    let mut ffplay_stderr: Option<JoinHandle<std::io::Result<u64>>> = None;

                    let logs_sub = Subscriber::new(32);
                    logs.accept_subscription(logs_sub.create_subscription());

                    let camera_sub = Subscriber::new(1);
                    camera.accept_subscription(camera_sub.create_subscription());

                    let mut controls_pub = MonoPublisher::from(controls.create_unreliable_subscription());
                    let controls_sub = Subscriber::new(1);
                    controls.accept_subscription(controls_sub.create_subscription());

                    let mut important_pub = MonoPublisher::from(important.create_reliable_subscription());
                    let important_sub = Subscriber::new(8);
                    important.accept_subscription(important_sub.create_subscription());

                    let mut last_enable_camera = shared.enable_camera.load(Ordering::Relaxed);

                    macro_rules! kill_ffplay {
                        () => {
                            if let Some(mut ffplay) = ffplay {
                                match ffplay.kill() {
                                    Ok(()) => match ffplay.wait() {
                                        Ok(x) => info!("ffplay exited with {x}"),
                                        Err(e) => error!("ffplay failed to exit {e}")
                                    }
                                    Err(e) => error!("ffplay failed to be killed {e}")
                                }
                            }
                        };
                    }

                    loop {
                        macro_rules! received {
                            () => {{
                                let tmp = last_receive_time.elapsed();
                                last_receive_time += tmp;
                                shared.base_mut_queue.push(Box::new(|mut base| {
                                    base.emit_signal("something_received".into(), &[]);
                                }));
                            }};
                        }
                        macro_rules! make_ffplay {
                            () => {{
                                let mut child = std::process::Command::new("ffplay")
                                .args([
                                    "-protocol_whitelist",
                                    "file,rtp,udp",
                                    "-i",
                                    "camera.sdp",
                                    "-flags",
                                    "low_delay",
                                    "-avioflags",
                                    "direct",
                                    "-probesize",
                                    "32",
                                    "-analyzeduration",
                                    "0",
                                    "-sync",
                                    "ext",
                                    "-framedrop",
                                ])
                                .stderr(Stdio::piped())
                                .spawn()
                                .expect("Failed to init ffplay process");

                            if let Some(mut ffplay) = ffplay.take() {
                                let _ = ffplay.kill();
                            }

                            let mut stderr = child.stderr.take().unwrap();
                            ffplay = Some(child);

                            ffplay_stderr = Some(
                            std::thread::spawn(move || {
                                std::io::copy(
                                    &mut stderr,
                                    &mut std::fs::OpenOptions::new()
                                        .append(true)
                                        .create(true)
                                        .open("camera-ffmpeg.log")
                                        .expect("camera-ffmpeg.log should be writable"),
                                )
                            }));
                            }};
                        }
                        tokio::select! {
                            result = important_sub.recv_or_closed() => {
                                let Some(result) = result else {
                                    godot_error!("important_sub closed");
                                    error!("important_sub closed");
                                    break;
                                };
                                received!();

                                let msg = match result {
                                    Ok(x) => x,
                                    Err(e) => {
                                        godot_error!("Failed to parse incoming log: {e}");
                                        continue;
                                    }
                                };

                                match msg {
                                    _ => {}
                                }
                            }
                            result = logs_sub.recv_or_closed() => {
                                let Some(result) = result else {
                                    godot_error!("logs_sub closed");
                                    error!("logs_sub closed");
                                    break;
                                };
                                received!();

                                let log = match result {
                                    Ok(x) => x,
                                    Err(e) => {
                                        godot_error!("Failed to parse incoming log: {e}");
                                        continue;
                                    }
                                };

                                godot_print!("{log}");
                            }
                            result = camera_sub.recv_or_closed() => {
                                let Some(result) = result else {
                                    godot_error!("camera_sub closed");
                                    error!("camera_sub closed");
                                    break;
                                };
                                received!();

                                let sdp = match result {
                                    Ok(x) => x,
                                    Err(e) => {
                                        godot_error!("Failed to parse incoming sdp: {e}");
                                        error!("Failed to parse incoming sdp: {e}");
                                        continue;
                                    }
                                };

                                std::fs::write("camera.sdp", sdp.as_bytes())
                                            .expect("camera.sdp should be writable");

                                make_ffplay!();
                            }
                            result = controls_sub.recv_or_closed() => {
                                let Some(result) = result else {
                                    godot_error!("controls_sub closed");
                                    error!("controls_sub closed");
                                    break;
                                };
                                received!();

                                let controls = match result {
                                    Ok(x) => x,
                                    Err(e) => {
                                        godot_error!("Failed to parse incoming controls: {e}");
                                        continue;
                                    }
                                };

                                shared.echo_controls.store(controls != shared.controls_data.load(), Ordering::Relaxed);
                            }
                            _ = tokio::time::sleep(Duration::from_millis(50)) => {}
                        }

                        if Arc::strong_count(&shared) == 1 {
                            kill_ffplay!();
                            return Ok(());
                        }

                        if shared.echo_controls.load(Ordering::Relaxed) {
                            controls_pub.set(shared.controls_data.load());
                        }
                        
                        if let Some(inner) = &mut ffplay_stderr {
                            if inner.is_finished() {
                                if let Err(e) = ffplay_stderr.take().unwrap().join().unwrap() {
                                    error!("{e}");
                                    godot_error!("{e}");
                                }
                            }
                        }

                        let current_enable_camera = shared.enable_camera.load(Ordering::Relaxed);
                        if current_enable_camera != last_enable_camera {
                            last_enable_camera = current_enable_camera;
                            if current_enable_camera {
                                important_pub.set(ImportantMessage::EnableCamera);
                                
                                if ffplay_stderr.is_none() {
                                    make_ffplay!();
                                }
                            } else {
                                important_pub.set(ImportantMessage::DisableCamera);
                            }
                        }
                    }

                    shared.connected.store(false, Ordering::Relaxed);
                    shared.base_mut_queue.push(Box::new(|mut base| {
                        base.emit_signal("disconnected".into(), &[]);
                    }));
                    kill_ffplay!();
                }
            }, "conn-receiver");

            app.add_node(network_node);
            Ok(app)
        };
        self.thr = Some(std::thread::spawn(|| {
            let mut run_options = default_run_options!();
            run_options.enable_console_subscriber = false;
            run_options.auxilliary_control = false;
            if let Err(e) = start_unros_runtime(main, run_options) {
                godot_error!("{e}");
                unros::log::error!("{e}");
            }
        }));
    }

    fn on_notification(&mut self, notif: NodeNotification) {
        if notif == NodeNotification::WmCloseRequest {
            self.shared = None;
            if let Some(thr) = self.thr.take() {
                let _ = thr.join();
            }

            self.base()
                .get_tree()
                .unwrap()
                .call_deferred("quit".into(), &[]);
        }
    }
}

#[godot_api]
impl LunabotConn {
    #[constant]
    const RECV_FROM: u16 = 43721;
    #[constant]
    const IMAGE_WIDTH: usize = 1280;
    #[constant]
    const IMAGE_HEIGHT: usize = 720;

    #[signal]
    fn connected(&self);

    #[signal]
    fn disconnected(&self);

    #[signal]
    fn something_received(&self);

    #[signal]
    fn odometry_received(&self, position: Vector2);

    #[signal]
    fn network_statistics(
        &self,
        total_bytes: f32,
        ping: i32,
        packet_loss: f32,
        packet_throttle: f32,
    );

    #[func]
    fn is_lunabot_connected(&mut self) -> bool {
        self.shared
            .as_ref()
            .unwrap()
            .connected
            .load(Ordering::Relaxed)
    }

    #[func]
    fn send_steering(&self, mut drive: f32, mut steering: f32) {
        if drive > 1.0 {
            drive = 1.0;
            godot_warn!("Drive greater than 1!")
        }
        if drive < -1.0 {
            drive = -1.0;
            godot_warn!("Drive lesser than -1!");
        }
        if steering > 1.0 {
            steering = 1.0;
            godot_warn!("Steering greater than 1!")
        }
        if steering < -1.0 {
            steering = -1.0;
            godot_warn!("Steering lesser than -1!")
        }

        let shared = self.shared.as_ref().unwrap();
        let mut controls_packet = shared.controls_data.load();
        controls_packet.drive = (drive * 127.0).round() as i8;
        controls_packet.steering = (steering * 127.0).round() as i8;
        shared.controls_data.store(controls_packet);
        shared.echo_controls.store(true, Ordering::Relaxed);
    }

    #[func]
    fn send_arm_controls(&self, mut arm_vel: f32) {
        if arm_vel > 1.0 {
            arm_vel = 1.0;
            godot_warn!("arm_vel greater than 1!")
        }
        if arm_vel < -1.0 {
            arm_vel = -1.0;
            godot_warn!("arm_vel lesser than -1!")
        }

        let shared = self.shared.as_ref().unwrap();
        let mut controls_packet = shared.controls_data.load();
        controls_packet.arm_vel = (arm_vel * 127.0).round() as i8;
        shared.controls_data.store(controls_packet);
        shared.echo_controls.store(true, Ordering::Relaxed);
    }

    #[func]
    fn enable_camera(&self) {
        let shared = self.shared.as_ref().unwrap();
        shared.enable_camera.store(true, Ordering::Relaxed);
    }

    #[func]
    fn disable_camera(&self) {
        let shared = self.shared.as_ref().unwrap();
        shared.enable_camera.store(false, Ordering::Relaxed);
    }
}
