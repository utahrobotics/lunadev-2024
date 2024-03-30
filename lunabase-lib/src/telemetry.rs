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
use lunabot::{make_negotiation, ControlsPacket};
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

                    let logs_sub = Subscriber::new(32);
                    logs.accept_subscription(logs_sub.create_subscription());

                    let camera_sub = Subscriber::new(1);
                    camera.accept_subscription(camera_sub.create_subscription());

                    let mut controls_pub = MonoPublisher::from(controls.create_unreliable_subscription());
                    let controls_sub = Subscriber::new(1);
                    controls.accept_subscription(controls_sub.create_subscription());

                    let mut _important_pub = MonoPublisher::from(important.create_reliable_subscription());
                    let important_sub = Subscriber::new(8);
                    important.accept_subscription(important_sub.create_subscription());

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

                                std::thread::spawn(move || {
                                    let _ = std::io::copy(
                                        &mut stderr,
                                        &mut std::fs::OpenOptions::new()
                                            .append(true)
                                            .create(true)
                                            .open("camera-ffmpeg.log")
                                            .expect("camera-ffmpeg.log should be writable"),
                                    );
                                });
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
                            if let Some(mut ffplay) = ffplay {
                                match ffplay.kill() {
                                    Ok(()) => match ffplay.wait() {
                                        Ok(x) => info!("ffplay exited with {x}"),
                                        Err(e) => error!("ffplay failed to exit {e}")
                                    }
                                    Err(e) => error!("ffplay failed to be killed {e}")
                                }
                            }
                            return Ok(());
                        }

                        if shared.echo_controls.load(Ordering::Relaxed) {
                            controls_pub.set(shared.controls_data.load());
                        }
                    }

                    shared.connected.store(false, Ordering::Relaxed);
                    shared.base_mut_queue.push(Box::new(|mut base| {
                        base.emit_signal("disconnected".into(), &[]);
                    }));
                    if let Some(mut ffplay) = ffplay {
                        match ffplay.kill() {
                            Ok(()) => match ffplay.wait() {
                                Ok(x) => info!("ffplay exited with {x}"),
                                Err(e) => error!("ffplay failed to exit {e}")
                            }
                            Err(e) => error!("ffplay failed to be killed {e}")
                        }
                    }
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
            }
        }));
        // let task = move || 'main: loop {
        //     let mut server = ENetConnection::new_gd();
        //     let err = server.create_host_bound("*".into(), Self::RECV_FROM as i32);
        //     if err != Error::OK {
        //         godot_error!("Failed to start ENet Server: {err:?}");
        //         std::thread::sleep(Duration::from_secs(3));
        //         continue;
        //     }

        //     loop {
        //         loop {
        //             let result = server.service_ex().timeout(200).done();

        //             if Arc::strong_count(&shared) == 1 {
        //                 peer.peer_disconnect();
        //                 if let Some(mut ffplay) = ffplay {
        //                     let _ = ffplay.kill();
        //                 }
        //                 loop {
        //                     let result = server.service();
        //                     let event_type: EventType = result.get(0).to();
        //                     let new_peer: Option<Gd<ENetPacketPeer>> = result.get(1).to();

        //                     match event_type {
        //                         EventType::ERROR => {
        //                             godot_error!("Server faced an error disconnecting");
        //                             break;
        //                         }
        //                         EventType::DISCONNECT => break,
        //                         EventType::CONNECT => {
        //                             godot_error!(
        //                                 "Somehow connected to a peer: {}",
        //                                 new_peer.unwrap().get_remote_address()
        //                             );
        //                         }
        //                         EventType::RECEIVE => {}
        //                         _ => unreachable!(),
        //                     }
        //                 }
        //                 server.destroy();
        //                 return;
        //             }

        //             let receive_elapsed = last_receive_time.elapsed();
        //             if !pinged && receive_elapsed.as_secs() >= 3 {
        //                 pinged = true;
        //                 peer.send(
        //                     Channels::Important.into_godot() as i32 - 1,
        //                     std::iter::once(ImportantMessage::Ping.into_godot() - 1).collect(),
        //                     ENetPacketPeer::FLAG_RELIABLE,
        //                 );
        //             }

        //             let controls_data: Box<[u8]> = shared
        //                 .controls_data
        //                 .iter()
        //                 .map(|n| n.load(Ordering::Relaxed))
        //                 .collect();
        //             let event_type: EventType = result.get(0).to();
        //             let new_peer: Option<Gd<ENetPacketPeer>> = result.get(1).to();
        //             // let _data: i32 = result.get(2).to();
        //             let channel: u8 = result.get(3).to();
        //             let channel = Channels::from_godot(channel + 1);

        //             match event_type {
        //                 EventType::ERROR => {
        //                     server.destroy();
        //                     godot_error!("Server faced an error");
        //                     shared.connected.store(false, Ordering::Relaxed);
        //                     shared.base_mut_queue.push(Box::new(|mut base| {
        //                         base.emit_signal("disconnected".into(), &[]);
        //                     }));
        //                     continue 'main;
        //                 }
        //                 EventType::DISCONNECT => break,
        //                 EventType::CONNECT => {
        //                     godot_error!(
        //                         "Somehow connected to a peer: {}",
        //                         new_peer.unwrap().get_remote_address()
        //                     );
        //                 }
        //                 EventType::RECEIVE => {
        //                     pinged = false;
        //                     last_receive_time += receive_elapsed;
        //                     shared.base_mut_queue.push(Box::new(|mut base| {
        //                         base.emit_signal("something_received".into(), &[]);
        //                     }));
        //                     let data = peer.get_packet().to_vec();

        //                     match channel {
        //                         Channels::Important => {}
        //                         Channels::Camera => {
        //                             // let data = data.to_vec();
        //                             std::fs::write("camera.sdp", data)
        //                                 .expect("camera.sdp should be writable");

        //                             let mut child = std::process::Command::new("ffplay")
        //                                 .args([
        //                                     "-protocol_whitelist",
        //                                     "file,rtp,udp",
        //                                     "-i",
        //                                     "camera.sdp",
        //                                     "-flags",
        //                                     "low_delay",
        //                                     "-avioflags",
        //                                     "direct",
        //                                     "-probesize",
        //                                     "32",
        //                                     "-analyzeduration",
        //                                     "0",
        //                                     "-sync",
        //                                     "ext",
        //                                     "-framedrop",
        //                                 ])
        //                                 .stderr(Stdio::piped())
        //                                 .spawn()
        //                                 .expect("Failed to init ffplay process");

        //                             let mut stderr = child.stderr.take().unwrap();
        //                             ffplay = Some(child);

        //                             std::thread::spawn(move || {
        //                                 let _ = std::io::copy(
        //                                     &mut stderr,
        //                                     &mut std::fs::OpenOptions::new()
        //                                         .append(true)
        //                                         .create(true)
        //                                         .open("camera-ffmpeg.log")
        //                                         .expect("camera-ffmpeg.log should be writable"),
        //                                 );
        //                             });
        //                         }
        //                         Channels::Odometry => {
        //                             let x =
        //                                 f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        //                             let y =
        //                                 f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        //                             // if x == 0.0 || y == 0.0 {
        //                             //     godot_error!("Invalid odometry origin");
        //                             // }
        //                             let origin = Vector2::new(x, y);
        //                             shared.base_mut_queue.push(Box::new(move |mut base| {
        //                                 base.emit_signal(
        //                                     "odometry_received".into(),
        //                                     &[origin.to_variant()],
        //                                 );
        //                             }));
        //                         }
        //                         Channels::Controls => {
        //                             if data.len() != shared.controls_data.len() {
        //                                 godot_error!("Invalid controls packet");
        //                                 continue;
        //                             }
        //                             shared.echo_controls.store(
        //                                 data.as_slice() != controls_data.deref(),
        //                                 Ordering::Relaxed,
        //                             );
        //                         }
        //                         Channels::Logs => {
        //                             let Ok(log) = String::from_utf8(data) else {
        //                                 godot_error!("Failed to parse incoming log");
        //                                 continue;
        //                             };
        //                             godot_print!("{log}");
        //                         }
        //                         Channels::Max => {}
        //                     }
        //                 }
        //                 _ => {}
        //             }

        //             if shared.echo_controls.load(Ordering::Relaxed) {
        //                 peer.send(
        //                     Channels::Controls.into_godot() as i32 - 1,
        //                     controls_data.into_iter().copied().collect(),
        //                     ENetPacketPeer::FLAG_UNRELIABLE_FRAGMENT
        //                         | ENetPacketPeer::FLAG_UNSEQUENCED,
        //                 );
        //             }

        //             while let Some(packet) = shared.packet_queue.pop() {
        //                 peer.send(
        //                     packet.channel.into_godot() as i32 - 1,
        //                     packet.data.into_iter().collect(),
        //                     packet.flags,
        //                 );
        //             }
        //         }

        //         if let Some(mut ffplay) = ffplay {
        //             let _ = ffplay.kill();
        //         }
        //         shared.connected.store(false, Ordering::Relaxed);
        //         shared.base_mut_queue.push(Box::new(|mut base| {
        //             base.emit_signal("disconnected".into(), &[]);
        //         }));
        //     }
        // };
        // self.thr = Some(std::thread::spawn(task));
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
}
