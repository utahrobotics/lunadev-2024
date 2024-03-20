use std::{
    ops::Deref,
    process::Stdio,
    sync::{
        atomic::{AtomicBool, AtomicU8, Ordering},
        Arc,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use crossbeam::{atomic::AtomicCell, queue::SegQueue};
use godot::{
    engine::{
        enet_connection::EventType, global::Error, image::Format, ENetConnection, ENetPacketPeer,
        Image,
    },
    obj::BaseMut,
    prelude::*,
};

use crate::init_panic_hook;

struct Packet {
    channel: Channels,
    data: Vec<u8>,
    flags: i32,
}

struct LunabotShared {
    base_mut_queue: SegQueue<Box<dyn FnOnce(BaseMut<LunabotConn>) + Send>>,
    packet_queue: SegQueue<Packet>,
    controls_data: Box<[AtomicU8]>,
    echo_controls: AtomicBool,
    camera_frame: AtomicCell<Option<Arc<Box<[u8]>>>>,
    connected: AtomicBool,
}

#[derive(GodotClass)]
#[class(base=Node)]
struct LunabotConn {
    shared: Option<Arc<LunabotShared>>,
    base: Base<Node>,
    thr: Option<JoinHandle<()>>,
}

#[derive(GodotConvert, Var, Export, Debug)]
#[godot(via = u8)]
enum Channels {
    Important,
    Camera,
    Odometry,
    Controls,
    Logs,
    Max,
}

#[derive(GodotConvert, Var, Export, Debug)]
#[godot(via = u8)]
enum ImportantMessage {
    EnableCamera,
    DisableCamera,
    Ping,
}

#[godot_api]
impl INode for LunabotConn {
    fn init(base: Base<Node>) -> Self {
        init_panic_hook();
        let controls_data: Box<[AtomicU8]> = if Self::USE_ARCHIMEDES {
            Box::new(std::array::from_fn::<_, 4, _>(|_| AtomicU8::default()))
        } else {
            Box::new(std::array::from_fn::<_, 3, _>(|_| AtomicU8::default()))
        };

        let shared = LunabotShared {
            base_mut_queue: SegQueue::default(),
            packet_queue: SegQueue::default(),
            controls_data,
            echo_controls: AtomicBool::default(),
            camera_frame: AtomicCell::default(),
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
        while let Some(func) = self.shared.as_ref().unwrap().base_mut_queue.pop() {
            func(self.base_mut());
        }

        let shared = self.shared.as_ref().unwrap();

        if let Some(camera_frame) = shared.camera_frame.take() {
            self.base_mut().emit_signal(
                "camera_frame_received".into(),
                &[Image::create_from_data(
                    Self::IMAGE_WIDTH as i32,
                    Self::IMAGE_HEIGHT as i32,
                    false,
                    Format::RGB8,
                    camera_frame.iter().copied().collect(),
                )
                .unwrap()
                .to_variant()],
            );
        }
    }

    fn ready(&mut self) {
        std::fs::File::create("camera-ffmpeg.log").expect("camera-ffmpeg.log should be writable");

        let shared = self.shared.clone().unwrap();
        let task = move || 'main: loop {
            let mut server = ENetConnection::new_gd();
            let err = server.create_host_bound("*".into(), Self::RECV_FROM as i32);
            if err != Error::OK {
                godot_error!("Failed to start ENet Server: {err:?}");
                std::thread::sleep(Duration::from_secs(3));
                continue;
            }

            loop {
                let mut peer: Gd<ENetPacketPeer>;
                loop {
                    let result = server.service_ex().timeout(200).done();

                    if Arc::strong_count(&shared) == 1 {
                        server.destroy();
                        return;
                    }

                    let event_type: EventType = result.get(0).to();
                    let new_peer: Option<Gd<ENetPacketPeer>> = result.get(1).to();
                    // let _data: i32 = result.get(2).to();
                    // let channel: u8 = result.get(3).to();
                    // let channel = Channels::from_godot(channel);

                    match event_type {
                        EventType::ERROR => {
                            server.destroy();
                            godot_error!("Server faced an error");
                            continue 'main;
                        }
                        EventType::CONNECT => {
                            peer = new_peer.unwrap();
                            break;
                        }
                        EventType::DISCONNECT => {
                            godot_error!(
                                "Somehow disconnected from a peer: {}",
                                new_peer.unwrap().get_remote_address()
                            );
                        }
                        EventType::RECEIVE => {
                            godot_error!("Somehow received message from a peer");
                        }
                        _ => {}
                    }
                }

                shared.base_mut_queue.push(Box::new(|mut base| {
                    base.emit_signal("connected".into(), &[]);
                }));
                shared.echo_controls.store(false, Ordering::Relaxed);
                shared.connected.store(true, Ordering::Relaxed);
                let mut last_receive_time = Instant::now();
                let mut pinged = false;
                let mut ffplay: Option<std::process::Child> = None;

                loop {
                    let result = server.service_ex().timeout(200).done();

                    if Arc::strong_count(&shared) == 1 {
                        peer.peer_disconnect();
                        if let Some(mut ffplay) = ffplay {
                            let _ = ffplay.kill();
                        }
                        loop {
                            let result = server.service();
                            let event_type: EventType = result.get(0).to();
                            let new_peer: Option<Gd<ENetPacketPeer>> = result.get(1).to();

                            match event_type {
                                EventType::ERROR => {
                                    godot_error!("Server faced an error disconnecting");
                                    break;
                                }
                                EventType::DISCONNECT => break,
                                EventType::CONNECT => {
                                    godot_error!(
                                        "Somehow connected to a peer: {}",
                                        new_peer.unwrap().get_remote_address()
                                    );
                                }
                                EventType::RECEIVE => {}
                                _ => unreachable!(),
                            }
                        }
                        server.destroy();
                        return;
                    }

                    let receive_elapsed = last_receive_time.elapsed();
                    if !pinged && receive_elapsed.as_secs() >= 3 {
                        pinged = true;
                        peer.send(
                            Channels::Important.into_godot() as i32 - 1,
                            std::iter::once(ImportantMessage::Ping.into_godot() - 1).collect(),
                            ENetPacketPeer::FLAG_RELIABLE,
                        );
                    }

                    let controls_data: Box<[u8]> = shared
                        .controls_data
                        .iter()
                        .map(|n| n.load(Ordering::Relaxed))
                        .collect();
                    let event_type: EventType = result.get(0).to();
                    let new_peer: Option<Gd<ENetPacketPeer>> = result.get(1).to();
                    // let _data: i32 = result.get(2).to();
                    let channel: u8 = result.get(3).to();
                    let channel = Channels::from_godot(channel + 1);

                    match event_type {
                        EventType::ERROR => {
                            server.destroy();
                            godot_error!("Server faced an error");
                            shared.connected.store(false, Ordering::Relaxed);
                            shared.base_mut_queue.push(Box::new(|mut base| {
                                base.emit_signal("disconnected".into(), &[]);
                            }));
                            continue 'main;
                        }
                        EventType::DISCONNECT => break,
                        EventType::CONNECT => {
                            godot_error!(
                                "Somehow connected to a peer: {}",
                                new_peer.unwrap().get_remote_address()
                            );
                        }
                        EventType::RECEIVE => {
                            pinged = false;
                            last_receive_time += receive_elapsed;
                            shared.base_mut_queue.push(Box::new(|mut base| {
                                base.emit_signal("something_received".into(), &[]);
                            }));
                            let data = peer.get_packet().to_vec();

                            match channel {
                                Channels::Important => {}
                                Channels::Camera => {
                                    // let data = data.to_vec();
                                    std::fs::write("camera.sdp", data)
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
                                Channels::Odometry => {
                                    let x =
                                        f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                                    let y =
                                        f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
                                    // if x == 0.0 || y == 0.0 {
                                    //     godot_error!("Invalid odometry origin");
                                    // }
                                    let origin = Vector2::new(x, y);
                                    shared.base_mut_queue.push(Box::new(move |mut base| {
                                        base.emit_signal(
                                            "odometry_received".into(),
                                            &[origin.to_variant()],
                                        );
                                    }));
                                }
                                Channels::Controls => {
                                    if data.len() != shared.controls_data.len() {
                                        godot_error!("Invalid controls packet");
                                        continue;
                                    }
                                    shared.echo_controls.store(
                                        data.as_slice() != controls_data.deref(),
                                        Ordering::Relaxed,
                                    );
                                }
                                Channels::Logs => {
                                    let Ok(log) = String::from_utf8(data) else {
                                        godot_error!("Failed to parse incoming log");
                                        continue;
                                    };
                                    godot_print!("{log}");
                                }
                                Channels::Max => {}
                            }
                        }
                        _ => {}
                    }

                    if shared.echo_controls.load(Ordering::Relaxed) {
                        peer.send(
                            Channels::Controls.into_godot() as i32 - 1,
                            controls_data.into_iter().copied().collect(),
                            ENetPacketPeer::FLAG_UNRELIABLE_FRAGMENT
                                | ENetPacketPeer::FLAG_UNSEQUENCED,
                        );
                    }

                    while let Some(packet) = shared.packet_queue.pop() {
                        peer.send(
                            packet.channel.into_godot() as i32 - 1,
                            packet.data.into_iter().collect(),
                            packet.flags,
                        );
                    }
                }

                if let Some(mut ffplay) = ffplay {
                    let _ = ffplay.kill();
                }
                shared.connected.store(false, Ordering::Relaxed);
                shared.base_mut_queue.push(Box::new(|mut base| {
                    base.emit_signal("disconnected".into(), &[]);
                }));
            }
        };
        self.thr = Some(std::thread::spawn(task));
    }

    fn exit_tree(&mut self) {
        self.shared = None;
        if let Some(thr) = self.thr.take() {
            let _ = thr.join();
        }
    }
}

#[godot_api]
impl LunabotConn {
    #[constant]
    const RECV_FROM: u16 = 43721;
    #[constant]
    const USE_ARCHIMEDES: bool = false;
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
    fn raw_send(&self, channel: Channels, data: PackedByteArray, flags: i32) {
        self.shared.as_ref().unwrap().packet_queue.push(Packet {
            channel,
            data: data.to_vec(),
            flags,
        });
    }

    #[func]
    fn raw_send_reliable(&self, channel: Channels, data: PackedByteArray) {
        self.raw_send(channel, data, ENetPacketPeer::FLAG_RELIABLE);
    }

    #[func]
    fn raw_send_unreliable(&self, channel: Channels, data: PackedByteArray) {
        self.raw_send(
            channel,
            data,
            ENetPacketPeer::FLAG_UNRELIABLE_FRAGMENT | ENetPacketPeer::FLAG_UNSEQUENCED,
        );
    }

    #[func]
    fn send_important_msg(&self, msg: ImportantMessage) {
        self.shared.as_ref().unwrap().packet_queue.push(Packet {
            channel: Channels::Important,
            data: vec![msg.into_godot()],
            flags: ENetPacketPeer::FLAG_RELIABLE,
        });
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

        let drive = (drive * 127.0).round() as i8;
        let steering = (steering * 127.0).round() as i8;

        let shared = self.shared.as_ref().unwrap();
        shared.controls_data[0].store(drive.to_le_bytes()[0], Ordering::Relaxed);
        shared.controls_data[1].store(steering.to_le_bytes()[0], Ordering::Relaxed);
        shared.echo_controls.store(true, Ordering::Relaxed);
    }

    #[func]
    fn send_arm_controls(&self, mut arm_vel: f32, mut drum_vel: f32) {
        if arm_vel > 1.0 {
            arm_vel = 1.0;
            godot_warn!("arm_vel greater than 1!")
        }
        if arm_vel < -1.0 {
            arm_vel = -1.0;
            godot_warn!("arm_vel lesser than -1!");
        }

        let arm_vel = (arm_vel * 127.0).round() as i8;

        let shared = self.shared.as_ref().unwrap();
        shared.controls_data[2].store(arm_vel.to_le_bytes()[0], Ordering::Relaxed);
        if Self::USE_ARCHIMEDES {
            if drum_vel > 1.0 {
                drum_vel = 1.0;
                godot_warn!("drum_vel greater than 1!")
            }
            if drum_vel < -1.0 {
                drum_vel = -1.0;
                godot_warn!("drum_vel lesser than -1!")
            }
            let drum_vel = (drum_vel * 127.0).round() as i8;
            shared.controls_data[3].store(drum_vel.to_le_bytes()[0], Ordering::Relaxed);
            shared.echo_controls.store(true, Ordering::Relaxed);
        }
    }
}
