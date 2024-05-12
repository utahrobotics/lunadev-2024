use lunabot_lib::{ArmAction, ArmParameters, Odometry};
use nalgebra::Vector3;
use serde::Deserialize;
use serial::SerialConnection;
use unros::{
    anyhow,
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, MonoPublisher, Publisher, PublisherRef, Subscriber},
    runtime::RuntimeContext,
    setup_logging, tokio, DontDrop, ShouldNotDrop,
};

#[derive(Deserialize, Default)]
struct ArmsConfig {
    #[serde(default)]
    home: bool
}

#[derive(ShouldNotDrop)]
pub struct Arms {
    arm_sub: Subscriber<ArmParameters>,
    tilt_conn: SerialConnection<String, String>,
    lift_conn: SerialConnection<String, String>,
    odometry_pub: Publisher<Odometry>,
    dont_drop: DontDrop<Self>,
    config: ArmsConfig
}

impl Arms {
    pub fn new(tilt_port: impl Into<String>, lift_port: impl Into<String>) -> Self {
        Self {
            arm_sub: Subscriber::new(4),
            dont_drop: DontDrop::new("arms"),
            tilt_conn: SerialConnection::new(tilt_port, 115200, true).map_to_string(),
            lift_conn: SerialConnection::new(lift_port, 115200, true).map_to_string(),
            odometry_pub: Publisher::default(),
            config: unros::get_env().map_err(|e| {
                unros::log::error!("{e}");
            }).unwrap_or_default()
        }
    }

    pub fn get_arm_sub(&self) -> DirectSubscription<ArmParameters> {
        self.arm_sub.create_subscription()
    }

    pub fn get_odometry_pub(&self) -> PublisherRef<Odometry> {
        self.odometry_pub.get_ref()
    }
}

impl AsyncNode for Arms {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        self.dont_drop.ignore_drop = true;
        let tilt_repl = MonoPublisher::from(self.tilt_conn.message_to_send_sub());
        let tilt_repl_sub = Subscriber::new(8);
        self.tilt_conn
            .msg_received_pub()
            .accept_subscription(tilt_repl_sub.create_subscription());
        let lift_repl = MonoPublisher::from(self.lift_conn.message_to_send_sub());

        struct DropRepl(MonoPublisher<String, DirectSubscription<String>>);

        impl Drop for DropRepl {
            fn drop(&mut self) {
                self.0.set("store_pos()\r".into());
            }
        }

        let mut tilt_repl = DropRepl(tilt_repl);
        let mut tilt_repl = &mut tilt_repl.0;

        let mut lift_repl = DropRepl(lift_repl);
        let mut lift_repl = &mut lift_repl.0;

        let lift_sub = Subscriber::new(32);
        let tilt_sub = Subscriber::new(32);
        self.lift_conn.msg_received_pub().accept_subscription(lift_sub.create_subscription());
        self.tilt_conn.msg_received_pub().accept_subscription(tilt_sub.create_subscription());

        let arms_fut = async {
            tilt_repl.set("get_info()\r".into());
            let mut info = String::new();

            loop {
                let tmp = tilt_repl_sub.recv().await;
                info.push_str(&tmp);

                if info.len() < 15 {
                    continue;
                }

                let info = info.split_at(10).1.trim();

                if info.starts_with("tilt") {
                    break;
                } else if info.starts_with("lift") {
                    std::mem::swap(&mut tilt_repl, &mut lift_repl);
                    break;
                } else {
                    return Err(anyhow::anyhow!("Unexpected response from arm: {}", info));
                }
            }

            if self.config.home {
                lift_repl.set("extend_home()\r".into());
                tilt_repl.set("extend_home()\r".into());
                tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            }

            tokio::spawn(async move {
                let mut lift_buf = String::new();
                let mut lift_value = 0f32;

                let mut tilt_buf = String::new();
                let mut tilt_value = 0f32;

                let mut lift_accel = Vector3::default();
                let mut tilt_accel = Vector3::default();

                loop {
                    tokio::select! {
                        lift_msg = lift_sub.recv() => {
                            lift_buf += &lift_msg;
                            if let Some(index) = lift_buf.find('\n') {
                                let line = lift_buf.split_at(index).0;
                                let mut split = line.split(' ');

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                let mut result: Result<isize, _> = next.parse();
                                let Ok(value1) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                result = next.parse();
                                let Ok(value2) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                let mut result: Result<f32, _> = next.parse();
                                let Ok(x) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                result = next.parse();
                                let Ok(y) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                result = next.parse();
                                let Ok(z) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                lift_accel = Vector3::new(x, y, z);
                                lift_value = (value1 + value2) as f32 / 2.0;
                            }
                        }
                        tilt_msg = tilt_sub.recv() => {
                            tilt_buf += &tilt_msg;
                            if let Some(index) = tilt_buf.find('\n') {
                                let line = tilt_buf.split_at(index).0;
                                let mut split = line.split(' ');

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from tilt: {}", line);
                                    tilt_buf.drain(0..index + 1);
                                    continue;
                                };
                                let mut result: Result<isize, _> = next.parse();
                                let Ok(value1) = result else {
                                    unros::log::error!("Unexpected response from tilt: {}", line);
                                    tilt_buf.drain(0..index + 1);
                                    continue;
                                };
                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from tilt: {}", line);
                                    tilt_buf.drain(0..index + 1);
                                    continue;
                                };
                                result = next.parse();
                                let Ok(value2) = result else {
                                    unros::log::error!("Unexpected response from tilt: {}", line);
                                    tilt_buf.drain(0..index + 1);
                                    continue;
                                };

                                let mut result: Result<f32, _> = next.parse();
                                let Ok(x) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                result = next.parse();
                                let Ok(y) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                let Some(next) = split.next() else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };
                                result = next.parse();
                                let Ok(z) = result else {
                                    unros::log::error!("Unexpected response from lift: {}", line);
                                    lift_buf.drain(0..index + 1);
                                    continue;
                                };

                                tilt_accel = Vector3::new(x, y, z);
                                tilt_value = (value1 + value2) as f32 / 2.0;
                            }
                        }
                    }

                    let lift_l = 37.0 + 25.5 / 4500.0 * lift_value;
                    let tilt_l = 37.0 + 25.5 / 4500.0 * tilt_value;

                    let arm_angle = -0.1354 - ((lift_l.powi(2) - 3312.5) / 3148.5).acos() + ((tilt_l.powi(2) - 2530.9) / 1785.6).acos();
                    let accel = (lift_accel + tilt_accel) / 2.0;
                    self.odometry_pub.set(Odometry {
                        arm_angle,
                        acceleration: accel.into()
                    });

                    let front_elevation = 159.031 + 52.4931 * (0.0001206 * tilt_l.powi(2) + 2.53947 * tilt_l - 24.8994).cos() + 643.036 * (0.00005467 * lift_l.powi(2) + 2.51201 * lift_l - 8.12657);
                    let back_elevation = 29.6433 + 66.0333 * (0.000345 * tilt_l.powi(2) + 2.50845 * tilt_l - 24.3035).cos() + 7.28242 * (0.003568 * lift_l.powi(2) + 2.24195 * lift_l - 3.48055);

                    println!("{}cm {}cm", front_elevation, back_elevation);
                }
            });

            loop {
                let params = self.arm_sub.recv().await;

                match params.lift {
                    ArmAction::Extend => lift_repl.set("e()\r".into()),
                    ArmAction::Retract => lift_repl.set("r()\r".into()),
                    ArmAction::Stop => lift_repl.set("s()\r".into()),
                    ArmAction::Home => lift_repl.set("extend_home()\r".into()),
                    ArmAction::SetValue(_) => todo!(),
                }

                match params.tilt {
                    ArmAction::Extend => tilt_repl.set("e()\r".into()),
                    ArmAction::Retract => tilt_repl.set("r()\r".into()),
                    ArmAction::Stop => tilt_repl.set("s()\r".into()),
                    ArmAction::Home => tilt_repl.set("extend_home()\r".into()),
                    ArmAction::SetValue(_) => todo!(),
                }
            }
        };

        // self.tilt_conn
        //     .get_intrinsics()
        //     .manually_run("tilt-bucket".into());
        // self.lift_conn
        //     .get_intrinsics()
        //     .manually_run("lift-arm".into());

        tokio::select! {
            res = arms_fut => res,
            res = self.tilt_conn.run(context.clone()) => res,
            res = self.lift_conn.run(context.clone()) => res
        }
    }
}
