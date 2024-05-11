use lunabot_lib::{ArmAction, ArmParameters};
use serial::SerialConnection;
use unros::{
    anyhow,
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, MonoPublisher, Publisher, Subscriber},
    runtime::RuntimeContext,
    setup_logging, tokio, DontDrop, ShouldNotDrop,
};

#[derive(ShouldNotDrop)]
pub struct Arms {
    arm_sub: Subscriber<ArmParameters>,
    tilt_conn: SerialConnection<String, String>,
    lift_conn: SerialConnection<String, String>,
    tilt_value: Publisher<f32>,
    lift_value: Publisher<f32>,
    dont_drop: DontDrop<Self>,
}

impl Arms {
    pub fn new(tilt_port: impl Into<String>, lift_port: impl Into<String>) -> Self {
        Self {
            arm_sub: Subscriber::new(4),
            dont_drop: DontDrop::new("arms"),
            tilt_conn: SerialConnection::new(tilt_port, 115200, true).map_to_string(),
            lift_conn: SerialConnection::new(lift_port, 115200, true).map_to_string(),
            tilt_value: Publisher::default(),
            lift_value: Publisher::default()
        }
    }

    pub fn get_arm_sub(&self) -> DirectSubscription<ArmParameters> {
        self.arm_sub.create_subscription()
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

            // lift_repl.set("extend_home()\r".into());
            // tilt_repl.set("extend_home()\r".into());
            // tokio::time::sleep(std::time::Duration::from_secs(21000)).await;

            tokio::spawn(async move {
                let mut lift_buf = String::new();
                let mut sum = 0isize;
                let mut count = 0usize;
                loop {
                    lift_buf += &lift_sub.recv().await;
                    if let Some(index) = lift_buf.find('\n') {
                        let line = lift_buf.split_at(index).0;
                        if !line.starts_with("Motor") {
                            lift_buf.drain(0..index + 1);
                            continue;
                        }

                        let value: isize = line.split_at(9).1.trim().parse().unwrap();
                        sum += value;
                        count += 1;
                        lift_buf.drain(0..index + 1);
                        if count == 2 {
                            self.lift_value.set(sum as f32 / 9000.0);
                            sum = 0;
                            count = 0;
                        }

                    }
                }
            });

            tokio::spawn(async move {
                let mut tilt_buf = String::new();
                let mut sum = 0isize;
                let mut count = 0usize;
                loop {
                    tilt_buf += &tilt_sub.recv().await;
                    if let Some(index) = tilt_buf.find('\n') {
                        let line = tilt_buf.split_at(index).0;
                        if !line.starts_with("Motor") {
                            tilt_buf.drain(0..index + 1);
                            continue;
                        }
                        
                        let value: isize = line.split_at(9).1.trim().parse().unwrap();
                        sum += value;
                        count += 1;
                        tilt_buf.drain(0..index + 1);
                        if count == 2 {
                            self.tilt_value.set(sum as f32 / 9000.0);
                            sum = 0;
                            count = 0;
                        }
                    }
                }
            });

            loop {
                let params = self.arm_sub.recv().await;

                match params.lift {
                    ArmAction::Extend => lift_repl.set("e()\r".into()),
                    ArmAction::Retract => lift_repl.set("r()\r".into()),
                    ArmAction::Stop => lift_repl.set("s()\r".into()),
                    // ArmAction::Home => lift_repl.set("extend_home()\r".into()),
                    ArmAction::SetValue(_) => todo!(),
                }

                match params.tilt {
                    ArmAction::Extend => tilt_repl.set("e()\r".into()),
                    ArmAction::Retract => tilt_repl.set("r()\r".into()),
                    ArmAction::Stop => tilt_repl.set("s()\r".into()),
                    // ArmAction::Home => tilt_repl.set("extend_home()\r".into()),
                    ArmAction::SetValue(_) => todo!(),
                }
                lift_repl.set("print_pos()\r".into());
                tilt_repl.set("print_pos()\r".into());
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
