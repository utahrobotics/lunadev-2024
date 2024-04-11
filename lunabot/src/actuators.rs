use lunabot::ArmParameters;
use serial::SerialConnection;
use unros::{
    anyhow, async_trait,
    pubsub::{subs::DirectSubscription, MonoPublisher, Subscriber},
    setup_logging, tokio, Node, NodeIntrinsics, RuntimeContext,
};

pub struct Arms {
    arm_sub: Subscriber<ArmParameters>,
    tilt_conn: SerialConnection<String, String>,
    lift_conn: SerialConnection<String, String>,
    intrinsics: NodeIntrinsics<Self>,
}

impl Arms {
    pub fn new(tilt_port: impl Into<String>, lift_port: impl Into<String>) -> Self {
        Self {
            arm_sub: Subscriber::new(4),
            intrinsics: Default::default(),
            tilt_conn: SerialConnection::new(tilt_port, 115200, true).map_to_string(),
            lift_conn: SerialConnection::new(lift_port, 115200, true).map_to_string(),
        }
    }

    pub fn get_arm_sub(&self) -> DirectSubscription<ArmParameters> {
        self.arm_sub.create_subscription()
    }
}

#[async_trait]
impl Node for Arms {
    const DEFAULT_NAME: &'static str = "arms";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let mut tilt_repl = MonoPublisher::from(self.tilt_conn.message_to_send_sub());
        let tilt_repl_sub = Subscriber::new(1);
        self.tilt_conn
            .msg_received_pub()
            .accept_subscription(tilt_repl_sub.create_subscription());
        let mut lift_repl = MonoPublisher::from(self.lift_conn.message_to_send_sub());

        let arms_fut = async {
            tilt_repl.set("info()\r".into());
            loop {
                let params = self.arm_sub.recv().await;
                let info = tilt_repl_sub.recv().await;

                if info.starts_with("INFO:") {
                    if info.contains("INFO:\nRole: Tilt\n") {
                        // Do Nothing, we are correct already
                    } else if info.contains("INFO:\nRole: Lift\n") {
                        std::mem::swap(&mut tilt_repl, &mut lift_repl);
                        info!("Arms were swapped");
                    } else {
                        break Err(anyhow::anyhow!("Unexpected response from arm: {}", info));
                    }
                } else {
                    break Err(anyhow::anyhow!("Unexpected response from arm: {}", info));
                }

                match params {
                    ArmParameters::TiltUp => tilt_repl.set("r()\r".into()),
                    ArmParameters::TiltDown => tilt_repl.set("e()\r".into()),
                    ArmParameters::Stop => {
                        tilt_repl.set("s()\r".into());
                        lift_repl.set("s()\r".into());
                    }
                    ArmParameters::LiftArm => lift_repl.set("e()\r".into()),
                    ArmParameters::LowerArm => lift_repl.set("r()\r".into()),
                    ArmParameters::SetArm { .. } => todo!(),
                }
            }
        };

        self.tilt_conn
            .get_intrinsics()
            .manually_run("tilt-bucket".into());
        self.lift_conn
            .get_intrinsics()
            .manually_run("lift-arm".into());

        tokio::select! {
            res = arms_fut => res,
            res = self.tilt_conn.run(context.clone()) => res,
            res = self.lift_conn.run(context.clone()) => res
        }
    }
}
