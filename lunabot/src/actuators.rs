use lunabot::ArmParameters;
use serde::Deserialize;
use serial::SerialConnection;
use unros::{
    anyhow, async_trait, get_env,
    pubsub::{subs::DirectSubscription, MonoPublisher, Subscriber},
    setup_logging, Node, NodeIntrinsics, RuntimeContext,
};

pub struct Arms {
    arm_sub: Subscriber<ArmParameters>,
    tilt_conn: SerialConnection<String, String>,
    lift_conn: SerialConnection<String, String>,
    intrinsics: NodeIntrinsics<Self>,
}

#[derive(Deserialize)]
struct ArmConfig {
    tilt_port: String,
    lift_port: String,
}

impl Arms {
    pub fn new() -> anyhow::Result<Self> {
        let config: ArmConfig = get_env()?;
        Ok(Self {
            arm_sub: Subscriber::new(4),
            intrinsics: Default::default(),
            tilt_conn: SerialConnection::new(config.tilt_port, 115200, true).map_to_string(),
            lift_conn: SerialConnection::new(config.lift_port, 115200, true).map_to_string(),
        })
    }

    pub fn new_with(
        tilt_conn: SerialConnection<String, String>,
        lift_conn: SerialConnection<String, String>,
    ) -> Self {
        Self {
            arm_sub: Subscriber::new(4),
            intrinsics: Default::default(),
            tilt_conn,
            lift_conn,
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
        let mut lift_repl = MonoPublisher::from(self.lift_conn.message_to_send_sub());
        context.spawn_node(self.tilt_conn);
        context.spawn_node(self.lift_conn);

        loop {
            let params = self.arm_sub.recv().await;

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
    }
}
