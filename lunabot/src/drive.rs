use navigator::drive::Steering;
use py_repl::PyRepl;
use serde::Deserialize;
use unros::{
    anyhow, async_trait, get_env, log, pubsub::{subs::DirectSubscription, Subscriber}, setup_logging, Node, NodeIntrinsics, RuntimeContext
};

pub struct Drive {
    steering_sub: Subscriber<Steering>,
    vesc: PyRepl,
    intrinsics: NodeIntrinsics<Self>,
    left_invert: bool,
    right_invert: bool,
}

#[derive(Deserialize)]
struct DriveConfig {
    left_port: String,
    right_port: String,

    #[serde(default)]
    left_invert: bool,
    #[serde(default)]
    right_invert: bool,
}

impl Drive {
    pub fn new() -> anyhow::Result<Self> {
        let mut vesc = PyRepl::new("lunabot/src")?;
        let config: DriveConfig = get_env()?;

        let mut msg = vesc.exec("from drive_vesc import *")?;
        if !msg.trim().is_empty() {
            log::error!("{msg}");
        }
        msg = vesc.exec(&format!(r#"vesc = Driver("{}", "{}")"#, config.left_port, config.right_port))?;
        if !msg.trim().is_empty() {
            log::error!("{msg}");
        }

        Ok(Self {
            steering_sub: Subscriber::new(4),
            intrinsics: Default::default(),
            vesc,
            left_invert: config.left_invert,
            right_invert: config.right_invert,
        })
    }

    pub fn get_steering_sub(&self) -> DirectSubscription<Steering> {
        self.steering_sub.create_subscription()
    }
}

#[async_trait]
impl Node for Drive {
    const DEFAULT_NAME: &'static str = "drive";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        // let mut left_duty = MonoPublisher::from(self.left_vesc.get_duty_sub());
        // let mut right_duty = MonoPublisher::from(self.right_vesc.get_duty_sub());

        let steering_fut = async {
            loop {
                let steering = self.steering_sub.recv().await;

                let left_modifier = if self.left_invert { -1.0 } else { 1.0 };
                let right_modifier = if self.right_invert { -1.0 } else { 1.0 };

                let msg = self.vesc.exec(&format!(r#"vesc.set_duty_cycle({}, {})"#, steering.left * left_modifier, steering.right * right_modifier))?;
                if !msg.trim().is_empty() {
                    error!("{msg}");
                }

                // left_duty.set((steering.left * 100 as f32 * left_modifier).round() as i32);
                // right_duty.set((steering.right * 100 as f32 * right_modifier).round() as i32);
            }
        };

        // self.left_vesc
        //     .get_intrinsics()
        //     .manually_run("left-vesc".into());
        // self.right_vesc
        //     .get_intrinsics()
        //     .manually_run("right-vesc".into());

        // tokio::select! {
        //     res = steering_fut => res,
        //     res = self.left_vesc.run(context.clone()) => res,
        //     res = self.right_vesc.run(context.clone()) => res
        // }

        steering_fut.await
    }
}
