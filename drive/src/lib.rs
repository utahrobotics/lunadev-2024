use global_msgs::Steering;
use serial::SerialConnection;
use unros_core::{
    anyhow, async_trait, setup_logging,
    pubsub::{Publisher, Subscriber, Subscription},
    tokio, Node, RuntimeContext,
};

pub struct Drive {
    drive_controller: SerialConnection,
    steering_sub: Subscriber<Steering>,
}

impl Drive {
    pub fn new(serial: SerialConnection) -> Self {
        Self {
            drive_controller: serial,
            steering_sub: Subscriber::default(),
        }
    }

    pub fn create_steering_sub(&mut self) -> Subscription<Steering> {
        self.steering_sub.create_subscription(8)
    }
}

#[async_trait]
impl Node for Drive {
    const DEFAULT_NAME: &'static str = "drive";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        let context2 = context.clone();
        setup_logging!(context);

        let mut write_signal = Publisher::default();
        write_signal.accept_subscription(self.drive_controller.create_message_to_send_sub());
        let mut read_sub = Subscriber::default();
        self.drive_controller
            .accept_msg_received_sub(read_sub.create_subscription(8));

        let handle = tokio::spawn(async move {
            'main: loop {
                let Some(steering) = self.steering_sub.recv_or_closed().await else {
                    break Ok(());
                };
                write_signal.set(
                    format!("setDrive({:.2},{:.2})", steering.left, steering.right)
                        .into_bytes()
                        .into(),
                );
                let mut output = String::new();
                loop {
                    let msg = read_sub.recv().await;
                    output += match std::str::from_utf8(&msg) {
                        Ok(x) => x,
                        Err(e) => break 'main Err(anyhow::Error::from(e)),
                    };

                    if output.ends_with(">>> ") {
                        if output.contains("Exception") {
                            error!("{output}");
                        }
                        break;
                    }
                }
            }
        });

        tokio::select! {
            result = self.drive_controller.run(context2) => { result }
            result = handle => { result.unwrap() }
        }
    }
}
