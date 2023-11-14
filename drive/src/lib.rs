use global_msgs::Steering;
use serial::SerialConnection;
use unros_core::{Node, async_trait, RuntimeContext, anyhow, signal::{unbounded::UnboundedSubscription, Signal}, tokio, setup_logging};

pub struct Drive {
    drive_controller: SerialConnection,
    steering_sub: UnboundedSubscription<Steering>
}


impl Drive {
    pub fn new(serial: SerialConnection, steering_sub: UnboundedSubscription<Steering>) -> Self {
        Self {
            drive_controller: serial,
            steering_sub
        }
    }
}


#[async_trait]
impl Node for Drive {
    const DEFAULT_NAME: &'static str = "drive";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        let context2 = context.clone();
        setup_logging!(context);

        let mut steering_sub = self.steering_sub.to_watched().await;
        let mut write_signal = Signal::default();
        self.drive_controller.message_to_send_subscription(write_signal.get_ref().subscribe_bounded());
        let mut read_sub = self.drive_controller.get_msg_received_signal().subscribe_unbounded();

        let handle = tokio::spawn(async move {
            'main: loop {
                let Some(steering) = steering_sub.changed_or_closed().await else { break Ok(()); };
                let (left, right) = steering.to_left_right_drive();
                write_signal.set(format!("setDrive({:.2},{:.2})", left, right).into_bytes().into());
                let mut output = String::new();
                loop {
                    let msg = read_sub.recv().await;
                    output += match std::str::from_utf8(&msg)  {
                        Ok(x) => x,
                        Err(e) => break 'main Err(anyhow::Error::from(e))
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

