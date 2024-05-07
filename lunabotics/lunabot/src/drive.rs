use base64::prelude::*;
use lunabot_lib::Steering;
use py_repl::PyRepl;
use serde::Deserialize;
use serial::{Bytes, SerialConnection};
use unros::{
    anyhow, get_env, log,
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, MonoPublisher, Subscriber},
    runtime::RuntimeContext,
    setup_logging, tokio, DontDrop, ShouldNotDrop,
};

#[derive(ShouldNotDrop)]
pub struct Drive {
    steering_sub: Subscriber<Steering>,
    vesc: PyRepl,
    left_conn: SerialConnection,
    right_conn: SerialConnection,
    dont_drop: DontDrop<Self>,
    left_invert: bool,
    right_invert: bool,
    get_values_respose_len: usize,
    get_values_request: Bytes,
}

#[derive(Deserialize)]
struct DriveConfig {
    #[serde(default)]
    left_invert: bool,
    #[serde(default)]
    right_invert: bool,
}

impl Drive {
    pub fn new(
        left_port: impl Into<String>,
        right_port: impl Into<String>,
    ) -> anyhow::Result<Self> {
        let mut vesc = PyRepl::new("lunabotics/lunabot/src")?;
        let config: DriveConfig = get_env()?;

        // let msg = vesc.exec(include_str!("drive_vesc.py"))?;
        let msg = vesc.exec("from drive_vesc import *")?;
        if !msg.trim().is_empty() {
            log::error!("{msg}");
        }
        let get_values_respose_len: usize = vesc.exec("print(GET_VALUES_MSG_LENGTH)")?.trim().parse()?;
        let get_values_request = BASE64_STANDARD
            .decode(vesc.exec("get_GET_VALUES()")?)?
            .into();

        Ok(Self {
            steering_sub: Subscriber::new(4),
            dont_drop: DontDrop::new("drive"),
            vesc,
            left_invert: config.left_invert,
            right_invert: config.right_invert,
            left_conn: SerialConnection::new(left_port, 115200, true),
            right_conn: SerialConnection::new(right_port, 115200, true),
            get_values_respose_len,
            get_values_request,
        })
    }

    pub fn get_steering_sub(&self) -> DirectSubscription<Steering> {
        self.steering_sub.create_subscription()
    }
}

impl AsyncNode for Drive {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;

        let mut left_pub = MonoPublisher::from(self.left_conn.message_to_send_sub());
        let left_sub = Subscriber::new(8);
        self.left_conn
            .msg_received_pub()
            .accept_subscription(left_sub.create_subscription());
        let mut right_pub = MonoPublisher::from(self.right_conn.message_to_send_sub());

        let steering_fut = async {
            left_pub.set(self.get_values_request.clone());
            let mut get_values_buf = Vec::with_capacity(self.get_values_respose_len);
            let mut read_bytes = 0usize;

            while read_bytes < self.get_values_respose_len {
                let bytes = left_sub.recv().await;
                let bytes_len = bytes.len();
                get_values_buf.extend_from_slice(&bytes);
                read_bytes += bytes_len;
            }

            let app_controller_id = self.vesc.exec(&format!(
                r#"decode_get_values("{}")"#,
                BASE64_STANDARD.encode(&get_values_buf)
            ))?;

            if app_controller_id == "4" {
                std::mem::swap(&mut left_pub, &mut right_pub);
            }

            loop {
                let steering = self.steering_sub.recv().await;

                let left_modifier = if self.left_invert { -1.0 } else { 1.0 };
                let right_modifier = if self.right_invert { -1.0 } else { 1.0 };

                let left_b64 = self.vesc.exec(&format!(
                    r#"encode_duty_cycle({})"#,
                    steering.left * left_modifier
                ))?;
                let right_b64 = self.vesc.exec(&format!(
                    r#"encode_duty_cycle({})"#,
                    steering.right * right_modifier
                ))?;
                let left_vesc_msg = match BASE64_STANDARD.decode(left_b64.trim()) {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to decode left vesc message: {e}");
                        continue;
                    }
                };
                let right_vesc_msg = match BASE64_STANDARD.decode(right_b64.trim()) {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to decode right vesc message: {e}");
                        continue;
                    }
                };

                left_pub.set(left_vesc_msg.into());
                right_pub.set(right_vesc_msg.into());
            }
        };

        // self.left_conn
        //     .get_intrinsics()
        //     .manually_run("left-vesc".into());
        // self.right_conn
        //     .get_intrinsics()
        //     .manually_run("right-vesc".into());

        tokio::select! {
            res = steering_fut => res,
            res = self.left_conn.run(context.clone()) => res,
            res = self.right_conn.run(context.clone()) => res
        }
    }
}
