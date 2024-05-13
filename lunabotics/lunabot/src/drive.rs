use base64::prelude::*;
use lunabot_lib::Steering;
use py_repl::PyRepl;
use serde::Deserialize;
use serial::{Bytes, SerialConnection};
use unros::{
    anyhow, get_env, log,
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, MonoPublisher, Publisher, PublisherRef, Subscriber},
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
    current_pub: Publisher<(f32, f32)>,
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
        let mut vesc = PyRepl::new(".")?;
        let config: DriveConfig = get_env()?;

        let msg = vesc.exec(include_str!("drive_vesc.py"))?;
        // let msg = vesc.exec("from drive_vesc import *")?;
        if !msg.trim().is_empty() {
            log::error!("{msg}");
        }

        let get_values_respose_len: usize =
            vesc.exec("print(GET_VALUES_MSG_LENGTH)")?.trim().parse()?;
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
            current_pub: Publisher::default(),
        })
    }

    pub fn get_steering_sub(&self) -> DirectSubscription<Steering> {
        self.steering_sub.create_subscription()
    }

    pub fn get_current_pub(&self) -> PublisherRef<(f32, f32)> {
        self.current_pub.get_ref()
    }
}

impl AsyncNode for Drive {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;

        let mut left_pub = MonoPublisher::from(self.left_conn.message_to_send_sub());
        let mut left_sub = Subscriber::new(8);
        self.left_conn
            .msg_received_pub()
            .accept_subscription(left_sub.create_subscription());
        let mut right_pub = MonoPublisher::from(self.right_conn.message_to_send_sub());
        let mut right_sub = Subscriber::new(8);
        self.right_conn
            .msg_received_pub()
            .accept_subscription(right_sub.create_subscription());

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
                r#"decode_app_controller_id("{}")"#,
                BASE64_STANDARD.encode(&get_values_buf)
            ))?;

            if app_controller_id == "4" {
                std::mem::swap(&mut left_pub, &mut right_pub);
                std::mem::swap(&mut left_sub, &mut right_sub);
            }

            loop {
                for _ in 0..10 {
                    let steering;

                    tokio::select! {
                        tmp = self.steering_sub.recv() => {
                            steering = tmp;
                        }
                        _ = tokio::time::sleep(std::time::Duration::from_millis(100)) => {
                            continue;
                        }
                    }

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
                    match BASE64_STANDARD.decode(left_b64.trim()) {
                        Ok(x) => left_pub.set(x.into()),
                        Err(e) => {
                            error!("Failed to decode left vesc message: {}: {e}", left_b64.trim());
                        }
                    }
                    match BASE64_STANDARD.decode(right_b64.trim()) {
                        Ok(x) => right_pub.set(x.into()),
                        Err(e) => {
                            error!("Failed to decode right vesc message: {}: {e}", right_b64.trim());
                        }
                    }
                }

                // left_pub.set(self.get_values_request.clone());
                // get_values_buf.clear();
                // read_bytes = 0;

                // while read_bytes < self.get_values_respose_len {
                //     let bytes = left_sub.recv().await;
                //     let bytes_len = bytes.len();
                //     get_values_buf.extend_from_slice(&bytes);
                //     read_bytes += bytes_len;
                // }

                // let left_current = self.vesc.exec(&format!(
                //     r#"decode_avg_motor_current("{}")"#,
                //     BASE64_STANDARD.encode(&get_values_buf)
                // ))?;
                // let left_current: f32 = left_current.parse()?;

                // right_pub.set(self.get_values_request.clone());
                // get_values_buf.clear();
                // read_bytes = 0;

                // while read_bytes < self.get_values_respose_len {
                //     let bytes = right_sub.recv().await;
                //     let bytes_len = bytes.len();
                //     get_values_buf.extend_from_slice(&bytes);
                //     read_bytes += bytes_len;
                // }

                // let right_current = self.vesc.exec(&format!(
                //     r#"decode_avg_motor_current("{}")"#,
                //     BASE64_STANDARD.encode(&get_values_buf)
                // ))?;
                // let right_current: f32 = right_current.parse()?;

                // self.current_pub.set((left_current, right_current));
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
