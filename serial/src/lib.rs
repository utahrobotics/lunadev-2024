//! This crate offers several ways to interface with serial ports under
//! the Unros framwork.

use std::{ops::Deref, sync::Arc, time::Duration};

use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros_core::{
    anyhow, async_trait,
    bytes::Bytes,
    pubsub::{Publisher, Subscriber, Subscription},
    setup_logging,
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt},
    },
    Node, NodeIntrinsics, RuntimeContext,
};

/// A single duplex connection to a serial port
pub struct SerialConnection {
    path: Arc<str>,
    baud_rate: u32,
    msg_received: Publisher<Bytes>,
    messages_to_send: Subscriber<Bytes>,
    tolerate_error: bool,
    intrinsics: NodeIntrinsics<Self>,
}

impl SerialConnection {
    /// Creates a pending connection to a serial port.
    ///
    /// The connection is not actually made until this node is ran.
    /// If `tolerate_error` is `true`, then errors are ignored and
    /// actions are retried.
    pub async fn new<'a>(path: String, baud_rate: u32, tolerate_error: bool) -> Self {
        Self {
            path: path.into_boxed_str().into(),
            baud_rate,
            msg_received: Publisher::default(),
            messages_to_send: Subscriber::new(8),
            tolerate_error,
            intrinsics: Default::default(),
        }
    }

    async fn connect(&mut self, context: &RuntimeContext) -> anyhow::Result<Option<SerialStream>> {
        setup_logging!(context);

        let path = self.path.clone();
        let baud_rate = self.baud_rate;
        let result: tokio_serial::Result<_> = (|| {
            #[allow(unused_mut)]
            let mut stream = tokio_serial::new(path.deref(), baud_rate).open_native_async()?;
            #[cfg(unix)]
            stream.set_exclusive(true)?;
            stream.clear(tokio_serial::ClearBuffer::All)?;
            stream.set_break()?;
            // stream.set_flow_control(tokio_serial::FlowControl::Software)?;
            Ok(stream)
        })();

        let stream = if self.tolerate_error {
            match result {
                Ok(s) => Some(s),
                Err(e) => {
                    error!("Failed to connect to: {}: {e}", self.path);
                    None
                }
            }
        } else {
            Some(result?)
        };

        if stream.is_some() {
            info!("Succesfully connected to: {}", self.path);
        }
        Ok(stream)
    }

    /// Gets a reference to the `Signal` that represents received `Bytes`.
    pub fn accept_msg_received_sub(&mut self, sub: Subscription<Bytes>) {
        self.msg_received.accept_subscription(sub);
    }

    /// Provide a subscription whose messages will be written to the serial port.
    pub fn create_message_to_send_sub(&self) -> Subscription<Bytes> {
        self.messages_to_send.create_subscription()
    }
}

#[async_trait]
impl Node for SerialConnection {
    const DEFAULT_NAME: &'static str = "serial_connection";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        loop {
            let Some(mut stream) = self.connect(&context).await? else {
                tokio::time::sleep(Duration::from_secs(2)).await;
                continue;
            };

            let mut buf = [0; 1024];
            let e = loop {
                tokio::select! {
                    msg = self.messages_to_send.recv() => {
                        let Err(e) = stream.write_all(&msg).await else { continue; };
                        break e;
                    }
                    result = stream.read(&mut buf) => {
                        let n = match result {
                            Ok(n) => n,
                            Err(e) => break e,
                        };
                        let bytes = buf.split_at(n).0.to_vec();
                        self.msg_received.set(bytes.into());
                    }
                }
            };

            if self.tolerate_error {
                error!(
                    "Encountered the following error while communicating with: {}: {e}",
                    self.path
                );
            } else {
                break Err(e.into());
            }
        }
    }
}

/// A single `VESC` connection to a serial port.
pub struct VescConnection {
    serial: SerialConnection,
    current: Subscriber<u32>,
    duty: Subscriber<u32>,
    intrinsics: NodeIntrinsics<Self>,
}

impl VescConnection {
    /// Wraps the given `SerialConnection` with the `VESC` protocol.
    ///
    /// Pre-existing subscriptions in the given `SerialConnection` will be ignored (but not dropped).
    pub fn new(serial: SerialConnection) -> Self {
        Self {
            serial,
            current: Subscriber::new(32),
            duty: Subscriber::new(32),
            intrinsics: Default::default(),
        }
    }

    /// Provide a subscription for the current level.
    pub fn connect_current_from(&self) -> Subscription<u32> {
        self.current.create_subscription()
    }

    /// Provide a subscription for the duty cycle.
    pub fn connect_duty_from(&self) -> Subscription<u32> {
        self.current.create_subscription()
    }
}

#[async_trait]
impl Node for VescConnection {
    const DEFAULT_NAME: &'static str = "vesc_connection";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        loop {
            let Some(stream) = self.serial.connect(&context).await? else {
                tokio::time::sleep(Duration::from_secs(2)).await;
                continue;
            };

            let mut stream = vesc_comm::VescConnection::new(stream);

            let e = loop {
                tokio::select! {
                    duty = self.duty.recv() => {
                        let Err(e) = stream.set_duty(duty).await else { continue; };
                        break e;
                    }
                    current = self.current.recv() => {
                        let Err(e) = stream.set_current(current).await else { continue; };
                        break e;
                    }
                }
            };

            if self.serial.tolerate_error {
                error!(
                    "Encountered the following error while communicating with: {}: {e}",
                    self.serial.path
                );
            } else {
                break Err(e.into());
            }
        }
    }
}
