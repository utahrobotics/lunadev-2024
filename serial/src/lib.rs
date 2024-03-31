//! This crate offers several ways to interface with serial ports under
//! the Unros framwork.

use std::{ops::Deref, sync::Arc, time::Duration};

pub use bytes::Bytes;
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros::{
    anyhow, async_trait,
    pubsub::{subs::DirectSubscription, Publisher, PublisherRef, Subscriber},
    setup_logging,
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt},
    },
    Node, NodeIntrinsics, RuntimeContext,
};

/// A single duplex connection to a serial port
pub struct SerialConnection<I: Send + Clone + 'static = Bytes, O: Send + Clone + 'static = Bytes> {
    path: Arc<str>,
    baud_rate: u32,

    output_map: Box<dyn FnMut(Bytes) -> Option<O> + Send>,
    input_map: Box<dyn FnMut(I) -> Bytes + Send>,

    serial_output: Publisher<O>,
    serial_input: Subscriber<I>,
    tolerate_error: bool,
    intrinsics: NodeIntrinsics<Self>,
}

impl SerialConnection {
    /// Creates a pending connection to a serial port.
    ///
    /// The connection is not actually made until this node is ran.
    /// If `tolerate_error` is `true`, then errors are ignored and
    /// actions are retried.
    pub fn new<'a>(path: impl Into<String>, baud_rate: u32, tolerate_error: bool) -> Self {
        Self {
            path: path.into().into_boxed_str().into(),
            baud_rate,
            output_map: Box::new(Some),
            input_map: Box::new(|x| x),
            serial_output: Publisher::default(),
            serial_input: Subscriber::new(8),
            tolerate_error,
            intrinsics: Default::default(),
        }
    }
}

impl<I: Send + Clone + 'static, O: Send + Clone + 'static> SerialConnection<I, O> {
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
    pub fn msg_received_pub(&self) -> PublisherRef<O> {
        self.serial_output.get_ref()
    }

    /// Provide a subscription whose messages will be written to the serial port.
    pub fn message_to_send_sub(&self) -> DirectSubscription<I> {
        self.serial_input.create_subscription()
    }

    pub fn map_input<NewI: Send + Clone + 'static>(
        mut self,
        input_map: impl FnMut(NewI) -> Bytes + Send + 'static,
    ) -> SerialConnection<NewI, O> {
        self.intrinsics.ignore_drop();
        SerialConnection {
            path: self.path,
            baud_rate: self.baud_rate,
            output_map: self.output_map,
            input_map: Box::new(input_map),
            serial_output: self.serial_output,
            serial_input: Subscriber::new(self.serial_input.get_size()),
            tolerate_error: self.tolerate_error,
            intrinsics: NodeIntrinsics::default(),
        }
    }

    pub fn map_output<NewO: Send + Clone + 'static>(
        mut self,
        output_map: impl FnMut(Bytes) -> Option<NewO> + Send + 'static,
    ) -> SerialConnection<I, NewO> {
        self.intrinsics.ignore_drop();
        SerialConnection {
            path: self.path,
            baud_rate: self.baud_rate,
            output_map: Box::new(output_map),
            input_map: self.input_map,
            serial_output: Publisher::default(),
            serial_input: self.serial_input,
            tolerate_error: self.tolerate_error,
            intrinsics: NodeIntrinsics::default(),
        }
    }
}

#[async_trait]
impl<I: Send + Clone + 'static, O: Send + Clone + 'static> Node for SerialConnection<I, O> {
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
                    msg = self.serial_input.recv() => {
                        let msg = (self.input_map)(msg);
                        let Err(e) = stream.write_all(&msg).await else { continue; };
                        break e;
                    }
                    result = stream.read(&mut buf) => {
                        let n = match result {
                            Ok(n) => n,
                            Err(e) => break e,
                        };
                        let bytes = buf.split_at(n).0.to_vec();
                        if let Some(msg) = (self.output_map)(bytes.into()) {
                            self.serial_output.set(msg);
                        }
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
    duty: Subscriber<i32>,
    intrinsics: NodeIntrinsics<Self>,
}

impl VescConnection {
    /// Wraps the given `SerialConnection` with the `VESC` protocol.
    ///
    /// Pre-existing subscriptions and the `Publisher` in the given `SerialConnection` will be dropped.
    pub fn new(serial: SerialConnection) -> Self {
        Self {
            serial,
            current: Subscriber::new(32),
            duty: Subscriber::new(32),
            intrinsics: Default::default(),
        }
    }

    /// Provide a subscription for the current level.
    pub fn get_current_sub(&self) -> DirectSubscription<u32> {
        self.current.create_subscription()
    }

    /// Provide a subscription for the duty cycle.
    pub fn get_duty_sub(&self) -> DirectSubscription<i32> {
        self.duty.create_subscription()
    }
}

#[async_trait]
impl Node for VescConnection {
    const DEFAULT_NAME: &'static str = "vesc-connection";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        self.serial.get_intrinsics().ignore_drop();
        let _ = std::mem::replace(&mut self.serial.serial_input, Subscriber::new(1));
        std::mem::take(&mut self.serial.serial_output);

        loop {
            let Some(stream) = self.serial.connect(&context).await? else {
                tokio::time::sleep(Duration::from_secs(2)).await;
                continue;
            };

            let mut stream = vesc_comm::VescConnection::new(stream);

            if let Err(e) = stream
                .set_duty(u32::from_ne_bytes(0i32.to_ne_bytes()))
                .await
            {
                if self.serial.tolerate_error {
                    error!(
                        "Encountered the following error while communicating with: {}: {e}",
                        self.serial.path
                    );
                } else {
                    break Err(e.into());
                }
            }

            let e = loop {
                tokio::select! {
                    duty = self.duty.recv() => {
                        let Err(e) = stream.set_duty(u32::from_ne_bytes(duty.to_ne_bytes())).await else { continue; };
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
