//! This crate offers several ways to interface with serial ports under
//! the Unros framwork.

use std::{ops::Deref, sync::Arc, time::Duration};

pub use bytes::Bytes;
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros::{
    anyhow, node::AsyncNode, pubsub::{subs::DirectSubscription, Publisher, PublisherRef, Subscriber}, runtime::RuntimeContext, setup_logging, tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt},
    }, DontDrop
};

/// A single duplex connection to a serial port
pub struct SerialConnection<I=Bytes, O=Bytes> {
    path: Arc<str>,
    baud_rate: u32,

    output_map: Box<dyn FnMut(Bytes) -> Option<O> + Send>,
    input_map: Box<dyn FnMut(I) -> Bytes + Send>,

    serial_output: Publisher<O>,
    serial_input: Subscriber<I>,
    tolerate_error: bool,
    dont_drop: DontDrop,
}

impl SerialConnection {
    /// Creates a pending connection to a serial port.
    ///
    /// The connection is not actually made until this node is ran.
    /// If `tolerate_error` is `true`, then errors are ignored and
    /// actions are retried.
    pub fn new<'a>(path: impl Into<String>, baud_rate: u32, tolerate_error: bool) -> Self {
        let path: Arc<str> = path.into().into_boxed_str().into();
        Self {
            baud_rate,
            output_map: Box::new(Some),
            input_map: Box::new(|x| x),
            serial_output: Publisher::default(),
            serial_input: Subscriber::new(8),
            tolerate_error,
            dont_drop: DontDrop::new(path.to_string()),
            path,
        }
    }

    pub fn map_to_string(self) -> SerialConnection<String, String> {
        let mut output_buf = vec![];

        self.map_input(|msg: String| msg.into_bytes().into())
            .map_output(move |bytes: Bytes| {
                output_buf.extend_from_slice(&bytes);
                if let Ok(msg) = std::str::from_utf8(&output_buf) {
                    Some(msg.into())
                } else {
                    output_buf.clear();
                    None
                }
            })
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
            // stream.set_break()?;

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
        self,
        input_map: impl FnMut(NewI) -> Bytes + Send + 'static,
    ) -> SerialConnection<NewI, O> {
        SerialConnection {
            path: self.path,
            baud_rate: self.baud_rate,
            output_map: self.output_map,
            input_map: Box::new(input_map),
            serial_output: self.serial_output,
            serial_input: Subscriber::new(self.serial_input.get_size()),
            tolerate_error: self.tolerate_error,
            dont_drop: self.dont_drop,
        }
    }

    pub fn map_output<NewO: Send + Clone + 'static>(
        self,
        output_map: impl FnMut(Bytes) -> Option<NewO> + Send + 'static,
    ) -> SerialConnection<I, NewO> {
        SerialConnection {
            path: self.path,
            baud_rate: self.baud_rate,
            output_map: Box::new(output_map),
            input_map: self.input_map,
            serial_output: Publisher::default(),
            serial_input: self.serial_input,
            tolerate_error: self.tolerate_error,
            dont_drop: self.dont_drop,
        }
    }
}

impl<I: Send + Clone + 'static, O: Send + Clone + 'static> AsyncNode for SerialConnection<I, O> {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;

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
