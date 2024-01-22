//! This crate offers several ways to interface with serial ports under
//! the Unros framwork.

use std::{collections::VecDeque, ops::Deref, sync::Arc, time::Duration};

use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros_core::{
    anyhow, async_trait,
    bytes::Bytes,
    setup_logging,
    signal::{Publisher, Subscriber, Subscription},
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt},
        sync::Mutex,
        task::JoinSet,
    },
    Node, RuntimeContext,
};

/// A single duplex connection to a serial port
pub struct SerialConnection {
    path: Arc<str>,
    baud_rate: u32,
    msg_received: Option<Publisher<Bytes>>,
    messages_to_send: Arc<Mutex<Subscriber<Bytes>>>,
    tolerate_error: bool,
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
            msg_received: Some(Default::default()),
            messages_to_send: Arc::new(Mutex::new(Subscriber::default())),
            tolerate_error,
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
        self.msg_received.as_mut().unwrap().accept_subscription(sub);
    }

    /// Provide a subscription whose messages will be written to the serial port.
    pub fn create_message_to_send_sub(&mut self) -> Subscription<Bytes> {
        Arc::get_mut(&mut self.messages_to_send)
            .unwrap()
            .get_mut()
            .create_subscription(8)
    }
}

#[async_trait]
impl Node for SerialConnection {
    const DEFAULT_NAME: &'static str = "serial_connection";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let msg_received = Arc::new(std::sync::Mutex::new(self.msg_received.take().unwrap()));

        loop {
            let Some(stream) = self.connect(&context).await? else {
                tokio::time::sleep(Duration::from_secs(2)).await;
                continue;
            };

            let (mut reader, mut writer) = tokio::io::split(stream);

            let mut tasks = JoinSet::<anyhow::Result<()>>::new();
            let msg_received = msg_received.clone();

            tasks.spawn(async move {
                let mut buf = [0; 1024];
                loop {
                    let n = reader.read(&mut buf).await?;
                    let bytes = buf.split_at(n).0.to_vec();
                    msg_received.lock().unwrap().set(bytes.into());
                }
            });
            let messages_to_send = self.messages_to_send.clone();

            tasks.spawn(async move {
                let mut messages_to_send = messages_to_send.lock().await;
                loop {
                    let msg = messages_to_send.recv().await;
                    writer.write_all(&msg).await?;
                }
            });

            let e = tasks
                .join_next()
                .await
                .unwrap()
                .expect("I/O should not have panicked")
                .unwrap_err();

            if self.tolerate_error {
                error!(
                    "Encountered the following error while communicating with: {}: {e}",
                    self.path
                );
            } else {
                break Err(e);
            }
        }
    }
}

struct VescReader {
    recv: Subscriber<Bytes>,
    buffer: VecDeque<u8>,
}

impl embedded_hal::serial::Read<u8> for VescReader {
    type Error = ();

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if let Some(msg) = self.recv.try_recv() {
            self.buffer.extend(msg.into_iter());
        }
        self.buffer.pop_back().ok_or(nb::Error::WouldBlock)
    }
}

#[derive(Default)]
struct VescWriter {
    signal: Publisher<Bytes>,
    buffer: Vec<u8>,
}

impl embedded_hal::serial::Write<u8> for VescWriter {
    type Error = ();

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.buffer.push(word);
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let buffer: Bytes = self.buffer.drain(..).collect();
        self.signal.set(buffer);
        Ok(())
    }
}

/// A single `VESC` connection to a serial port.
pub struct VescConnection {
    serial: SerialConnection,
    current: Subscriber<u32>,
    duty: Subscriber<u32>,
}

impl VescConnection {
    /// Wraps the given `SerialConnection` with the `VESC` protocol.
    ///
    /// The given `SerialConnection` should not have any subscriptions.
    pub fn new(serial: SerialConnection) -> Self {
        Self {
            serial,
            current: Subscriber::default(),
            duty: Subscriber::default(),
        }
    }

    /// Provide a subscription for the current level.
    pub fn connect_current_from(&mut self) -> Subscription<u32> {
        self.current.create_subscription(32)
    }

    /// Provide a subscription for the duty cycle.
    pub fn connect_duty_from(&mut self) -> Subscription<u32> {
        self.current.create_subscription(32)
    }
}

#[async_trait]
impl Node for VescConnection {
    const DEFAULT_NAME: &'static str = "vesc_connection";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        let mut writer = VescWriter::default();
        writer
            .signal
            .accept_subscription(self.serial.create_message_to_send_sub());
        let mut vesc_reader = VescReader {
            recv: Subscriber::default(),
            buffer: Default::default(),
        };
        self.serial
            .accept_msg_received_sub(vesc_reader.recv.create_subscription(8));
        let mut vesc = vesc_comm::VescConnection::new(vesc_reader, writer);

        tokio::select! {
            result = self.serial.run(context) => result,
            _ = async {
                loop {
                    tokio::select! {
                        current = self.current.recv() => {
                            vesc.set_current(current).expect("Setting current should have not panicked");
                        }
                        duty = self.duty.recv() => {
                            vesc.set_duty(duty).expect("Setting duty should have not panicked");
                        }
                    }
                }
            } => { unreachable!() }
        }
    }
}
