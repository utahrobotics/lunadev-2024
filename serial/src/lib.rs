use std::{collections::VecDeque, ops::Deref, sync::Arc, time::Duration};

use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros_core::{
    anyhow, async_trait,
    bytes::Bytes,
    log, setup_logging,
    signal::{bounded::BoundedSubscription, watched::WatchedSubscription, Signal, SignalRef},
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt},
        sync::Mutex,
        task::JoinSet,
    },
    Node, RuntimeContext,
};

pub struct SerialConnection {
    path: Arc<str>,
    baud_rate: u32,
    // stream: Option<SerialStream>,
    msg_received: Option<Signal<Bytes>>,
    messages_to_send: Arc<Mutex<BoundedSubscription<Bytes, 64>>>,
    tolerate_error: bool,
}

impl SerialConnection {
    pub async fn new<'a>(
        path: String,
        baud_rate: u32,
        tolerate_error: bool,
    ) -> anyhow::Result<Self> {
        let out = Self {
            path: path.into_boxed_str().into(),
            // stream: None,
            baud_rate,
            msg_received: Some(Default::default()),
            messages_to_send: Arc::new(Mutex::new(BoundedSubscription::none())),
            tolerate_error,
        };

        // out.connect().await?;

        Ok(out)
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

    pub fn get_msg_received_signal(&mut self) -> SignalRef<Bytes> {
        self.msg_received.as_mut().unwrap().get_ref()
    }

    pub fn message_to_send_subscription(&mut self, sub: BoundedSubscription<Bytes, 64>) {
        *Arc::get_mut(&mut self.messages_to_send).unwrap().get_mut() += sub;
    }
}

#[async_trait]
impl Node for SerialConnection {
    const DEFAULT_NAME: &'static str = "serial_connection";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let msg_received = Arc::new(self.msg_received.take().unwrap());

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
                    msg_received.set(bytes.into());
                }
            });
            let messages_to_send = self.messages_to_send.clone();

            tasks.spawn(async move {
                let mut messages_to_send = messages_to_send.lock().await;
                loop {
                    let msg = messages_to_send.recv().await.unwrap();
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
    recv: BoundedSubscription<Bytes, 32>,
    buffer: VecDeque<u8>,
}

impl embedded_hal::serial::Read<u8> for VescReader {
    type Error = ();

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if let Some(result) = self.recv.try_recv() {
            match result {
                Ok(msg) => self.buffer.extend(msg.into_iter()),
                Err(n) => log::warn!("Lagged by {n} messages"),
            }
        }
        self.buffer.pop_back().ok_or(nb::Error::WouldBlock)
    }
}

#[derive(Default)]
struct VescWriter {
    signal: Signal<Bytes>,
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

pub struct VescConnection {
    serial: SerialConnection,
    current: WatchedSubscription<u32>,
    duty: WatchedSubscription<u32>,
}

impl VescConnection {
    pub fn new(serial: SerialConnection) -> Self {
        Self {
            serial,
            current: WatchedSubscription::none(),
            duty: WatchedSubscription::none(),
        }
    }

    pub fn connect_current_from(&mut self, sub: WatchedSubscription<u32>) {
        self.current = sub;
    }

    pub fn connect_duty_from(&mut self, sub: WatchedSubscription<u32>) {
        self.duty = sub;
    }
}

#[async_trait]
impl Node for VescConnection {
    const DEFAULT_NAME: &'static str = "vesc_connection";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        let mut writer = VescWriter::default();
        self.serial
            .message_to_send_subscription(writer.signal.get_ref().subscribe_bounded());
        let vesc_reader = VescReader {
            recv: self.serial.get_msg_received_signal().subscribe_bounded(),
            buffer: Default::default(),
        };
        let mut vesc = vesc_comm::VescConnection::new(vesc_reader, writer);

        tokio::select! {
            result = self.serial.run(context) => result,
            _ = async {
                loop {
                    tokio::select! {
                        current = self.current.wait_for_change() => {
                            vesc.set_current(current).expect("Setting current should have not panicked");
                        }
                        duty = self.duty.wait_for_change() => {
                            vesc.set_duty(duty).expect("Setting duty should have not panicked");
                        }
                    }
                }
            } => { unreachable!() }
        }
    }
}
