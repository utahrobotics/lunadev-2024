use std::{
    collections::VecDeque,
    future::Future,
    ops::Deref,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc,
    },
    time::Duration, io,
};

// use crossbeam::queue::SegQueue;
// use futures::{
//     stream::{SplitSink, SplitStream},
//     SinkExt, StreamExt,
// };
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
// use tokio_util::codec::{BytesCodec, Decoder, Framed, Encoder};
use unros_core::{
    anyhow, async_trait,
    bytes::{Bytes, BytesMut, BufMut},
    log::info,
    node_error, node_info, tokio::{
        self,
        runtime::Handle,
        sync::{
            mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender},
            Mutex,
        }, task::JoinSet, io::{AsyncReadExt, AsyncWriteExt},
    },
    Node, OwnedSignal, Signal, rayon,
};

pub struct SerialConnection {
    name: String,
    path: Arc<str>,
    baud_rate: u32,
    stream: Option<SerialStream>,
    msg_received: Option<OwnedSignal<Bytes>>,
    msg_to_send_sender: UnboundedSender<Bytes>,
    msg_to_send_receiver: Arc<Mutex<UnboundedReceiver<Bytes>>>,
    tolerate_error: bool,
}

impl SerialConnection {
    pub async fn new<'a>(
        path: String,
        baud_rate: u32,
        tolerate_error: bool,
    ) -> anyhow::Result<Self> {
        let (msg_to_send_sender, msg_to_send_receiver) = unbounded_channel();

        let mut out = Self {
            name: "serial_connection".into(),
            path: path.into_boxed_str().into(),
            stream: None,
            baud_rate,
            msg_received: Some(Default::default()),
            msg_to_send_sender,
            msg_to_send_receiver: Arc::new(Mutex::new(msg_to_send_receiver)),
            tolerate_error,
        };

        out.connect().await?;

        Ok(out)
    }

    async fn connect(&mut self) -> anyhow::Result<()> {
        let path = self.path.clone();
        let baud_rate = self.baud_rate;
        let result: tokio_serial::Result<_> = (|| {
            let mut stream = tokio_serial::new(path.deref(), baud_rate).open_native_async()?;
            stream.set_exclusive(false)?;
            stream.clear(tokio_serial::ClearBuffer::All)?;
            Ok(stream)
        })();

        let stream = if self.tolerate_error {
            match result {
                Ok(s) => Some(s),
                Err(e) => {
                    node_error!(self, "Failed to connect to: {}: {e}", self.path);
                    None
                }
            }
        } else {
            Some(result?)
        };

        if stream.is_some() {
            node_info!(self, "Succesfully connected to: {}", self.path);
        }
        self.stream = stream;
        Ok(())
    }

    pub fn get_msg_received_signal(&mut self) -> &mut OwnedSignal<Bytes> {
        self.msg_received.as_mut().unwrap()
    }

    pub fn connect_from(&self, signal: &mut impl Signal<Bytes>) {
        let sender = self.msg_to_send_sender.clone();
        signal.connect_to(move |x| {
            let _ = sender.send(x);
        });
    }
}

// struct LineCodec;

// impl Decoder for LineCodec {
//     type Item = Bytes;
//     type Error = io::Error;

//     fn decode(&mut self, buf: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
//         // let newline = src.as_ref().iter().position(|b| *b == b'\n');
//         // if let Some(n) = newline {
//         //     let line = src.split_to(n + 1);
//         //     return match std::str::from_utf8(&line) {
//         //         Ok(s) => Ok(Some(s.as_bytes().to_vec().into())),
//         //         Err(_) => Err(io::Error::new(io::ErrorKind::Other, "Invalid String")),
//         //     };
//         // }
//         // Ok(None)
//         if !buf.is_empty() {
//             let len = buf.len();
//             Ok(Some(buf.split_to(len).into()))
//         } else {
//             Ok(None)
//         }
//     }
// }

// impl Encoder<Bytes> for LineCodec {
//     type Error = io::Error;

//     fn encode(&mut self, item: Bytes, dst: &mut BytesMut) -> Result<(), Self::Error> {
//         // println!("In writer {:?}", &item);
//         dst.reserve(item.len());
//         dst.put(item);
//         // dst.put_u8(b'\n');
//         Ok(())
//     }
// }


#[async_trait]
impl Node for SerialConnection {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    async fn run(mut self) -> anyhow::Result<()> {
        let msg_received = Arc::new(self.msg_received.take().unwrap());

        loop {
            let Some(stream) = self.stream.take() else {
                tokio::time::sleep(Duration::from_secs(2)).await;
                self.connect().await?;
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
                    msg_received.emit(bytes.into()).await;
                    println!("in");
                }
            });
            let msg_to_send_receiver = self.msg_to_send_receiver.clone();

            tasks.spawn(async move {
                let mut msg_to_send_receiver = msg_to_send_receiver.lock().await;
                loop {
                    let msg = msg_to_send_receiver.recv().await.unwrap();
                    writer.write_all(&msg).await?;
                    println!("out");
                }
            });

            let e = tasks.join_next().await.unwrap().expect("I/O should not have panicked").unwrap_err();
            
            if self.tolerate_error {
                node_error!(
                    self,
                    "Encountered the following error while communicating with: {}: {e}",
                    self.path
                );
            } else {
                break Err(e);
            }
        }
    }
}

impl Drop for SerialConnection {
    fn drop(&mut self) {
        if let Some(stream) = &mut self.stream {
            if let Err(e) = stream.clear(tokio_serial::ClearBuffer::All) {
                node_error!(self, "Failed to clear buffer of serial device at: {}: {e}", self.path);
            }
        }
    }
}

struct VescReader {
    recv: std::sync::mpsc::Receiver<Bytes>,
    buffer: VecDeque<u8>,
}

impl embedded_hal::serial::Read<u8> for VescReader {
    type Error = ();

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if let Ok(msg) = self.recv.try_recv() {
            self.buffer.extend(msg.into_iter());
        }
        self.buffer.pop_back().ok_or(nb::Error::WouldBlock)
    }
}

struct VescWriter {
    signal: Arc<Mutex<OwnedSignal<Bytes>>>,
    buffer: Vec<u8>,
    handle: Handle,
}

impl embedded_hal::serial::Write<u8> for VescWriter {
    type Error = ();

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.buffer.push(word);
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let buffer: Bytes = self.buffer.drain(..).collect();
        let signal = self.signal.clone();
        self.handle.spawn(async move {
            signal.lock().await.emit(buffer).await;
        });
        Ok(())
    }
}

impl Signal<Bytes> for VescWriter {
    fn connect_to(&mut self, receiver: impl Fn(Bytes) + Send + Sync + 'static) {
        self.signal.blocking_lock().connect_to(receiver);
    }

    fn connect_to_async<F>(&mut self, receiver: impl Fn(Bytes) -> F + Send + Sync + 'static)
    where
        F: Future<Output = ()> + Send + Unpin + 'static,
    {
        self.signal.blocking_lock().connect_to_async(receiver);
    }

    fn connect_to_non_blocking(&mut self, receiver: impl Fn(Bytes) + Send + Sync + 'static)
    where
        Bytes: Send + 'static,
    {
        self.signal
            .blocking_lock()
            .connect_to_non_blocking(receiver);
    }

    fn connect_to_async_non_blocking<F>(
        &mut self,
        receiver: impl Fn(Bytes) -> F + Send + Sync + 'static,
    ) where
        F: Future<Output = ()> + Send + Unpin + 'static,
        Bytes: Send + 'static,
    {
        self.signal
            .blocking_lock()
            .connect_to_async_non_blocking(receiver);
    }
}

enum VescMessage {
    Current(u32),
    Duty(u32),
}

pub struct VescConnection {
    serial: SerialConnection,
    vesc_msg_sender: Sender<VescMessage>,
    vesc_msg_recv: Receiver<VescMessage>,
}

impl VescConnection {
    pub fn new(serial: SerialConnection) -> Self {
        let (vesc_msg_sender, vesc_msg_recv) = channel();
        Self {
            serial,
            vesc_msg_sender,
            vesc_msg_recv,
        }
    }

    pub fn connect_current_from(&self, signal: &mut impl Signal<u32>) {
        let vesc_msg_sender = self.vesc_msg_sender.clone();
        signal.connect_to(move |x| {
            let _ = vesc_msg_sender.send(VescMessage::Current(x));
        })
    }

    pub fn connect_duty_from(&self, signal: &mut impl Signal<u32>) {
        let vesc_msg_sender = self.vesc_msg_sender.clone();
        signal.connect_to(move |x| {
            let _ = vesc_msg_sender.send(VescMessage::Duty(x));
        })
    }
}

#[async_trait]
impl Node for VescConnection {
    fn set_name(&mut self, name: String) {
        self.serial.set_name(name);
    }
    fn get_name(&self) -> &str {
        self.serial.get_name()
    }
    async fn run(mut self) -> anyhow::Result<()> {
        let mut vesc_writer = VescWriter {
            signal: Default::default(),
            buffer: Default::default(),
            handle: Handle::current(),
        };
        self.serial.connect_from(&mut vesc_writer);
        let (send, recv) = std::sync::mpsc::channel();
        let vesc_reader = VescReader {
            recv,
            buffer: Default::default(),
        };
        self.serial.get_msg_received_signal().connect_to(move |x| {
            let _ = send.send(x);
        });
        let mut vesc = vesc_comm::VescConnection::new(vesc_reader, vesc_writer);

        rayon::spawn(move || loop {
            let Ok(msg) = self.vesc_msg_recv.recv() else {
                break;
            };
            match msg {
                VescMessage::Current(n) => vesc.set_current(n).unwrap(),
                VescMessage::Duty(n) => vesc.set_duty(n).unwrap(),
            }
        });

        self.serial.run().await
    }
}
