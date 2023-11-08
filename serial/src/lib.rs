use std::{borrow::Cow, collections::VecDeque, future::Future, pin::Pin, sync::{Arc, mpsc::{Sender, Receiver}}};

use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros_core::{
    anyhow, async_trait,
    tokio::{
        self,
        sync::{mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender}, Mutex}, runtime::Handle,
    },
    Node, OwnedSignal, Signal, rayon,
};

pub struct SerialConnection {
    name: String,
    stream: SerialStream,
    msg_received: OwnedSignal<Arc<[u8]>>,
    msg_to_send_sender: UnboundedSender<Arc<[u8]>>,
    msg_to_send_receiver: UnboundedReceiver<Arc<[u8]>>,
}

impl SerialConnection {
    pub fn new<'a>(path: impl Into<Cow<'a, str>>, baud_rate: u32) -> tokio_serial::Result<Self> {
        let mut stream = tokio_serial::new(path, baud_rate).open_native_async()?;
        stream.set_exclusive(true)?;
        let (msg_to_send_sender, msg_to_send_receiver) = unbounded_channel();
        Ok(Self {
            name: "serial_connection".into(),
            stream,
            msg_received: Default::default(),
            msg_to_send_sender,
            msg_to_send_receiver,
        })
    }

    pub fn get_msg_received_signal(&mut self) -> &mut OwnedSignal<Arc<[u8]>> {
        &mut self.msg_received
    }

    pub fn connect_from(&self, signal: &mut impl Signal<Arc<[u8]>>) {
        let sender = self.msg_to_send_sender.clone();
        signal.connect_to(move |x| {
            let _ = sender.send(x);
        });
    }
}

#[async_trait]
impl Node for SerialConnection {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }
    fn get_name(&self) -> &str {
        &self.name
    }
    async fn run(mut self) -> anyhow::Result<()> {
        let mut buf = Vec::with_capacity(1024);
        let mut send_buf = VecDeque::with_capacity(1024);

        loop {
            let write_fut: Pin<Box<dyn Future<Output = std::io::Result<()>> + Send>> =
                if send_buf.is_empty() {
                    Box::pin(std::future::pending())
                } else {
                    Box::pin(self.stream.writable())
                };
            tokio::select! {
                result = self.stream.readable() => {
                    result?;
                    let max_buf_size = self.stream.bytes_to_read()? as usize;
                    if max_buf_size > buf.len() {
                        buf.resize(max_buf_size, 0);
                    }
                    let n = match self.stream.try_read(&mut buf) {
                        Ok(n) => n,
                        Err(e) => if e.kind() == std::io::ErrorKind::WouldBlock {
                            continue;
                        } else {
                            return Err(e.into());
                        }
                    };
                    let bytes = buf.split_at(n).0.to_vec();
                    self.msg_received.emit(bytes.into_boxed_slice().into()).await;

                }
                msg = self.msg_to_send_receiver.recv() => {
                    let msg = msg.unwrap();
                    send_buf.extend(msg.into_iter());
                }
                result = write_fut => {
                    result?;
                    let n = self.stream.try_write(send_buf.make_contiguous())?;
                    send_buf.drain(0..n);
                }
            }
        }
    }
}


struct VescReader {
    recv: std::sync::mpsc::Receiver<Arc<[u8]>>,
    buffer: VecDeque<u8>
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
    signal: Arc<Mutex<OwnedSignal<Arc<[u8]>>>>,
    buffer: Vec<u8>,
    handle: Handle
}


impl embedded_hal::serial::Write<u8> for VescWriter {
    type Error = ();

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.buffer.push(word);
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let buffer: Arc<[u8]> = self.buffer.drain(..).collect();
        let signal = self.signal.clone();
        self.handle.spawn(async move {
            signal.lock().await.emit(buffer).await;
        });
        Ok(())
    }
}


impl Signal<Arc<[u8]>> for VescWriter {
    fn connect_to(&mut self, receiver: impl Fn(Arc<[u8]>) + Send + Sync + 'static) {
        self.signal.blocking_lock().connect_to(receiver);
    }

    fn connect_to_async<F>(&mut self, receiver: impl Fn(Arc<[u8]>) -> F + Send + Sync + 'static)
    where
        F: Future<Output = ()> + Send + Unpin + 'static {
        self.signal.blocking_lock().connect_to_async(receiver);
    }

    fn connect_to_non_blocking(&mut self, receiver: impl Fn(Arc<[u8]>) + Send + Sync + 'static)
    where
        Arc<[u8]>: Send + 'static {
        self.signal.blocking_lock().connect_to_non_blocking(receiver);
    }

    fn connect_to_async_non_blocking<F>(
        &mut self,
        receiver: impl Fn(Arc<[u8]>) -> F + Send + Sync + 'static,
    ) where
        F: Future<Output = ()> + Send + Unpin + 'static,
        Arc<[u8]>: Send + 'static {
        self.signal.blocking_lock().connect_to_async_non_blocking(receiver);
    }
}


enum VescMessage {
    Current(u32),
    Duty(u32)
}


pub struct VescConnection {
    pub serial: SerialConnection,
    vesc_msg_sender: Sender<VescMessage>,
    vesc_msg_recv: Receiver<VescMessage>
}


impl VescConnection {
    pub fn connect_current_from(&self, signal: &mut impl Signal<u32>) {
        let vesc_msg_sender = self.vesc_msg_sender.clone();
        signal.connect_to(move |x| { let _ = vesc_msg_sender.send(VescMessage::Current(x)); })
    }

    pub fn connect_duty_from(&self, signal: &mut impl Signal<u32>) {
        let vesc_msg_sender = self.vesc_msg_sender.clone();
        signal.connect_to(move |x| { let _ = vesc_msg_sender.send(VescMessage::Duty(x)); })
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
        let mut vesc_writer = VescWriter { signal: Default::default(), buffer: Default::default(), handle: Handle::current() };
        self.serial.connect_from(&mut vesc_writer);
        let (send, recv) = std::sync::mpsc::channel();
        let vesc_reader = VescReader { recv, buffer: Default::default() };
        self.serial.get_msg_received_signal().connect_to(move |x| { let _ = send.send(x); });
        let mut vesc = vesc_comm::VescConnection::new(vesc_reader, vesc_writer);

        rayon::spawn(move || {
            loop {
                let Ok(msg) = self.vesc_msg_recv.recv() else { break; };
                match msg {
                    VescMessage::Current(n) => vesc.set_current(n).unwrap(),
                    VescMessage::Duty(n) => vesc.set_duty(n).unwrap(),
                }
            }
        });

        self.serial.run().await
    }
}
