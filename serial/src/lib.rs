use std::{borrow::Cow, collections::VecDeque, future::Future, pin::Pin, sync::Arc};

use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};
use unros_core::{
    anyhow, async_trait,
    tokio::{
        self,
        sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender},
    },
    Node, OwnedSignal, Signal,
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
