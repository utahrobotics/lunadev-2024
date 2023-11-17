use std::{
    io::Write,
    net::SocketAddr,
    path::{Path, PathBuf},
};

use log::error;
use tokio::{
    fs::File,
    io::{AsyncWrite, AsyncWriteExt, BufWriter},
    net::TcpStream,
    sync::mpsc,
};

use crate::SUB_LOGGING_DIR;

struct DataDumpInner {
    writer: mpsc::UnboundedSender<Vec<u8>>,
    empty_vecs: mpsc::UnboundedReceiver<Vec<u8>>,
}

#[derive(Default)]
pub struct DataDump(Option<DataDumpInner>);

impl DataDump {
    pub async fn new_file(filename: impl AsRef<Path>) -> std::io::Result<Self> {
        let Some(path) = SUB_LOGGING_DIR.get() else {
            return Ok(Self(None));
        };
        let filename = PathBuf::from(filename.as_ref());
        let file = File::create(PathBuf::from(path).join(&filename)).await?;
        Self::new(BufWriter::new(file), filename.to_string_lossy())
    }

    pub async fn new_tcp(addr: SocketAddr) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr).await?;
        Self::new(BufWriter::new(stream), addr.to_string())
    }

    pub fn new<A>(mut writer: A, name: impl Into<String>) -> std::io::Result<Self>
    where
        A: AsyncWrite + Unpin + Send + 'static,
    {
        let name = name.into();
        let (empty_vecs_sender, empty_vecs) = mpsc::unbounded_channel();
        let (writer_sender, mut reader) = mpsc::unbounded_channel::<Vec<_>>();
        std::thread::spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_io()
                .build()
                .unwrap()
                .block_on(async move {
                    loop {
                        let Some(mut bytes) = reader.recv().await else {
                            break;
                        };
                        if let Err(e) = writer.write_all(&bytes).await {
                            error!("Failed to write to {name:?}: {e}");
                            return;
                        }
                        bytes.clear();
                        let _ = empty_vecs_sender.send(bytes);
                    }
                    if let Err(e) = writer.flush().await {
                        error!("Failed to flush to {name:?}: {e}");
                    }
                });
        });
        Ok(Self(Some(DataDumpInner {
            writer: writer_sender,
            empty_vecs,
        })))
    }
}

impl Write for DataDump {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let Some(inner) = self.0.as_mut() else {
            return Ok(buf.len());
        };
        let mut vec = inner
            .empty_vecs
            .try_recv()
            .unwrap_or_else(|_| Vec::with_capacity(buf.len()));

        let buf_capacity = vec.capacity();
        let n;
        if buf_capacity < buf.len() {
            n = buf_capacity;
            vec.extend_from_slice(buf.split_at(buf_capacity).0);
        } else {
            n = buf.len();
            vec.extend_from_slice(buf);
        }

        inner
            .writer
            .send(vec)
            .map_err(|_| std::io::Error::from(std::io::ErrorKind::BrokenPipe))?;

        Ok(n)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
