//! Data dumps are an alternative way of logging that is more suited to
//! large collections of data.
//!
//! Data dumps offer a way to write data to some location such that the
//! code producing the data does not get blocked by writing. If the write
//! is queued successfully, then the write is guaranteed to occur, as long
//! as the current program is not forcefully terminated.

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

use super::SUB_LOGGING_DIR;

struct DataDumpInner {
    writer: mpsc::UnboundedSender<Vec<u8>>,
    empty_vecs: mpsc::UnboundedReceiver<Vec<u8>>,
}

/// A handle to the Data Dump thread that handles all writes
/// to a location.
///
/// This struct implements the blocking `Write` interface as provided
/// by `Rust`, instead of the non-blocking `AsyncWrite` offered by `tokio`.
/// However, writes are guaranteed to be non-blocking and thus safe to use in
/// async code.
///
/// # Note
/// A default `DataDump` does not write to any location. It is equivalent to `sink()`.
#[derive(Default)]
pub struct DataDump(Option<DataDumpInner>);

impl DataDump {
    /// Create a `DataDump` that writes to the given path.
    ///
    /// For logging purposes, the filename is used as the name of the dump.
    ///
    /// # Note
    /// If the given path is relative, it will be considered relative to the
    /// sub-logging directory. If a logging implementation has not been initialized
    /// through `init_logger`, `async_run_all`, or `run_all`, then this method will
    /// return a `NotFound` io error.
    pub async fn new_file(path: impl AsRef<Path>) -> std::io::Result<Self> {
        let file = if path.as_ref().is_absolute() {
            File::create(path.as_ref()).await?
        } else {
            let Some(sub_log_dir) = SUB_LOGGING_DIR.get() else {
                return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "Sub-logging directory has not been initialized with a call to `init_logger`, `async_run_all`, or `run_all`"));
            };
            let filename = PathBuf::from(path.as_ref());
            File::create(PathBuf::from(sub_log_dir).join(&filename)).await?
        };
        Self::new(BufWriter::new(file), path.as_ref().to_string_lossy())
    }

    /// Create a `DataDump` that writes to the network address.
    ///
    /// For logging purposes, the address is used as the name of the dump.
    pub async fn new_tcp(addr: SocketAddr) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr).await?;
        Self::new(BufWriter::new(stream), addr.to_string())
    }

    /// Create a `DataDump` that writes to the given writer.
    ///
    /// For logging purposes, the given name is used as the name of the dump.
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
