//! Data dumps are an alternative way of logging that is more suited to
//! large collections of data.
//!
//! Data dumps offer a way to write data to some location such that the
//! code producing the data does not get blocked by writing. If the write
//! is queued successfully, then the write is guaranteed to occur, as long
//! as the current program is not forcefully terminated.

use std::{
    error::Error,
    fmt::Display,
    io::Write,
    net::SocketAddr,
    num::NonZeroU32,
    path::{Path, PathBuf},
};

use ffmpeg_sidecar::command::FfmpegCommand;
use image::DynamicImage;
use log::error;
use tokio::{
    fs::File,
    io::{AsyncWrite, AsyncWriteExt, BufWriter},
    net::TcpStream,
    sync::mpsc,
};

use crate::spawn_persistent_thread;

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
            File::create(PathBuf::from(sub_log_dir).join(path.as_ref())).await?
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
        spawn_persistent_thread(move || {
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

#[derive(Debug)]
pub struct VideoWriteError;

impl Display for VideoWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "The video writing thread has failed for some reason")
    }
}
impl Error for VideoWriteError {}

pub struct VideoDataDump {
    writer: std::sync::mpsc::Sender<DynamicImage>,
}

#[derive(Debug)]
pub enum VideoDumpInitError {
    VideoError(std::io::Error),
    LoggingError(anyhow::Error),
}

impl Error for VideoDumpInitError {}
impl Display for VideoDumpInitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VideoDumpInitError::VideoError(e) => {
                write!(f, "Faced an error initializing the video encoder: {e}")
            }
            VideoDumpInitError::LoggingError(e) => write!(
                f,
                "Faced an error setting up the logging for the video encoder: {e}"
            ),
        }
    }
}

// static VIDEO_INIT: OnceLock<()> = OnceLock::new();

impl VideoDataDump {
    pub fn new(
        width: u32,
        height: u32,
        path: impl AsRef<Path>,
    ) -> Result<Self, VideoDumpInitError> {
        let pathbuf;
        let path = if path.as_ref().is_absolute() {
            path.as_ref()
        } else {
            let Some(sub_log_dir) = SUB_LOGGING_DIR.get() else {
                return Err(VideoDumpInitError::LoggingError(anyhow::anyhow!("Sub-logging directory has not been initialized with a call to `init_logger`, `async_run_all`, or `run_all`")));
            };
            pathbuf = PathBuf::from(sub_log_dir).join(path.as_ref());
            pathbuf.as_path()
        };
        let (writer, receiver) = std::sync::mpsc::channel::<DynamicImage>();

        let mut resizer = fast_image_resize::Resizer::new(
            fast_image_resize::ResizeAlg::Convolution(fast_image_resize::FilterType::Box),
        );

        let mut dst_image = fast_image_resize::Image::new(
            NonZeroU32::new(width).unwrap(),
            NonZeroU32::new(height).unwrap(),
            fast_image_resize::PixelType::U8x3,
        );

        let mut output = FfmpegCommand::new()
            .args([
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", &format!("{width}x{height}"), "-r", "60", "-use_wallclock_as_timestamps", "1"
            ])
            .input("-")
            .args(["-c:v", "libx265"])
            .args(["-y".as_ref(), path.as_os_str()])
            .spawn()
            .map_err(|e| VideoDumpInitError::VideoError(e))?;

        let mut video_out = output.take_stdin().unwrap();

        spawn_persistent_thread(move || {
            let _output = output;
            loop {
            let Ok(frame) = receiver.recv() else {
                if let Err(e) = video_out.flush() {
                    error!("Failed to flush encoder: {e}");
                }
                break;
            };
            let src_image = fast_image_resize::Image::from_vec_u8(
                NonZeroU32::new(frame.width()).expect("incoming image width should be non-zero"),
                NonZeroU32::new(frame.height()).expect("incoming image height should be non-zero"),
                frame.into_rgb8().into_vec(),
                fast_image_resize::PixelType::U8x3,
            )
            .unwrap();

            // Get mutable view of destination image data
            let mut dst_view = dst_image.view_mut();
            resizer.resize(&src_image.view(), &mut dst_view).unwrap();

            let frame = dst_image.buffer();
            if let Err(e) = video_out.write_all(frame) {
                error!("Faced the following error while writing video frame: {e}");
            }
        }});

        Ok(Self {
            writer,
        })
    }

    pub fn write_frame(&mut self, frame: DynamicImage) -> Result<(), VideoWriteError> {
        self.writer.send(frame).map_err(|_| VideoWriteError)
    }
}
