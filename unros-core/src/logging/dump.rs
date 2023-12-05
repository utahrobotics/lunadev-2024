//! Data dumps are an alternative way of logging that is more suited to
//! large collections of data.
//!
//! Data dumps offer a way to write data to some location such that the
//! code producing the data does not get blocked by writing. If the write
//! is queued successfully, then the write is guaranteed to occur, as long
//! as the current program is not forcefully terminated.

use std::{
    fmt::Display,
    error::Error,
    io::Write,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::OnceLock, time::{Instant, Duration},
};

use image::DynamicImage;
use log::error;
use tokio::{
    fs::File,
    io::{AsyncWrite, AsyncWriteExt, BufWriter},
    net::TcpStream,
    sync::mpsc,
};
use video_rs::{Encoder, EncoderSettings, Locator, ffmpeg::{format::Pixel, ffi::{av_image_fill_arrays, AVPixelFormat}}, RawFrame, Time, PixelFormat, Options};

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

#[derive(Debug)]
pub struct VideoWriteError;

impl Display for VideoWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "The video writing thread has failed for some reason")
    }
}
impl Error for VideoWriteError {}

pub struct VideoDataDump {
    writer: std::sync::mpsc::Sender<(DynamicImage, Time)>,
    start: Instant,
    elapsed: Duration
}

#[derive(Debug)]
pub enum VideoDumpInitError {
    VideoError(video_rs::Error),
    LoggingError(anyhow::Error),
}


impl Error for VideoDumpInitError {}
impl Display for VideoDumpInitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VideoDumpInitError::VideoError(e) => write!(f, "Faced an error initializing the video encoder: {e}"),
            VideoDumpInitError::LoggingError(e) => write!(f, "Faced an error setting up the logging for the video encoder: {e}"),
        }
    }
}


static VIDEO_INIT: OnceLock<()> = OnceLock::new();

impl VideoDataDump {
    pub fn new(
        width: u32,
        height: u32,
        path: impl AsRef<Path>,
    ) -> Result<Self, VideoDumpInitError> {
        VIDEO_INIT.get_or_try_init(|| {
            video_rs::init()
                .map_err(|e| VideoDumpInitError::LoggingError(anyhow::anyhow!(e.to_string())))
        })?;

        let path = if path.as_ref().is_absolute() {
            Locator::Path(path.as_ref().into())
        } else {
            let Some(sub_log_dir) = SUB_LOGGING_DIR.get() else {
                return Err(VideoDumpInitError::LoggingError(anyhow::anyhow!("Sub-logging directory has not been initialized with a call to `init_logger`, `async_run_all`, or `run_all`")));
            };
            Locator::Path(PathBuf::from(sub_log_dir).join(path.as_ref()))
        };

        let settings = EncoderSettings::for_h264_custom(width as usize, height as usize, PixelFormat::RGB24, Options::new_h264());
        let mut encoder =
            Encoder::new(&path, settings).map_err(|e| VideoDumpInitError::VideoError(e))?;
        let (writer, receiver) = std::sync::mpsc::channel::<(DynamicImage, Time)>();

        std::thread::spawn(move || loop {
            let Ok((frame, time)) = receiver.recv() else {
                if let Err(e) = encoder.finish() {
                    error!("Failed to close encoder: {e}");
                }
                break;
            };
            let frame = frame.into_rgb8().into_vec();
            let mut raw = RawFrame::new(Pixel::RGB24, width, height);
            unsafe {
                let raw = raw.as_mut_ptr();
                let err = av_image_fill_arrays(
                    (*raw).data.as_mut_ptr(),
                    (*raw).linesize.as_mut_ptr(),
                    frame.as_ptr(),
                    AVPixelFormat::AV_PIX_FMT_RGB24,
                    width as i32,
                    height as i32,
                    1
                );
                if err < 0 {
                    error!("Internal error: unable to fill raw image: {err}");
                    break;
                }
                if err as u32 / width / height != 3 {
                    error!("Internal error: size mismatch: {err}");
                    break;
                }
                (*raw).pts = time.with_time_base(encoder.time_base()).into_value().unwrap();
            }

            if let Err(e) = encoder.encode_raw(raw) {
                error!("Failed to write to encoder: {e}");
                break;
            }
        });

        Ok(Self { writer, start: Instant::now(), elapsed: Duration::ZERO })
    }

    pub fn write_frame(&mut self, frame: DynamicImage) -> Result<(), VideoWriteError> {
        let time = Time::from_secs_f64(self.elapsed.as_secs_f64());
        self.elapsed = self.start.elapsed();
        self.writer.send((frame, time)).map_err(|_| VideoWriteError)
    }
}
