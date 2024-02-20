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
    io::{ErrorKind, Write},
    net::{SocketAddr, SocketAddrV4},
    num::NonZeroU32,
    path::{Path, PathBuf},
};

use ffmpeg_sidecar::{child::FfmpegChild, command::FfmpegCommand, event::FfmpegEvent};
use image::DynamicImage;
use log::{error, info, warn};
use tokio::{
    fs::File,
    io::{AsyncWrite, AsyncWriteExt, BufWriter},
    net::TcpStream,
    sync::{mpsc, oneshot},
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
    video_writer: std::sync::mpsc::Sender<DynamicImage>,
    // path: PathBuf,
    // start: Instant,
}

#[derive(Debug)]
pub enum VideoDumpInitError {
    IOError(std::io::Error),
    VideoError(String),
    LoggingError(anyhow::Error),
    FFMPEGInstallError(String),
}

impl Error for VideoDumpInitError {}
impl Display for VideoDumpInitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VideoDumpInitError::IOError(e) => {
                write!(f, "Faced an error initializing the video encoder: {e}")
            }
            VideoDumpInitError::LoggingError(e) => write!(
                f,
                "Faced an error setting up the logging for the video encoder: {e}"
            ),
            VideoDumpInitError::FFMPEGInstallError(e) => write!(
                f,
                "Faced an error installing FFMPEG for the video encoder: {e}"
            ),
            VideoDumpInitError::VideoError(e) => {
                write!(f, "Faced an error while encoding video: {e}")
            }
        }
    }
}

pub enum SubtitleWriteError {
    IOError(std::io::Error),
}

impl VideoDataDump {
    pub fn new_file(
        width: u32,
        height: u32,
        path: impl AsRef<Path>,
        fps: usize,
    ) -> Result<Self, VideoDumpInitError> {
        ffmpeg_sidecar::download::auto_download()
            .map_err(|e| VideoDumpInitError::FFMPEGInstallError(e.to_string()))?;
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

        let output = FfmpegCommand::new()
            .args([
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                &format!("{width}x{height}"),
            ])
            .input("-")
            .args(["-vf", &format!("fps={fps}")])
            .args(["-c:v", "libx265"])
            .args(["-y".as_ref(), path.as_os_str()])
            .spawn()
            .map_err(VideoDumpInitError::IOError)?;

        Self::new(width, height, output)
    }

    pub async fn new_rtp(
        width: u32,
        height: u32,
        addr: SocketAddrV4,
        fps: usize,
    ) -> Result<(Self, oneshot::Receiver<String>), VideoDumpInitError> {
        ffmpeg_sidecar::download::auto_download()
            .map_err(|e| VideoDumpInitError::FFMPEGInstallError(e.to_string()))?;

        let Some(sub_log_dir) = SUB_LOGGING_DIR.get() else {
            return Err(VideoDumpInitError::LoggingError(anyhow::anyhow!("Sub-logging directory has not been initialized with a call to `init_logger`, `async_run_all`, or `run_all`")));
        };
        let sdp_file_path = PathBuf::from(sub_log_dir).join(format!("{addr}.sdp"));

        let output = FfmpegCommand::new()
            .hwaccel("auto")
            .args([
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                &format!("{width}x{height}"),
            ])
            .input("-")
            .args(["-vf", &format!("fps={fps}")])
            .args(["-c:v", "libx265", "-pix_fmt", "yuv420p", "-crf", "40"])
            .args([
                "-preset",
                "veryfast",
                "-tune",
                "zerolatency",
                "-strict",
                "2",
            ])
            .args(["-sdp_file".as_ref(), sdp_file_path.as_path()])
            .args(["-f", "rtp", &format!("rtp://{addr}?rtcpport={}", addr.port() + 1)])
            .spawn()
            .map_err(VideoDumpInitError::IOError)?;

        let (sender, receiver) = oneshot::channel();
        tokio::spawn(async move {
            loop {
                match tokio::fs::read_to_string(&sdp_file_path).await {
                    Ok(x) => {
                        if x.is_empty() {
                            continue;
                        }
                        let _ = sender.send(x);
                        break;
                    }
                    Err(e) => match e.kind() {
                        std::io::ErrorKind::NotFound => {}
                        _ => {
                            error!("Faced the following error while trying to read sdp file: {e}");
                            break;
                        }
                    },
                }
                std::hint::spin_loop();
            }
        });

        Ok((Self::new(width, height, output)?, receiver))
    }

    fn new(width: u32, height: u32, mut output: FfmpegChild) -> Result<Self, VideoDumpInitError> {
        let (writer, receiver) = std::sync::mpsc::channel::<DynamicImage>();

        let mut resizer = fast_image_resize::Resizer::new(
            fast_image_resize::ResizeAlg::Convolution(fast_image_resize::FilterType::Box),
        );

        let mut dst_image = fast_image_resize::Image::new(
            NonZeroU32::new(width).unwrap(),
            NonZeroU32::new(height).unwrap(),
            fast_image_resize::PixelType::U8x3,
        );

        let mut video_out = output.take_stdin().unwrap();

        let events = output
            .iter()
            .map_err(|e| VideoDumpInitError::VideoError(e.to_string()))?;

        spawn_persistent_thread(move || {
            events.for_each(|event| match event {
                FfmpegEvent::Log(level, msg) => match level {
                    ffmpeg_sidecar::event::LogLevel::Info => info!("{msg}"),
                    ffmpeg_sidecar::event::LogLevel::Warning => warn!("{msg}"),
                    ffmpeg_sidecar::event::LogLevel::Unknown => {}
                    _ => error!("{msg}"),
                },
                _ => {}
            });
        });

        spawn_persistent_thread(move || {
            loop {
                let Ok(frame) = receiver.recv() else {
                    if let Err(e) = video_out.flush() {
                        error!("Failed to flush encoder: {e}");
                    }
                    break;
                };
                let src_image = fast_image_resize::Image::from_vec_u8(
                    NonZeroU32::new(frame.width())
                        .expect("incoming image width should be non-zero"),
                    NonZeroU32::new(frame.height())
                        .expect("incoming image height should be non-zero"),
                    frame.into_rgb8().into_vec(),
                    fast_image_resize::PixelType::U8x3,
                )
                .unwrap();

                // Get mutable view of destination image data
                let mut dst_view = dst_image.view_mut();
                resizer.resize(&src_image.view(), &mut dst_view).unwrap();

                let frame = dst_image.buffer();
                if let Err(e) = video_out.write_all(frame) {
                    if e.kind() == ErrorKind::BrokenPipe {
                        error!("Video out has closed!");
                        break;
                    } else {
                        error!("Faced the following error while writing video frame: {e}");
                    }
                }
            }
        });

        Ok(Self {
            video_writer: writer,
            // start: Instant::now(),
            // path: path.to_path_buf(),
        })
    }

    pub fn write_frame(&mut self, frame: DynamicImage) -> Result<(), VideoWriteError> {
        self.video_writer.send(frame).map_err(|_| VideoWriteError)
    }

    // pub async fn init_subtitles(&self) -> Result<SubtitleDump, std::io::Error> {
    //     let path = PathBuf::from(self.path.file_stem().unwrap()).with_extension("srt");
    //     let mut timestamp = Timestamp::new(0, 0, 0, 0);
    //     timestamp.add_milliseconds(self.start.elapsed().as_millis() as i32);
    //     Ok(SubtitleDump {
    //         file: DataDump::new_file(path).await?,
    //         start: self.start,
    //         timestamp,
    //         last_sub: None,
    //         count: 0,
    //     })
    // }
}

// pub struct SubtitleDump {
//     file: DataDump,
//     start: Instant,
//     timestamp: Timestamp,
//     count: usize,
//     last_sub: Option<String>,
// }

// impl SubtitleDump {
//     pub fn write_subtitle(&mut self, subtitle: impl Into<String>) -> std::io::Result<()> {
//         let start_time = self.timestamp;
//         let mut end_time = start_time;
//         let now = Instant::now();
//         end_time.add_milliseconds(now.duration_since(self.start).as_millis() as i32);

//         if let Some(last_sub) = self.last_sub.take() {
//             self.count += 1;
//             let sub = Subtitle::new(self.count, start_time, end_time, last_sub);
//             self.start = now;
//             self.timestamp = end_time;
//             self.last_sub = Some(subtitle.into());
//             writeln!(self.file, "{sub}\n")
//         } else {
//             self.start = now;
//             self.timestamp = end_time;
//             self.last_sub = Some(subtitle.into());
//             Ok(())
//         }
//     }

//     pub fn clear_subtitle(&mut self) -> std::io::Result<()> {
//         let start_time = self.timestamp;
//         let mut end_time = start_time;
//         let now = Instant::now();
//         end_time.add_milliseconds(now.duration_since(self.start).as_millis() as i32);

//         if let Some(last_sub) = self.last_sub.take() {
//             self.count += 1;
//             let sub = Subtitle::new(self.count, start_time, end_time, last_sub);
//             self.start = now;
//             self.timestamp = end_time;
//             self.last_sub = None;
//             writeln!(self.file, "{sub}\n")
//         } else {
//             self.start = now;
//             self.timestamp = end_time;
//             self.last_sub = None;
//             Ok(())
//         }
//     }
// }

// impl Drop for SubtitleDump {
//     fn drop(&mut self) {
//         if let Some(last_sub) = self.last_sub.take() {
//             let start_time = self.timestamp;
//             let mut end_time = start_time;
//             let now = Instant::now();
//             end_time.add_milliseconds(now.duration_since(self.start).as_millis() as i32);
//             self.count += 1;
//             let sub = Subtitle::new(self.count, start_time, end_time, last_sub);
//             self.start = now;
//             self.timestamp = end_time;
//             self.last_sub = None;
//             if let Err(e) = writeln!(self.file, "{sub}\n") {
//                 error!("Failed to write final subtitle: {e}");
//             }
//         }
//     }
// }
