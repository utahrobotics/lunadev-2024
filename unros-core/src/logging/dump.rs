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
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::Arc,
    time::{Duration, Instant},
};

use crossbeam::{queue::ArrayQueue, utils::Backoff};
use ffmpeg_sidecar::{child::FfmpegChild, command::FfmpegCommand, event::FfmpegEvent};
use image::{DynamicImage, EncodableLayout};
use log::{error, info, warn};
use tokio::{
    fs::File,
    io::{AsyncWrite, AsyncWriteExt, BufWriter},
    net::TcpStream,
    sync::mpsc,
};

use crate::runtime::RuntimeContextExt;

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
    pub async fn new_file(
        path: impl AsRef<Path>,
        context: &impl RuntimeContextExt,
    ) -> std::io::Result<Self> {
        let file = if path.as_ref().is_absolute() {
            File::create(path.as_ref()).await?
        } else {
            // let Some(sub_log_dir) = SUB_LOGGING_DIR.get() else {
            //     return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "Sub-logging directory has not been initialized with a call to `init_logger`, `async_run_all`, or `run_all`"));
            // };
            File::create(PathBuf::from(context.get_dump_path()).join(path.as_ref())).await?
        };
        Self::new(
            BufWriter::new(file),
            path.as_ref().to_string_lossy(),
            context,
        )
    }

    /// Create a `DataDump` that writes to the network address.
    ///
    /// For logging purposes, the address is used as the name of the dump.
    pub async fn new_tcp(
        addr: SocketAddr,
        context: &impl RuntimeContextExt,
    ) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr).await?;
        Self::new(BufWriter::new(stream), addr.to_string(), context)
    }

    /// Create a `DataDump` that writes to the given writer.
    ///
    /// For logging purposes, the given name is used as the name of the dump.
    pub fn new<A>(
        mut writer: A,
        name: impl Into<String>,
        context: &impl RuntimeContextExt,
    ) -> std::io::Result<Self>
    where
        A: AsyncWrite + Unpin + Send + 'static,
    {
        let name = name.into();
        let (empty_vecs_sender, empty_vecs) = mpsc::unbounded_channel();
        let (writer_sender, mut reader) = mpsc::unbounded_channel::<Vec<_>>();
        context.spawn_persistent_async(async move {
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

/// An error faced while writing video frames.
#[derive(Debug)]
pub enum VideoWriteError {
    /// The size of the given image is incorrect.
    IncorrectDimensions {
        expected_width: u32,
        expected_height: u32,
        actual_width: u32,
        actual_height: u32,
    },
    /// There is no information attached to the error.
    Unknown,
}

impl Display for VideoWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unknown => write!(f, "The video writing thread has failed for some reason"),
            Self::IncorrectDimensions { expected_width: expected_x, expected_height: expected_y, actual_width: actual_x, actual_height: actual_y } => write!(f, "Image dimensions are wrong. Expected {expected_x}x{expected_y}, got {actual_x}x{actual_y}"),
        }
    }
}
impl Error for VideoWriteError {}

#[derive(Clone)]
enum VideoDataDumpType {
    Rtp(SocketAddrV4),
    File(PathBuf),
    Display,
}

impl std::fmt::Display for VideoDataDumpType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VideoDataDumpType::Rtp(addr) => write!(f, "{addr}"),
            VideoDataDumpType::File(path) => {
                write!(f, "{}", path.file_name().unwrap().to_string_lossy())
            }
            VideoDataDumpType::Display => {
                write!(f, "Display")
            }
        }
    }
}

/// A dump for writing images into videos using `ffmpeg`.
///
/// If `ffmpeg` is not installed, it will be downloaded locally
/// automatically.
pub struct VideoDataDump {
    video_writer: Arc<ArrayQueue<Arc<DynamicImage>>>,
    width: u32,
    height: u32,
    dump_type: VideoDataDumpType,
    // path: PathBuf,
    // start: Instant,
}

/// An error faced while initializing a `VideoDataDump`.
#[derive(Debug)]
pub enum VideoDumpInitError {
    /// An error writing to or reading from `ffmpeg`.
    IOError(std::io::Error),
    /// An error from `ffmpeg` while it was encoding the video.
    VideoError(String),
    /// An error setting up the logging for the dump.
    LoggingError(anyhow::Error),
    /// An error automatically installing `ffmpeg`.
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

/// The type of filter used when scaling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalingFilter {
    /// Nearest neighbor. Excellent for performance.
    ///
    /// This adds no blurring whatsoever when upscaling, and mediocre quality when downscaling.
    Neighbor,
    /// Uses a fast bilinear algorithm. Good for performance.
    ///
    /// This adds some blurring when upscaling, and average quality when downscaling.
    FastBilinear,
}

impl std::fmt::Display for ScalingFilter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Neighbor => write!(f, "neighbor"),
            Self::FastBilinear => write!(f, "fast_bilinear"),
        }
    }
}

impl VideoDataDump {
    /// Generates the sdp file with the given address
    #[must_use]
    pub fn generate_sdp(addr: SocketAddrV4) -> String {
        format!(
            "v=0
o=- 0 0 IN IP4 127.0.0.1
s=No Name
c=IN IP4 {}
t=0 0
a=tool:libavformat 58.76.100
m=video {} RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1",
            addr.ip(),
            addr.port()
        )
    }

    /// Creates a new `VideoDataDump` that displays to a window.
    ///
    /// `ffplay` may need to be installed separately.
    pub fn new_display(
        in_width: u32,
        in_height: u32,
        fps: usize,
        context: &impl RuntimeContextExt,
    ) -> Result<Self, VideoDumpInitError> {
        let mut cmd = Command::new("ffplay")
            .args([
                "-f",
                "rawvideo",
                "-pixel_format",
                "rgb24",
                "-video_size",
                &format!("{in_width}x{in_height}"),
                "-vf",
                &format!("fps={fps}"),
                "-i",
                "-",
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(VideoDumpInitError::IOError)?;

        let queue_sender = Arc::new(ArrayQueue::<Arc<DynamicImage>>::new(1));
        let queue_receiver = queue_sender.clone();

        let mut video_out = cmd.stdin.take().unwrap();
        // let mut video_err = cmd.stderr.unwrap();

        let backoff = Backoff::new();

        context.spawn_persistent_sync(move || loop {
            let Some(frame) = queue_receiver.pop() else {
                if Arc::strong_count(&queue_receiver) == 1 {
                    if let Err(e) = video_out.flush() {
                        error!("Failed to flush Display: {e}");
                    }
                    break;
                }
                backoff.snooze();
                continue;
            };
            backoff.reset();

            if let Err(e) = video_out.write_all(frame.to_rgb8().as_bytes()) {
                if e.kind() == ErrorKind::BrokenPipe {
                    match cmd.wait_with_output() {
                        Ok(output) => {
                            if !output.status.success() {
                                let err = String::from_utf8_lossy(&output.stderr);
                                error!("Display closed: {err}");
                            }
                        }
                        Err(e) => {
                            error!("Display closed: {e}");
                        }
                    }
                    break;
                } else {
                    error!("Faced the following error while writing video frame to Display: {e}");
                }
            }
        });

        Ok(Self {
            video_writer: queue_sender,
            width: in_width,
            height: in_height,
            dump_type: VideoDataDumpType::Display,
        })
    }

    /// Creates a new `VideoDataDump` that writes to a video file.
    pub fn new_file(
        in_width: u32,
        in_height: u32,
        out_width: u32,
        out_height: u32,
        scale_filter: ScalingFilter,
        path: impl AsRef<Path>,
        fps: usize,
        context: &impl RuntimeContextExt,
    ) -> Result<Self, VideoDumpInitError> {
        ffmpeg_sidecar::download::auto_download()
            .map_err(|e| VideoDumpInitError::FFMPEGInstallError(e.to_string()))?;
        let pathbuf: PathBuf = if path.as_ref().is_absolute() {
            path.as_ref().into()
        } else {
            // let Some(sub_log_dir) = SUB_LOGGING_DIR.get() else {
            //     return Err(VideoDumpInitError::LoggingError(anyhow::anyhow!("Sub-logging directory has not been initialized with a call to `init_logger`, `async_run_all`, or `run_all`")));
            // };
            PathBuf::from(context.get_dump_path()).join(path.as_ref())
        };

        let output = FfmpegCommand::new()
            .hwaccel("auto")
            .args([
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                &format!("{in_width}x{in_height}"),
            ])
            .input("-")
            .args([
                "-vf",
                &format!("fps={fps},scale={out_width}:{out_height}"),
                "-sws_flags",
                &scale_filter.to_string(),
            ])
            .args(["-c:v", "libx265"])
            .args(["-y".as_ref(), pathbuf.as_os_str()])
            .spawn()
            .map_err(VideoDumpInitError::IOError)?;

        Self::new(
            in_width,
            in_height,
            fps,
            VideoDataDumpType::File(pathbuf),
            output,
            context,
        )
    }

    /// Creates a new `VideoDataDump` that streams to a client over RTP.
    pub fn new_rtp(
        in_width: u32,
        in_height: u32,
        out_width: u32,
        out_height: u32,
        scale_filter: ScalingFilter,
        addr: SocketAddrV4,
        fps: usize,
        context: &impl RuntimeContextExt,
    ) -> Result<Self, VideoDumpInitError> {
        ffmpeg_sidecar::download::auto_download()
            .map_err(|e| VideoDumpInitError::FFMPEGInstallError(e.to_string()))?;

        let output = FfmpegCommand::new()
            .hwaccel("auto")
            .format("rawvideo")
            .pix_fmt("rgb24")
            .size(in_width, in_height)
            .input("-")
            .codec_video("libx264")
            .pix_fmt("yuv420p")
            .args([
                "-crf",
                "35",
                "-an",
                "-vf",
                &format!("fps={fps},scale={out_width}:{out_height}"),
                "-sws_flags",
                &scale_filter.to_string(),
            ])
            .args([
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-strict",
                "2",
                "-avioflags",
                "direct",
                "-rtsp_transport",
                "udp",
            ])
            // .args(["-sdp_file", "sdp.txt"])
            .format("rtp")
            .output(format!("rtp://{addr}"))
            .spawn()
            .map_err(VideoDumpInitError::IOError)?;

        Self::new(
            in_width,
            in_height,
            fps,
            VideoDataDumpType::Rtp(addr),
            output,
            context,
        )
    }

    fn new(
        in_width: u32,
        in_height: u32,
        fps: usize,
        dump_type: VideoDataDumpType,
        mut output: FfmpegChild,
        context: &impl RuntimeContextExt,
    ) -> Result<Self, VideoDumpInitError> {
        let queue_sender = Arc::new(ArrayQueue::<Arc<DynamicImage>>::new(1));
        let queue_receiver = queue_sender.clone();

        let mut video_out = output.take_stdin().unwrap();

        let events = output
            .iter()
            .map_err(|e| VideoDumpInitError::VideoError(e.to_string()))?;

        let dump_type2 = dump_type.clone();

        context.spawn_persistent_sync(move || {
            events.for_each(|event| {
                if let FfmpegEvent::Log(level, msg) = event {
                    match level {
                        ffmpeg_sidecar::event::LogLevel::Info => info!("[{dump_type2}] {msg}"),
                        ffmpeg_sidecar::event::LogLevel::Warning => warn!("[{dump_type2}] {msg}"),
                        ffmpeg_sidecar::event::LogLevel::Unknown => {}
                        _ => error!("[{dump_type2}] {msg}"),
                    }
                }
            });
        });

        let dump_type2 = dump_type.clone();
        let backoff = Backoff::new();
        let mut last_frame = None;
        let mut instant = Instant::now();
        let delta = Duration::from_millis((1000 / fps) as u64);

        context.spawn_persistent_sync(move || loop {
            let elapsed = instant.elapsed();
            let frame = match queue_receiver.pop() {
                Some(x) => {
                    last_frame = Some(x.clone());
                    x
                }
                None => {
                    if Arc::strong_count(&queue_receiver) == 1 {
                        if let Err(e) = video_out.flush() {
                            error!("Failed to flush {}: {e}", dump_type2);
                        }
                        break;
                    }
                    backoff.snooze();
                    if let Some(last_frame) = last_frame.clone() {
                        if elapsed >= delta {
                            last_frame
                        } else {
                            continue;
                        }
                    } else {
                        continue;
                    }
                }
            };

            instant += elapsed;
            backoff.reset();

            if let Err(e) = video_out.write_all(frame.to_rgb8().as_bytes()) {
                if e.kind() == ErrorKind::BrokenPipe {
                    error!("{} has closed!", dump_type2);
                    break;
                } else {
                    error!(
                        "Faced the following error while writing video frame to {}: {e}",
                        dump_type2
                    );
                }
            }
        });

        Ok(Self {
            video_writer: queue_sender,
            width: in_width,
            height: in_height,
            dump_type,
        })
    }

    /// Writes an image into this dump.
    pub fn write_frame(&mut self, frame: Arc<DynamicImage>) -> Result<(), VideoWriteError> {
        if frame.width() != self.width || frame.height() != self.height {
            return Err(VideoWriteError::IncorrectDimensions {
                expected_width: self.width,
                expected_height: self.height,
                actual_width: frame.width(),
                actual_height: frame.height(),
            });
        }

        if Arc::strong_count(&self.video_writer) == 1 {
            return Err(VideoWriteError::Unknown);
        }
        match self.video_writer.force_push(frame) {
            Some(_) => {
                warn!(
                    "Overwriting last frame in {} as queue was full!",
                    self.dump_type
                );
                Ok(())
            }
            None => Ok(()),
        }
    }

    pub fn write_frame_quiet(&mut self, frame: Arc<DynamicImage>) -> Result<(), VideoWriteError> {
        if frame.width() != self.width || frame.height() != self.height {
            return Err(VideoWriteError::IncorrectDimensions {
                expected_width: self.width,
                expected_height: self.height,
                actual_width: frame.width(),
                actual_height: frame.height(),
            });
        }

        if Arc::strong_count(&self.video_writer) == 1 {
            return Err(VideoWriteError::Unknown);
        }
        self.video_writer.force_push(frame);
        Ok(())
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

// /// An error while writing subtitles to a video.
// pub enum SubtitleWriteError {
//     /// An error while writing the subtitiles to the file.
//     IOError(std::io::Error),
// }

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
