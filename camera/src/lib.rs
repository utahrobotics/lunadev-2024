//! This crate provides a node that can connect to any generic
//! color camera. This crate is cross-platform.
//!
//! Do note that this crate should not be expected to connect
//! to RealSense cameras.

use std::sync::Arc;

#[cfg(unix)]
use std::{sync::Mutex, time::Duration};

#[cfg(unix)]
pub use eye::hal::device::Description;
#[cfg(unix)]
use eye::hal::{
    format::PixelFormat,
    platform::Device,
    traits::{Context as HalContext, Device as HalDevice, Stream},
    PlatformContext,
};
#[cfg(unix)]
use image::codecs::jpeg::JpegDecoder;
use image::{imageops::FilterType, DynamicImage};
use unros::{
    anyhow, node::SyncNode, pubsub::{Publisher, PublisherRef}, setup_logging, DontDrop, runtime::RuntimeContext
};
#[cfg(not(unix))]
pub struct Description {
    pub uri: String,
    pub product: String,
}

#[cfg(unix)]
static PLATFORM: Mutex<Option<PlatformContext>> = Mutex::new(None);

/// A pending connection to a camera.
///
/// The connection is not created until this `Node` is ran.
pub struct Camera<F = fn(DynamicImage, u32, u32) -> DynamicImage> {
    pub fps: u32,
    pub res_x: u32,
    pub res_y: u32,
    #[cfg(unix)]
    device: Mutex<Device<'static>>,
    description: Description,
    image_received: Publisher<Arc<DynamicImage>>,
    dont_drop: DontDrop,
    #[allow(dead_code)]
    resizer: F,
}

fn crop_resize(img: DynamicImage, res_x: u32, res_y: u32) -> DynamicImage {
    img.resize_to_fill(res_x, res_y, FilterType::Triangle)
}

impl Camera {
    /// Creates a pending connection to the camera with the given index.
    pub fn new(description: Description) -> anyhow::Result<Self> {
        #[cfg(unix)]
        let mut platform_lock;
        #[cfg(unix)]
        let platform = {
            platform_lock = PLATFORM.lock().unwrap();
            if platform_lock.is_none() {
                *platform_lock = Some(
                    PlatformContext::all()
                        .next()
                        .ok_or_else(|| anyhow::anyhow!("Unable to get PlatformContext"))?,
                );
            }
            platform_lock.as_mut().unwrap()
        };

        Ok(Self {
            fps: 0,
            res_x: 0,
            res_y: 0,
            #[cfg(unix)]
            device: Mutex::new(platform.open_device(&description.uri)?),
            image_received: Default::default(),
            dont_drop: DontDrop::new(description.product.clone()),
            description,
            resizer: crop_resize,
        })
    }
}

impl<F: FnMut(DynamicImage, u32, u32) -> DynamicImage + Send> Camera<F> {
    pub fn get_camera_name(&self) -> &str {
        &self.description.product
    }

    pub fn get_camera_uri(&self) -> &str {
        &self.description.uri
    }

    /// Gets a reference to the `Signal` that represents received images.
    pub fn image_received_pub(&self) -> PublisherRef<Arc<DynamicImage>> {
        self.image_received.get_ref()
    }
}

impl<F> SyncNode for Camera<F>
where
    F: FnMut(DynamicImage, u32, u32) -> DynamicImage + Send + 'static,
{
    type Result = anyhow::Result<()>;
    const PERSISTENT: bool = true;

    #[cfg(unix)]
    fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;
        let device = self.device.get_mut().unwrap();
        let streams = device.streams()?;

        let Some(mut stream_desc) = streams.into_iter().next() else {
            return Err(anyhow::anyhow!(
                "No streams available for {} at {}",
                self.description.product,
                self.description.uri
            ));
        };

        if self.res_x != 0 {
            stream_desc.width = self.res_x;
        } else if self.res_y != 0 {
            stream_desc.height = self.res_y;
        }

        // stream_desc.pixfmt = PixelFormat::Rgb(8);

        stream_desc.interval = if self.fps == 0 {
            Duration::from_secs(1) / 30
        } else {
            Duration::from_secs(1) / self.fps
        };

        let mut stream = device.start_stream(&stream_desc)?;

        loop {
            let Some(result) = stream.next() else {
                break Ok(());
            };
            let frame = result?;
            if context.is_runtime_exiting() {
                break Ok(());
            }

            let mut img = match &stream_desc.pixfmt {
                PixelFormat::Gray(8) => todo!(),
                PixelFormat::Rgb(8) => todo!(),
                PixelFormat::Jpeg => {
                    let decoder = JpegDecoder::new(frame).unwrap();
                    DynamicImage::from_decoder(decoder)?
                }
                _ => unreachable!(),
            };

            img = (self.resizer)(img, self.res_x, self.res_y);
            if img.width() != self.res_x || img.height() != self.res_y {
                error!(
                    "Image was resized incorrectly to {}x{} instead of {}x{}",
                    img.width(),
                    img.height(),
                    self.res_x,
                    self.res_y
                );
            } else {
                self.image_received.set(Arc::new(img));
            }
        }
        // let index = CameraIndex::Index(self.camera_index);

        // let requested = if self.fps > 0 {
        //     if self.res_x > 0 && self.res_y > 0 {
        //         RequestedFormat::new::<RgbFormat>(RequestedFormatType::Exact(CameraFormat::new(
        //             Resolution::new(self.res_x, self.res_y),
        //             FrameFormat::RAWRGB,
        //             self.fps,
        //         )))
        //     } else {
        //         RequestedFormat::new::<RgbFormat>(RequestedFormatType::HighestFrameRate(self.fps))
        //     }
        // } else if self.res_x > 0 && self.res_y > 0 {
        //     RequestedFormat::new::<RgbFormat>(RequestedFormatType::HighestResolution(
        //         Resolution::new(self.res_x, self.res_y),
        //     ))
        // } else {
        //     RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate)
        // };

        // let res_x = self.res_x;
        // let res_y = self.res_y;
    }
    #[cfg(not(unix))]
    fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;
        warn!("Camera node is not supported on this platform");
        Ok(())
    }
}

/// Returns an iterator over all the cameras that were identified on this computer.
#[cfg(unix)]
pub fn discover_all_cameras() -> anyhow::Result<impl Iterator<Item = Camera>> {
    let ctx = PlatformContext::all()
        .next()
        .ok_or_else(|| anyhow::anyhow!("Unable to get PlatformContext"))?;
    Ok(ctx
        .devices()?
        .into_iter()
        .filter_map(|desc| match Camera::new(desc) {
            Ok(cam) => Some(cam),
            Err(e) => {
                unros::log::error!("{e:?}");
                None
            }
        }))
}

/// Returns an iterator over all the cameras that were identified on this computer.
#[cfg(not(unix))]
pub fn discover_all_cameras() -> anyhow::Result<impl Iterator<Item = Camera>> {
    Ok(std::iter::empty())
}
