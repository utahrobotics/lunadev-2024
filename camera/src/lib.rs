//! This crate provides a node that can connect to any generic
//! color camera. This crate is cross-platform.
//!
//! Do note that this crate should not be expected to connect
//! to RealSense cameras.

use std::sync::Arc;

use image::{imageops::FilterType, DynamicImage};
use nokhwa::{
    pixel_format::RgbFormat,
    query,
    utils::{CameraIndex, RequestedFormat, RequestedFormatType},
};
use unros_core::{
    anyhow::{self, Context},
    async_trait, log,
    pubsub::{Publisher, Subscription},
    setup_logging, tokio_rayon, Node, NodeIntrinsics, RuntimeContext,
};

/// A pending connection to a camera.
///
/// The connection is not created until this `Node` is ran.
pub struct Camera {
    pub fps: u32,
    pub camera_index: u32,
    pub res_x: u32,
    pub res_y: u32,
    image_received: Publisher<Arc<DynamicImage>>,
    intrinsics: NodeIntrinsics<Self>,
}

impl Camera {
    /// Creates a pending connection to the camera with the given index.
    pub fn new(camera_index: u32) -> anyhow::Result<Self> {
        let tmp_camera = nokhwa::Camera::new(
            CameraIndex::Index(camera_index),
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate),
        )
        .context("Failed to initialize camera")?;
        log::info!(
            "Pinged {} with index {}",
            tmp_camera.info().human_name(),
            camera_index
        );
        Ok(Self {
            fps: 0,
            res_x: 0,
            res_y: 0,
            camera_index,
            image_received: Default::default(),
            intrinsics: Default::default(),
        })
    }

    /// Gets a reference to the `Signal` that represents received images.
    pub fn accept_image_received_sub(&mut self, sub: Subscription<Arc<DynamicImage>>) {
        self.image_received.accept_subscription(sub);
    }
}

#[async_trait]
impl Node for Camera {
    const DEFAULT_NAME: &'static str = "camera";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let index = CameraIndex::Index(self.camera_index);

        let requested = if self.fps > 0 {
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::HighestFrameRate(self.fps))
        } else {
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate)
        };

        let res_x = self.res_x;
        let res_y = self.res_y;

        tokio_rayon::spawn(move || {
            let mut camera =
                nokhwa::Camera::new(index, requested).context("Failed to initialize camera")?;
            camera.open_stream()?;
            loop {
                let frame = camera.frame()?;
                let decoded = frame.decode_image::<RgbFormat>().unwrap();
                let mut img = DynamicImage::from(decoded);
                if res_x != 0 && res_y != 0 {
                    img = img.resize(res_x, res_y, FilterType::CatmullRom);
                }
                self.image_received.set(Arc::new(img));
            }
        })
        .await
    }
}

/// Returns an iterator over all the cameras that were identified on this computer.
pub fn discover_all_cameras() -> anyhow::Result<impl Iterator<Item = Camera>> {
    Ok(query(nokhwa::utils::ApiBackend::Auto)?
        .into_iter()
        .filter_map(|info| {
            let CameraIndex::Index(n) = info.index() else {
                return None;
            };
            let Ok(cam) = Camera::new(*n) else {
                return None;
            };
            Some(cam)
        }))
}
