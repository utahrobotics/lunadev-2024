use std::sync::Arc;

use image::{imageops::FilterType, DynamicImage};
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{CameraIndex, RequestedFormat, RequestedFormatType},
};
use unros_core::{
    anyhow::{self, Context},
    async_trait, setup_logging,
    signal::{Signal, SignalRef},
    tokio_rayon, Node, RuntimeContext,
};

pub struct Camera {
    pub res_x: u32,
    pub res_y: u32,
    pub fps: u32,
    pub camera_index: u32,
    image_received: Signal<Arc<DynamicImage>>,
}

impl Camera {
    pub fn new(res_x: u32, res_y: u32, fps: u32, camera_index: u32) -> Self {
        Self {
            res_x,
            res_y,
            fps,
            camera_index,
            image_received: Default::default(),
        }
    }

    pub fn image_received_signal(&mut self) -> SignalRef<Arc<DynamicImage>> {
        self.image_received.get_ref()
    }
}

#[async_trait]
impl Node for Camera {
    const DEFAULT_NAME: &'static str = "camera";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let index = CameraIndex::Index(self.camera_index);

        let requested =
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::HighestFrameRate(self.fps));

        let res_x = self.res_x;
        let res_y = self.res_y;

        tokio_rayon::spawn(move || {
            let mut camera =
                nokhwa::Camera::new(index, requested).context("Failed to initialize camera")?;
            loop {
                let frame = camera.frame()?;
                let decoded = frame.decode_image::<RgbFormat>().unwrap();
                let img = DynamicImage::from(decoded);
                let img = Arc::new(img.resize(res_x, res_y, FilterType::CatmullRom));
                self.image_received.set(img);
            }
        })
        .await
    }
}
