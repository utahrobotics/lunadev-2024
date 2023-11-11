use std::sync::Arc;

use image::{imageops::FilterType, DynamicImage};
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{CameraIndex, RequestedFormat, RequestedFormatType},
};
use unros_core::{
    anyhow::{self, Context},
    async_trait,
    tokio::{self, sync::mpsc},
    tokio_rayon, Node, OwnedSignal,
};

pub struct Camera {
    name: String,
    pub res_x: u32,
    pub res_y: u32,
    pub fps: u32,
    pub camera_index: u32,
    image_received: Option<OwnedSignal<Arc<DynamicImage>>>,
}

impl Camera {
    pub fn new(res_x: u32, res_y: u32, fps: u32, camera_index: u32) -> Self {
        Self {
            name: "camera".into(),
            res_x,
            res_y,
            fps,
            camera_index,
            image_received: Some(Default::default()),
        }
    }

    pub fn image_received_signal(&mut self) -> &mut OwnedSignal<Arc<DynamicImage>> {
        self.image_received.as_mut().unwrap()
    }
}

#[async_trait]
impl Node for Camera {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    async fn run(mut self) -> anyhow::Result<()> {
        let image_received = self.image_received.take().unwrap();
        let index = CameraIndex::Index(self.camera_index);

        let requested =
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::HighestFrameRate(self.fps));

        let (sender, mut receiver) = mpsc::channel(1);
        tokio::spawn(async move {
            loop {
                let Some(img) = receiver.recv().await else {
                    break;
                };
                image_received.emit(img).await;
            }
        });

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
                sender
                    .blocking_send(img)
                    .context("Failed to emit image frame")?;
            }
        })
        .await
    }
}
