use std::{sync::{Arc, Mutex}, ops::Deref, collections::HashSet, path::Path};

use image::{DynamicImage, ImageBuffer, Rgb};
use realsense_rust::{device::Device, context::Context, pipeline::InactivePipeline, kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind, Rs2ProductLine}, config::Config, frame::{GyroFrame, ColorFrame}};
use unros_core::{Node, async_trait, anyhow, node_warn, tokio_rayon, OwnedSignal, tokio::runtime::Handle};

pub struct RealSenseCamera {
    name: String,
    device: Device,
    context: Arc<Mutex<Context>>,
    image_received: OwnedSignal<Arc<DynamicImage>>
}


impl RealSenseCamera {
    pub fn open(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let mut context = Context::new()?;
        let device = context.add_device(path)?;
        Ok(
            Self {
                name: "realsense".into(),
                device,
                context: Arc::new(Mutex::new(context)),
                image_received: Default::default()
            }
        )
    }
    pub fn image_received_signal(&mut self) -> &mut OwnedSignal<Arc<DynamicImage>> {
        &mut self.image_received
    }
}


#[async_trait]
impl Node for RealSenseCamera {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }
    fn get_name(&self) -> &str {
        &self.name
    }
    async fn run(self) -> anyhow::Result<()> {
        let pipeline = InactivePipeline::try_from(self.context.lock().unwrap().deref())?;
        let mut config = Config::new();
    
        let usb_cstr = self.device.info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
        let usb_val: f32 = usb_cstr.to_str().unwrap().parse().unwrap();
        if usb_val >= 3.0 {
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                // .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
                .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Rgb8, 30)?
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
        } else {
            node_warn!(self, "A Realsense camera is not attached to a USB 3.0 port");
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                // .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
        }

        // Change pipeline's type from InactivePipeline -> ActivePipeline
        let mut pipeline = pipeline.start(Some(config))?;
        let handle = Handle::current();

        tokio_rayon::spawn(move || {
            let image_received = Arc::new(self.image_received);
            loop {
                let frames = pipeline.wait(None)?;

                // Get depth
                // let mut depth_frames = frames.frames_of_type::<DepthFrame>();
                // if !depth_frames.is_empty() {
                //     let depth_frame = depth_frames.pop().unwrap();
                //     let tmp_distance =
                //         depth_frame.distance(depth_frame.width() / 2, depth_frame.height() / 2)?;
                //     if tmp_distance != 0.0 {
                //         distance = tmp_distance;
                //     }
                // }

                // Get color
                for frame in frames.frames_of_type::<ColorFrame>() {
                    unsafe {
                        let image_size = frame.width() * frame.height() * 3;
                        let ptr: *const _ = frame.get_data();
                        let ptr: *const u8 = ptr.cast();
                        let buf = std::slice::from_raw_parts(ptr, image_size).to_vec();
                        let img = ImageBuffer::<Rgb<u8>, _>::from_raw(frame.width() as u32, frame.height() as u32, buf).ok_or_else(|| anyhow::anyhow!("Failed to convert RealSense color frame into image"))?;
                        let img = DynamicImage::from(img);

                        let image_received = image_received.clone();
                        handle.spawn(async move {
                            image_received.emit(Arc::new(img)).await;
                        });
                    }
                }

                // Get gyro
                let motion_frames = frames.frames_of_type::<GyroFrame>();
                
            }
        }).await

    }
}


pub fn discover_all_realsense() -> anyhow::Result<impl Iterator<Item = RealSenseCamera>> {
    let context = Context::new()?;
    let devices = context.query_devices(HashSet::new());
    let context = Arc::new(Mutex::new(context));

    Ok(
        devices
            .into_iter()
            .map(move |device| {
                RealSenseCamera {
                    device,
                    context: context.clone(),
                    name: "realsense".into(),
                    image_received: Default::default()
                }
            })
    )
}