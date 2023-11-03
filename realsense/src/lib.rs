use std::{sync::{Arc, Mutex}, ops::Deref};

use realsense_rust::{device::Device, context::Context, pipeline::InactivePipeline, kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind}, config::Config, frame::{DepthFrame, GyroFrame, ColorFrame}};
use unros_core::{Node, async_trait, anyhow, node_warn, tokio_rayon, Signal};

pub struct RealSenseCamera {
    name: String,
    device: Device,
    context: Arc<Mutex<Context>>,
    image_received: Signal<DynamicImage>
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

        tokio_rayon::spawn(move || {
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
                let mut color_frames = frames.frames_of_type::<ColorFrame>();
                if !color_frames.is_empty() {
                    let depth_frame = depth_frames.pop().unwrap();
                    let tmp_distance =
                        depth_frame.distance(depth_frame.width() / 2, depth_frame.height() / 2)?;
                    if tmp_distance != 0.0 {
                        distance = tmp_distance;
                    }
                }

                // Get gyro
                let motion_frames = frames.frames_of_type::<GyroFrame>();
                if !motion_frames.is_empty() {
                    motion = *motion_frames[0].rotational_velocity();
                }

            }
        }).await

    }
}