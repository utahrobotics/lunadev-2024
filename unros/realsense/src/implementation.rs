use std::{
    collections::HashSet,
    ffi::OsStr,
    ops::Deref,
    os::unix::ffi::OsStrExt,
    path::Path,
    sync::{Arc, Mutex},
};

use image::{DynamicImage, ImageBuffer, Rgb};
use realsense_rust::{
    config::Config,
    context::Context,
    device::Device,
    frame::{ColorFrame, PixelKind},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use unros::{
    anyhow,
    node::SyncNode,
    pubsub::{Publisher, PublisherRef},
    runtime::RuntimeContext,
    setup_logging, DontDrop, ShouldNotDrop,
};

/// A connection to a RealSense Camera.
#[derive(ShouldNotDrop)]
pub struct RealSenseCamera {
    device: Device,
    context: Arc<Mutex<Context>>,
    image_received: Publisher<Arc<DynamicImage>>,
    dont_drop: DontDrop<Self>,
    pub res_x: u32,
    pub res_y: u32,
    pub fps: usize,
}

impl RealSenseCamera {
    /// Attempts to connect to the camera at the given `dev` path.
    pub fn open(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let mut context = Context::new()?;
        let path = path.as_ref();
        let device = context.add_device(path)?;
        Ok(Self {
            device,
            context: Arc::new(Mutex::new(context)),
            image_received: Default::default(),
            dont_drop: DontDrop::new(format!("realsense-{path:?}")),
            res_x: 0,
            res_y: 0,
            fps: 0
        })
    }

    pub fn image_received_pub(&self) -> PublisherRef<Arc<DynamicImage>> {
        self.image_received.get_ref()
    }

    pub fn get_path(&self) -> &Path {
        let path = self
            .device
            .info(Rs2CameraInfo::PhysicalPort)
            .expect("Failed to query camera port")
            .to_bytes();
        Path::new(OsStr::from_bytes(path))
    }

    pub fn get_name(&self) -> &Path {
        let path = self
            .device
            .info(Rs2CameraInfo::Name)
            .expect("Failed to query camera name")
            .to_bytes();
        Path::new(OsStr::from_bytes(path))
    }
}

impl SyncNode for RealSenseCamera {
    type Result = anyhow::Result<()>;
    const PERSISTENT: bool = true;

    fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;
        let pipeline = InactivePipeline::try_from(self.context.lock().unwrap().deref())?;
        let mut config = Config::new();

        let usb_cstr = self.device.info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
        let usb_val: f32 = usb_cstr.to_str().unwrap().parse().unwrap();
        if usb_val >= 3.0 {
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                .enable_stream(
                    Rs2StreamKind::Color,
                    None,
                    self.res_x as usize,
                    self.res_y as usize,
                    Rs2Format::Rgb8,
                    self.fps,
                )?;
        } else {
            warn!("This Realsense camera is not attached to a USB 3.0 port");
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, 0)?;
        }

        // Change pipeline's type from InactivePipeline -> ActivePipeline
        let mut pipeline = pipeline.start(Some(config))?;

        loop {
            let frames = pipeline.wait(None)?;
            if context.is_runtime_exiting() {
                break Ok(());
            }

            // Get color
            for frame in frames.frames_of_type::<ColorFrame>() {
                let Some(img) = ImageBuffer::<Rgb<u8>, _>::from_raw(
                    frame.width() as u32,
                    frame.height() as u32,
                    frame
                        .iter()
                        .flat_map(|px| {
                            let PixelKind::Bgr8 { r, g, b } = px else {
                                unreachable!()
                            };
                            [*r, *g, *b]
                        })
                        .collect(),
                ) else {
                    error!("Failed to copy realsense color image");
                    continue;
                };
                let img = Arc::new(DynamicImage::from(img));
                self.image_received.set(img);
            }
        }
    }
}

/// Returns an iterator over all the RealSense cameras that were identified on this computer.
pub fn discover_all_realsense() -> anyhow::Result<impl Iterator<Item = RealSenseCamera>> {
    let context = Context::new()?;
    let devices = context.query_devices(HashSet::new());
    let context = Arc::new(Mutex::new(context));

    Ok(devices.into_iter().map(move |device| RealSenseCamera {
        device,
        context: context.clone(),
        image_received: Default::default(),
        dont_drop: DontDrop::new("realsense"),
        res_x: 0,
        res_y: 0,
        fps: 0
    }))
}
