use std::{
    collections::HashSet,
    ops::Deref,
    path::Path,
    sync::{Arc, Mutex},
};

use image::{DynamicImage, ImageBuffer, Rgb};
use nalgebra::{Vector3, UnitQuaternion};
use quaternion_core::{to_euler_angles, RotationType, RotationSequence};
use realsense_rust::{
    config::Config,
    context::Context,
    device::Device,
    frame::{AccelFrame, ColorFrame, GyroFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use rig::{RigidBodyRef, StaticRigidBodyRef};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{Signal, SignalRef},
    tokio_rayon, Node, RuntimeContext,
};

#[derive(Clone, Copy)]
pub struct IMUFrame {
    pub acceleration: Vector3<f32>,
    pub angular_velocity: Vector3<f32>,
}

pub struct RealSenseCamera {
    device: Device,
    context: Arc<Mutex<Context>>,
    image_received: Signal<Arc<DynamicImage>>,
    imu_frame_received: Signal<IMUFrame>,
    rigid_body_ref: RigidBodyRef,
}

impl RealSenseCamera {
    pub fn open(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let mut context = Context::new()?;
        let device = context.add_device(path)?;
        Ok(Self {
            device,
            context: Arc::new(Mutex::new(context)),
            image_received: Default::default(),
            imu_frame_received: Default::default(),
            rigid_body_ref: RigidBodyRef::Static(StaticRigidBodyRef::identity("realsense")),
        })
    }

    pub fn image_received_signal(&mut self) -> SignalRef<Arc<DynamicImage>> {
        self.image_received.get_ref()
    }

    pub fn imu_frame_received(&mut self) -> SignalRef<IMUFrame> {
        self.imu_frame_received.get_ref()
    }

    pub fn set_rigid_body_ref(&mut self, rigid_body_ref: RigidBodyRef) {
        self.rigid_body_ref = rigid_body_ref;
    }
}

#[async_trait]
impl Node for RealSenseCamera {
    const DEFAULT_NAME: &'static str = "realsense";

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
        let pipeline = InactivePipeline::try_from(self.context.lock().unwrap().deref())?;
        let mut config = Config::new();

        let usb_cstr = self.device.info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
        let usb_val: f32 = usb_cstr.to_str().unwrap().parse().unwrap();
        if usb_val >= 3.0 {
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                // .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, 30)?
                .enable_stream(Rs2StreamKind::Color, None, 0, 0, Rs2Format::Rgb8, 0)?
                .enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::Any, 0)?
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
        } else {
            warn!("A Realsense camera is not attached to a USB 3.0 port");
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                // .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
                .enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::Any, 0)?
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
        }

        // Change pipeline's type from InactivePipeline -> ActivePipeline
        let mut pipeline = pipeline.start(Some(config))?;

        tokio_rayon::spawn(move || {
            let mut last_accel = Default::default();
            let mut last_ang_vel = Default::default();
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
                        let ptr: *const _ = frame.get_data();
                        let ptr: *const u8 = ptr.cast();
                        let buf = std::slice::from_raw_parts(ptr, frame.get_data_size()).to_vec();
                        let Some(img) = ImageBuffer::<Rgb<u8>, _>::from_raw(
                            frame.width() as u32,
                            frame.height() as u32,
                            buf,
                        ) else {
                            error!("Failed to copy realsense image");
                            continue;
                        };
                        let img = DynamicImage::from(img);
                        self.image_received.set(Arc::new(img));
                    }
                }

                for frame in frames.frames_of_type::<GyroFrame>() {
                    let ang_vel = frame.rotational_velocity();
                    let mut ang_vel = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), ang_vel[0]) * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -ang_vel[1]) * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), ang_vel[2]);
                    ang_vel = self.rigid_body_ref.get_global_isometry_f32().rotation * ang_vel;

                    let ang_vel = (ang_vel.w, [ang_vel.i, ang_vel.j, ang_vel.k]);

                    last_ang_vel = to_euler_angles(RotationType::Intrinsic, RotationSequence::XYZ, ang_vel).into();

                    self.imu_frame_received.set(IMUFrame {
                        acceleration: last_accel,
                        angular_velocity: last_ang_vel,
                    });
                }

                for frame in frames.frames_of_type::<AccelFrame>() {
                    let accel = frame.acceleration();
                    last_accel = self.rigid_body_ref.get_global_isometry_f32().rotation
                        * Vector3::new(accel[0], -accel[1], accel[2]);
                    self.imu_frame_received.set(IMUFrame {
                        acceleration: last_accel,
                        angular_velocity: last_ang_vel,
                    });
                }
            }
        })
        .await
    }
}

pub fn discover_all_realsense() -> anyhow::Result<impl Iterator<Item = RealSenseCamera>> {
    let context = Context::new()?;
    let devices = context.query_devices(HashSet::new());
    let context = Arc::new(Mutex::new(context));

    Ok(devices.into_iter().map(move |device| RealSenseCamera {
        device,
        context: context.clone(),
        image_received: Default::default(),
        imu_frame_received: Default::default(),
        rigid_body_ref: RigidBodyRef::Static(StaticRigidBodyRef::identity("realsense")),
    }))
}
