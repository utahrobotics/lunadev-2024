use std::{
    collections::HashSet,
    ffi::OsStr,
    ops::Deref,
    os::unix::ffi::OsStrExt,
    path::Path,
    sync::{Arc, Mutex},
};

use compute_shader::{
    buffers::{DynamicSize, StaticSize},
    wgpu::include_wgsl,
    Compute,
};
use image::DynamicImage;
use localization::frames::IMUFrame;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use pollster::FutureExt;
use realsense_rust::{
    config::Config,
    context::Context,
    device::Device,
    frame::{AccelFrame, DepthFrame, GyroFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use realsense_sys::rs2_deproject_pixel_to_point;
use rig::RobotElementRef;
use unros::{
    anyhow,
    node::SyncNode,
    pubsub::{Publisher, PublisherRef},
    runtime::RuntimeContext,
    setup_logging, DontDrop, ShouldNotDrop,
};

pub use crate::iter::{RealSensePoints, RealSensePointsIter};

/// A connection to a RealSense Camera.
#[derive(ShouldNotDrop)]
pub struct RealSenseCamera {
    device: Device,
    context: Arc<Mutex<Context>>,
    image_received: Publisher<Arc<DynamicImage>>,
    point_cloud_received: Publisher<RealSensePoints>,
    imu_frame_received: Publisher<IMUFrame<f32>>,
    robot_element: Option<RobotElementRef>,
    pub focal_length_frac: f32,
    pub min_distance: f32,
    pub skip_n: usize,
    dont_drop: DontDrop<Self>,
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
            point_cloud_received: Default::default(),
            imu_frame_received: Default::default(),
            robot_element: None,
            focal_length_frac: 0.5,
            min_distance: 0.4,
            skip_n: 4,
            dont_drop: DontDrop::new(format!("realsense-{path:?}")),
        })
    }

    pub fn image_received_pub(&self) -> PublisherRef<Arc<DynamicImage>> {
        self.image_received.get_ref()
    }

    pub fn cloud_received_pub(&self) -> PublisherRef<RealSensePoints> {
        self.point_cloud_received.get_ref()
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

    /// IMU frames are in global space, according to the rigid body
    /// provided to the RealSense camera.
    pub fn imu_frame_received_pub(&self) -> PublisherRef<IMUFrame<f32>> {
        self.imu_frame_received.get_ref()
    }

    /// Sets the robot element that represents this camera.
    ///
    /// Images are assumed to be captured from this rigid body, and the
    /// RealSense IMU is assumed to be relative to this rigid body. If a
    /// robot element was not provided, IMU messages will *not* be produced.
    ///
    /// This will replace the last rigid body that was provided.
    ///
    pub fn set_robot_element_ref(&mut self, robot_element_ref: RobotElementRef) {
        self.robot_element = Some(robot_element_ref);
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
                .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, 0)?
                .enable_stream(Rs2StreamKind::Color, None, 0, 0, Rs2Format::Rgb8, 0)?;
        } else {
            warn!("This Realsense camera is not attached to a USB 3.0 port");
            config
                .enable_device_from_serial(self.device.info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, 0)?;
        }
        if self.robot_element.is_some() {
            config
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?
                .enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::Any, 0)?;
        }

        // Change pipeline's type from InactivePipeline -> ActivePipeline
        let mut pipeline = pipeline.start(Some(config))?;

        // let mut last_img = None;

        let mut last_accel: Vector3<f32> = Default::default();
        let mut last_ang_vel: Vector3<f32> = Default::default();

        let depth_stream = pipeline
            .profile()
            .streams()
            .iter()
            .filter(|stream| stream.kind() == Rs2StreamKind::Depth)
            .next()
            .unwrap();
        let depth_intrinsics = depth_stream.intrinsics()?;

        let rays: Box<[_]> = (0..depth_intrinsics.height())
            .into_iter()
            .flat_map(|y| {
                (0..depth_intrinsics.width())
                    .into_iter()
                    .zip(std::iter::repeat(y))
                    .map(|(x, y)| {
                        let mut ray = [0.0f32; 3];
                        let pixel = [x as f32, y as f32];

                        unsafe {
                            rs2_deproject_pixel_to_point(
                                ray.first_mut().unwrap(),
                                &depth_intrinsics.0,
                                pixel.first().unwrap(),
                                1.0,
                            );
                        }

                        [ray[0], ray[1], -ray[2]]
                    })
            })
            .collect();
        let compute: Compute<(Option<&[[f32; 3]]>, &[u32], &f32, &f32), [[f32; 4]]> = Compute::new(
            include_wgsl!("project.wgsl"),
            (
                DynamicSize::new(rays.len()),
                DynamicSize::new(rays.len() / 2),
                StaticSize::default(),
                StaticSize::default(),
            ),
            DynamicSize::new(rays.len()),
            (
                (depth_intrinsics.width() / 2) as u32,
                depth_intrinsics.height() as u32,
                1,
            ),
        )
        .block_on()?;
        let mut rays = Some(rays);

        // let mut depth_buffer = vec![];

        loop {
            let frames = pipeline.wait(None)?;
            if context.is_runtime_exiting() {
                break Ok(());
            }

            // // Get color
            // for frame in frames.frames_of_type::<ColorFrame>() {
            //     let Some(img) = ImageBuffer::<Rgb<u8>, _>::from_raw(
            //         frame.width() as u32,
            //         frame.height() as u32,
            //         frame
            //             .iter()
            //             .flat_map(|px| {
            //                 let PixelKind::Bgr8 { r, g, b } = px else {
            //                     unreachable!()
            //                 };
            //                 [*r, *g, *b]
            //             })
            //             .collect(),
            //     ) else {
            //         error!("Failed to copy realsense color image");
            //         continue;
            //     };
            //     last_img = Some(img.clone());
            //     let img = Arc::new(DynamicImage::from(img));
            //     self.image_received.set(img);
            // }

            let Some(robot_element) = &self.robot_element else {
                continue;
            };

            for frame in frames.frames_of_type::<GyroFrame>() {
                last_ang_vel = nalgebra::convert(Vector3::from(*frame.rotational_velocity()));
            }

            for frame in frames.frames_of_type::<AccelFrame>() {
                last_accel = nalgebra::convert(Vector3::from(*frame.acceleration()));
            }

            let (w, [i, j, k]) = quaternion_core::from_euler_angles(
                quaternion_core::RotationType::Intrinsic,
                quaternion_core::RotationSequence::XYZ,
                [last_ang_vel.x, last_ang_vel.y, last_ang_vel.z],
            );

            self.imu_frame_received.set(IMUFrame {
                acceleration: last_accel,
                angular_velocity: UnitQuaternion::new_normalize(Quaternion::new(w, i, j, k)),
                acceleration_variance: 0.65,
                angular_velocity_variance: 0.005,
                robot_element: robot_element.clone(),
            });

            // let Some(last_img) = &last_img else {
            //     continue;
            // };

            for frame in frames.frames_of_type::<DepthFrame>() {
                let scale = frame.depth_units().unwrap();
                let depths = unsafe {
                    let ptr: *const _ = frame.get_data();
                    let ptr: *const u32 = ptr.cast();
                    std::slice::from_raw_parts(ptr, frame.width() * frame.height() / 2)
                };
                let points_buf = compute
                    .call(
                        rays.take().as_ref().map(|x| x.deref()),
                        depths,
                        &self.min_distance,
                        &scale,
                    )
                    .block_on();

                // use unros::rayon::iter::{IntoParallelIterator, ParallelIterator};
                // let min_x = points.into_par_iter().map(|p| ordered_float::NotNan::new(p.0.x).unwrap()).min().unwrap().into_inner();
                // let max_x = points.into_par_iter().map(|p| ordered_float::NotNan::new(p.0.x).unwrap()).max().unwrap().into_inner();
                // let min_y = points.into_par_iter().map(|p| ordered_float::NotNan::new(p.0.y).unwrap()).min().unwrap().into_inner();
                // let max_y = points.into_par_iter().map(|p| ordered_float::NotNan::new(p.0.y).unwrap()).max().unwrap().into_inner();
                // let min_z = points.into_par_iter().map(|p| ordered_float::NotNan::new(p.0.z).unwrap()).min().unwrap().into_inner();
                // let max_z = points.into_par_iter().map(|p| ordered_float::NotNan::new(p.0.z).unwrap()).max().unwrap().into_inner();
                // println!("{min_x:.2} {max_x:.2} {min_y:.2} {max_y:.2} {min_z:.2} {max_z:.2}");
                // let mut total: Vector3<f32> = points.into_par_iter().map(|p| (p.0.coords - origin).normalize()).sum();
                // total.normalize_mut();
                // println!("{total}");
                self.point_cloud_received.set(RealSensePoints {
                    buffer: Arc::new(points_buf),
                });
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
        point_cloud_received: Default::default(),
        imu_frame_received: Default::default(),
        robot_element: None,
        focal_length_frac: 0.5,
        min_distance: 0.4,
        skip_n: 4,
        dont_drop: DontDrop::new("realsense"),
    }))
}
