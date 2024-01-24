//! This crate provides a node that can connect to RealSense cameras and interpret
//! depth and color images.
//!
//! Unfortunately, this crate depends on the RealSense SDK. If you do not have this
//! SDK, remove this crate from the workspace.

use std::{
    collections::HashSet,
    ffi::OsStr,
    num::NonZeroU32,
    ops::Deref,
    os::unix::ffi::OsStrExt,
    path::Path,
    sync::{Arc, Mutex},
};

use bytemuck::cast_slice;
use cam_geom::{ExtrinsicParameters, IntrinsicParametersPerspective, PerspectiveParams, Pixels};
use image::{DynamicImage, ImageBuffer, Luma, Rgb};
use iter::ArcIter;
use localization::IMUFrame;
use nalgebra::{Dyn, Matrix, Point3, Quaternion, UnitQuaternion, VecStorage, Vector3, U2};
use realsense_rust::{
    config::Config,
    context::Context,
    device::Device,
    frame::{AccelFrame, ColorFrame, DepthFrame, GyroFrame, PixelKind},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use rig::RobotElementRef;
use unros_core::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscription},
    rayon::{
        iter::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator},
        join,
    },
    setup_logging, tokio_rayon, DropCheck, Node, RuntimeContext,
};

pub mod iter;

#[derive(Clone)]
pub struct PointCloud {
    /// An array of points in *global* space.
    ///
    /// The points have already been transformed
    /// to global space as efficient processing
    /// of this transformation is a hefty implementation
    /// detail that users need not worry
    /// about.
    pub points: Arc<[(Point3<f32>, image::Rgb<u8>)]>,
}

impl PointCloud {
    pub fn par_iter(&self) -> ArcIter<(Point3<f32>, image::Rgb<u8>)> {
        ArcIter::new(self.points.clone())
    }
}

/// A connection to a RealSense Camera.
pub struct RealSenseCamera {
    device: Device,
    context: Arc<Mutex<Context>>,
    image_received: Publisher<Arc<DynamicImage>>,
    point_cloud_received: Publisher<PointCloud>,
    imu_frame_received: Publisher<IMUFrame>,
    robot_element: Option<RobotElementRef>,
    pub focal_length_frac: f32,
    pub min_distance: f32,
}

impl RealSenseCamera {
    /// Attempts to connect to the camera at the given `dev` path.
    pub fn open(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let mut context = Context::new()?;
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
        })
    }

    pub fn accept_image_received_sub(&mut self, sub: Subscription<Arc<DynamicImage>>) {
        self.image_received.accept_subscription(sub);
    }

    pub fn accept_cloud_received_sub(&mut self, sub: Subscription<PointCloud>) {
        self.point_cloud_received.accept_subscription(sub);
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
    pub fn accept_imu_frame_received_sub(&mut self, sub: Subscription<IMUFrame>) {
        self.imu_frame_received.accept_subscription(sub);
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

#[async_trait]
impl Node for RealSenseCamera {
    const DEFAULT_NAME: &'static str = "realsense";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);
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

        let drop_check = DropCheck::default();
        let _outer_drop_check = drop_check.clone();
        let mut last_img = None;

        tokio_rayon::spawn(move || {
            let mut last_accel: Vector3<f32> = Default::default();
            let mut last_ang_vel: Vector3<f32> = Default::default();

            // Create Resizer instance and resize source image
            // into buffer of destination image
            let mut resizer = fast_image_resize::Resizer::new(
                fast_image_resize::ResizeAlg::Convolution(fast_image_resize::FilterType::Hamming),
            );

            loop {
                let frames = pipeline.wait(None)?;
                if drop_check.has_dropped() {
                    break Ok(());
                }

                // Get color
                for frame in frames.frames_of_type::<ColorFrame>() {
                    let Some(img) = ImageBuffer::<Rgb<u8>, _>::from_raw(
                        frame.width() as u32,
                        frame.height() as u32,
                        frame
                            .iter()
                            .map(|px| {
                                let PixelKind::Bgr8 { r, g, b } = px else {
                                    unreachable!()
                                };
                                [*r, *g, *b]
                            })
                            .flatten()
                            .collect(),
                    ) else {
                        error!("Failed to copy realsense color image");
                        continue;
                    };
                    last_img = Some(img.clone());
                    let img = Arc::new(DynamicImage::from(img));
                    self.image_received.set(img);
                }

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

                let Some(last_img) = &last_img else {
                    continue;
                };

                for frame in frames.frames_of_type::<DepthFrame>() {
                    let Some(img) = ImageBuffer::<Luma<u16>, Vec<_>>::from_raw(
                        frame.width() as u32,
                        frame.height() as u32,
                        frame
                            .iter()
                            .map(|px| {
                                let PixelKind::Z16 { depth } = px else {
                                    unreachable!()
                                };
                                *depth
                            })
                            .collect(),
                    ) else {
                        error!("Failed to copy realsense depth image");
                        continue;
                    };

                    let focal_length = frame.width() as f32 * self.focal_length_frac / 4.0;
                    let global_isometry = robot_element.get_global_isometry();
                    let frame_width = frame.width() as u32;
                    let frame_height = frame.height() as u32;
                    let min_distance = self.min_distance;

                    let (frame_bytes, raw_rays) = join(
                        || {
                            let width = NonZeroU32::new(frame_width).unwrap();
                            let height = NonZeroU32::new(frame_height).unwrap();
                            let src_image = fast_image_resize::Image::from_vec_u8(
                                width,
                                height,
                                cast_slice(&img).to_vec(),
                                fast_image_resize::PixelType::U16,
                            )
                            .unwrap();

                            let dst_width = NonZeroU32::new(frame_width / 4).unwrap();
                            let dst_height = NonZeroU32::new(frame_height / 4).unwrap();
                            let mut dst_image = fast_image_resize::Image::new(
                                dst_width,
                                dst_height,
                                fast_image_resize::PixelType::U16,
                            );

                            // Get mutable view of destination image data
                            let mut dst_view = dst_image.view_mut();
                            resizer.resize(&src_image.view(), &mut dst_view).unwrap();
                            dst_image.into_vec()
                        },
                        || {
                            let cam_geom = cam_geom::Camera::new(
                                IntrinsicParametersPerspective::from(PerspectiveParams {
                                    fx: focal_length,
                                    fy: focal_length,
                                    skew: 0.0,
                                    cx: frame_width as f32 / 8.0,
                                    cy: frame_height as f32 / 8.0,
                                }),
                                ExtrinsicParameters::from_pose(&nalgebra::convert(global_isometry)),
                            );
                            let pixel_coords = (0..frame_height / 4).into_iter().flat_map(|y| {
                                (0..frame_width / 4)
                                    .into_iter()
                                    .flat_map(move |x| [x as f32, y as f32])
                            });

                            let pixel_coords = Pixels::new(Matrix::<
                                f32,
                                Dyn,
                                U2,
                                VecStorage<f32, Dyn, U2>,
                            >::from_row_iterator(
                                frame_height as usize * frame_width as usize / 16,
                                pixel_coords,
                            ));
                            cam_geom.pixel_to_world(&pixel_coords)
                        },
                    );

                    let scale = frame.depth_units().unwrap();
                    let origin: Vector3<f32> =
                        nalgebra::convert(global_isometry.translation.vector);

                    let points: Arc<[_]> = cast_slice::<_, u16>(&frame_bytes)
                        .into_par_iter()
                        .enumerate()
                        .filter_map(|(i, depth)| {
                            if *depth == 0 {
                                return None;
                            }
                            let depth = *depth as f32 * scale;

                            if depth < min_distance {
                                return None;
                            }

                            let ray = raw_rays.data.row(i).transpose().normalize();

                            Some((
                                Point3::from(ray * depth + origin),
                                *last_img.get_pixel(
                                    i as u32 % (frame_width / 4) * 4,
                                    i as u32 / (frame_width / 4) * 4,
                                ),
                            ))
                        })
                        .collect();
                    self.point_cloud_received.set(PointCloud { points });
                }
            }
        })
        .await
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
    }))
}
