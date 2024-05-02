use std::ops::{Deref, DerefMut};

use fxhash::FxBuildHasher;
use image::{DynamicImage, ImageBuffer, Luma};
use localization::{
    engines::window::{DefaultWindowConfig, WindowLocalizer},
    frames::{IMUFrame, OrientationFrame, PositionFrame},
    Localizer,
};
use nalgebra::{Isometry3, Point2, Point3, Quaternion, UnitQuaternion, UnitVector3, Vector3};
use navigator::{
    drive::{diff::DifferentialDriver, DriveMode},
    pathfinding::{direct::DirectPathfinder, Pathfinder},
};
// use navigator::{
//     pathfinding::{direct::DirectPathfinder, Pathfinder},
//     DifferentialDriver, DriveMode,
// };
use obstacles::{sources::depth::DepthMapSource, HeightQuery, ObstacleHub, Shape};
// use navigator::{pathfinders::DirectPathfinder, DifferentialDriver};
use rand_distr::{Distribution, Normal};
use rig::Robot;
use unros::{
    anyhow, log,
    node::AsyncNode,
    pubsub::{subs::Subscription, Publisher, Subscriber},
    rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator, ParallelIterator},
    rng::quick_rng,
    runtime::MainRuntimeContext,
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt, BufStream},
        net::TcpListener,
    },
};

type Float = f32;
mod rays;

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "debug"])?;
    let mut camera = elements.remove("camera").unwrap();
    let camera_ref = camera.get_ref();
    let debug_element = elements.remove("debug").unwrap();

    let mut obstacle_hub = ObstacleHub::default();
    let depth_signal = Publisher::<Vec<f32>>::default();
    let depth_source = DepthMapSource::new(
        rays::RAYS.iter().copied().map(UnitVector3::new_unchecked),
        camera_ref,
        0.05,
        40000,
        32,
    )
    .await?;
    depth_signal.accept_subscription(depth_source.create_depth_subscription());
    obstacle_hub.add_source_mut(depth_source).unwrap();

    let mut pathfinder = Pathfinder::new_with_engine(
        0.1,
        DirectPathfinder {
            max_frac: 0.15,
            pathfind_shape: Shape::Cylinder {
                radius: 1.0,
                height: 4.0,
            },
            unsafe_shape: Shape::Cylinder {
                radius: 0.8,
                height: 4.0,
            },
            max_height_diff: 0.1,
            filter: |p: Point2<isize>| p.x.abs() <= 50 && p.y.abs() <= 50,
        },
        obstacle_hub.clone(),
        robot_base.get_ref(),
    );
    pathfinder.completion_distance = 0.1;
    pathfinder.correction_distance = 0.8;

    let robot_base_ref = robot_base.get_ref();
    let mut costmap_display = unros::logging::dump::VideoDataDump::new_display(200, 200, &context)?;
    tokio::spawn(async move {
        loop {
            let isometry = robot_base_ref.get_isometry();
            // tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            let mut heights = vec![0.0; 200 * 200];
            let origin = Vector3::new(0.0, 0.0, -1.0);
            let query: Box<[_]> = (-100..100)
                .into_iter()
                .flat_map(|y| {
                    (-100..100).into_iter().map(move |x| HeightQuery {
                        isometry: Isometry3::from_parts(
                            (isometry
                                * Point3::from(
                                    origin + Vector3::new(x as f32 * 0.02, 0.0, y as f32 * 0.02),
                                ))
                            .into(),
                            UnitQuaternion::default(),
                        ),
                        max_points: 32,
                        shape: Shape::Cylinder {
                            radius: 0.5,
                            height: 4.0,
                        },
                    })
                })
                .collect();

            let mut pending = obstacle_hub.query_height(query.iter().copied()).await;

            if let Some(heights_vec) = pending.next().await {
                heights_vec
                    .par_iter()
                    .map(|x| x.deref())
                    .zip(&mut heights)
                    .for_each(|(heights_in_shape, height)| {
                        let mean = heights_in_shape.iter().copied().sum::<f32>()
                            / heights_in_shape.len() as f32;
                        *height = mean;
                    });
            }

            let max_height = heights
                .iter()
                .copied()
                .filter(|x| x.is_finite())
                .fold(f32::NEG_INFINITY, f32::max)
                .max(0.1);
            let img_data: Vec<_> = heights
                .into_iter()
                .map(|h| {
                    if h.is_finite() {
                        (h / max_height * 255.0).round() as u8
                    } else {
                        0
                    }
                })
                .collect();
            let buf = ImageBuffer::<Luma<u8>, _>::from_raw(200, 200, img_data).unwrap();
            let img = DynamicImage::from(buf);
            if costmap_display.write_frame_quiet(img.into()).is_err() {
                break;
            }
        }
    });

    let path_sub = Subscriber::new(1);
    pathfinder
        .get_path_pub()
        .accept_subscription(path_sub.create_subscription());

    let mut driver = DifferentialDriver::new(robot_base.get_ref());
    driver.skip_distance = 0.5;
    let mut drive_mode_pub = driver.create_drive_mode_sub().into_mono_pub();
    drive_mode_pub.set(DriveMode::ForwardOnly);
    pathfinder
        .get_path_pub()
        .accept_subscription(driver.create_path_sub());
    let nav_task = pathfinder.get_navigation_handle();

    let mut localizer: Localizer<f32, WindowLocalizer<f32, _, _, _, _>> =
        Localizer::new(robot_base, DefaultWindowConfig::default());
    localizer.engine_config.isometry_func = |isometry| {
        isometry.translation.vector.y = 0.0;
        let mut scaled_axis = isometry.rotation.scaled_axis();
        scaled_axis.x = 0.0;
        scaled_axis.z = 0.0;
        isometry.rotation = UnitQuaternion::new(scaled_axis);
    };
    localizer.engine_config.additional_time_factor = 2.0;

    let position_pub = Publisher::default();
    position_pub.accept_subscription(localizer.create_position_sub().set_name("position"));

    let velocity_pub = Publisher::default();
    velocity_pub.accept_subscription(localizer.create_velocity_sub().set_name("velocity"));

    let orientation_pub = Publisher::default();
    orientation_pub.accept_subscription(localizer.create_orientation_sub().set_name("orientation"));

    let imu_pub = Publisher::default();
    imu_pub.accept_subscription(localizer.create_imu_sub().set_name("imu"));

    let steering_sub = Subscriber::new(1);
    driver
        .steering_pub()
        .accept_subscription(steering_sub.create_subscription());

    let tcp_listener = TcpListener::bind("0.0.0.0:11433").await?;
    tokio::spawn(async move {
        let (stream, _) = tcp_listener
            .accept()
            .await
            .expect("Connection should have succeeded");
        let mut stream = BufStream::new(stream);
        let mut depths = vec![];
        let mut last_left_steering = 0.0;
        let mut last_right_steering = 0.0;

        loop {
            let x = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let _y = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let z = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let _vx = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let _vy = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let _vz = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let ax = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let ay = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let az = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let w = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let i = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let j = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let k = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let vw = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let vi = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let vj = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let vk = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let x_rot = stream
                .read_f32_le()
                .await
                .expect("Failed to receive packet") as Float;
            let n = stream
                .read_u32_le()
                .await
                .expect("Failed to receive packet") as usize;

            position_pub.set(PositionFrame::rand(
                Point3::new(x, 0.0, z),
                0.03,
                debug_element.get_ref(),
            ));
            // velocity_pub.set(VelocityFrame::rand(
            //     Vector3::new(vx, vy, vz),
            //     0.03,
            //     debug_element.get_ref(),
            // ));
            let orientation = UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k));
            orientation_pub.set(OrientationFrame::rand(
                orientation,
                0.03,
                debug_element.get_ref(),
            ));
            imu_pub.set(IMUFrame::rand(
                Vector3::new(ax, ay, az),
                0.03,
                UnitQuaternion::new_unchecked(Quaternion::new(vw, vi, vj, vk)),
                0.03,
                debug_element.get_ref(),
            ));

            let mut camera_joint = match camera.get_local_joint() {
                rig::joints::JointMut::Hinge(x) => x,
                _ => unreachable!(),
            };

            camera_joint.set_angle(x_rot);
            depths.reserve(n.saturating_sub(depths.capacity()));
            let distr = Normal::new(0.0, 0.05).unwrap();
            let mut rng = quick_rng();
            assert_eq!(n, rays::RAYS.len());
            for _ in 0..n {
                let mut depth = stream.read_f32_le().await.expect("Failed to receive depth");
                depth *= 1.0 + distr.sample(rng.deref_mut());
                depths.push(depth);
            }

            let capacity = depths.capacity();
            depth_signal.set(depths);
            depths = Vec::with_capacity(capacity);

            if stream
                .read_u8()
                .await
                .expect("Failed to receive waypoint byte")
                == 255
            {
                let x = stream.read_f32_le().await.expect("Failed to receive point");
                let y = stream.read_f32_le().await.expect("Failed to receive point");
                match nav_task
                    .try_schedule_or_closed(Point3::new(x, 0.0, y))
                    .await
                {
                    Some(Ok(handle)) => {
                        tokio::spawn(async move {
                            match handle.wait().await {
                                Ok(()) => log::info!("Navigation complete"),
                                Err(e) => log::error!("{e}"),
                            }
                        });
                    }
                    Some(Err(e)) => log::error!("{e}"),
                    None => log::error!("Navigation task closed"),
                }
            }

            if let Some(steering) = steering_sub.try_recv() {
                last_left_steering = steering.left.into_inner();
                last_right_steering = steering.right.into_inner();
                stream
                    .write_f32_le(last_left_steering)
                    .await
                    .expect("Failed to write steering");
                stream
                    .write_f32_le(last_right_steering)
                    .await
                    .expect("Failed to write steering");
            } else {
                stream
                    .write_f32_le(last_left_steering)
                    .await
                    .expect("Failed to write steering");
                stream
                    .write_f32_le(last_right_steering)
                    .await
                    .expect("Failed to write steering");
            }

            let isometry = camera.get_isometry_of_base();

            stream
                .write_f32_le(isometry.translation.x)
                .await
                .expect("Failed to write position");
            stream
                .write_f32_le(isometry.translation.y)
                .await
                .expect("Failed to write position");
            stream
                .write_f32_le(isometry.translation.z)
                .await
                .expect("Failed to write position");

            stream
                .write_f32_le(isometry.rotation.w)
                .await
                .expect("Failed to write orientation");
            stream
                .write_f32_le(isometry.rotation.i)
                .await
                .expect("Failed to write orientation");
            stream
                .write_f32_le(isometry.rotation.j)
                .await
                .expect("Failed to write orientation");
            stream
                .write_f32_le(isometry.rotation.k)
                .await
                .expect("Failed to write orientation");

            stream.flush().await.expect("Failed to write steering");

            if let Some(path) = path_sub.try_recv() {
                stream
                    .write_u16_le(path.len() as u16)
                    .await
                    .expect("Failed to write path length");
                for point in path.iter() {
                    stream
                        .write_f32_le(point.x)
                        .await
                        .expect("Failed to write point.x");
                    stream
                        .write_f32_le(point.z)
                        .await
                        .expect("Failed to write point.z");
                }
            } else {
                stream
                    .write_u16_le(0)
                    .await
                    .expect("Failed to write path length");
            }

            stream.flush().await.expect("Failed to write path");
        }
    });

    driver.spawn(context.make_context("driver"));
    pathfinder.spawn(context.make_context("pathfinder"));
    localizer.spawn(context.make_context("localizer"));

    context.wait_for_exit().await;
    Ok(())
}