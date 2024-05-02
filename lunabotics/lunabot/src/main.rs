use apriltag::{AprilTagDetector, PoseObservation};
use camera::discover_all_cameras;
use costmap::CostmapGenerator;
use fxhash::FxBuildHasher;
use localization::{
    engines::window::{DefaultWindowConfig, WindowLocalizer},
    frames::{OrientationFrame, PositionFrame},
    Localizer,
};
use nalgebra::Point3;
use navigator::{pathfinding::Pathfinder, DifferentialDriver};
#[cfg(unix)]
use realsense::{discover_all_realsense, RealSensePoints};
use rig::Robot;
use telemetry::Telemetry;
use unros::{
    anyhow,
    node::{AsyncNode, SyncNode},
    pubsub::{subs::Subscription, Subscriber},
    runtime::MainRuntimeContext,
    setup_logging, ShouldNotDrop,
};

mod actuators;
mod drive;
mod serial;
// mod imu;
mod audio;
mod telemetry;

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    setup_logging!(context);
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "imu01"])?;
    let camera_element = elements.remove("camera").unwrap();
    let robot_base_ref = robot_base.get_ref();
    // let imu01 = elements.remove("imu01").unwrap().get_ref();

    let costmap = CostmapGenerator::new(10, robot_base_ref.clone());

    let mut cameras: Vec<_> = discover_all_cameras()?
        .filter_map(|mut x| {
            if x.get_camera_name().contains("RealSense") {
                x.ignore_drop();
                None
            } else {
                info!(
                    "Discovered {} at {}",
                    x.get_camera_name(),
                    x.get_camera_uri()
                );
                Some(x)
            }
        })
        .collect();
    info!("Discovered {} cameras", cameras.len());

    let telemetry = Telemetry::new(1280, 720, 20).await?;

    let arm_sub = Subscriber::new(8);
    telemetry
        .arm_pub()
        .accept_subscription(arm_sub.create_subscription());
    arm_sub
        .into_logger(|x| format!("{x:?}"), "arms.logs", &context)
        .await?;

    match serial::connect_to_serial() {
        Ok((arms, drive)) => {
            telemetry
                .steering_pub()
                .accept_subscription(drive.get_steering_sub());

            telemetry.arm_pub().accept_subscription(arms.get_arm_sub());
            drive.spawn(context.make_context("drive"));
            arms.spawn(context.make_context("arms"));
        }
        Err(e) => {
            error!("{e}");
        }
    }

    match cameras
        .iter()
        .enumerate()
        .filter_map(|(i, cam)| {
            if cam.get_camera_name() == "HD USB CAMERA: HD USB CAMERA" {
                Some(i)
            } else {
                None
            }
        })
        .next()
    {
        Some(i) => {
            let mut teleop_camera = cameras.remove(i);
            teleop_camera.res_x = 1280;
            teleop_camera.res_y = 720;
            teleop_camera
                .image_received_pub()
                .accept_subscription(telemetry.create_image_subscription());
            teleop_camera.spawn(context.make_context("teleop_camera"));
        }
        None => {
            error!("Teleop camera not found");
        }
    }

    //     let mut vid_writer = VideoDataDump::new_file(1280, 720, 24);
    //     tokio::spawn(async move { loop {

    // }});

    // let costmap_ref = costmap.clone();

    // let mut costmap_writer = VideoDataDump::new_file(80, 80, 24)?;
    // // let mut subtitle_writer = costmap_writer.init_subtitles().await?;

    // rayon::spawn(move || loop {
    //     std::thread::sleep(Duration::from_millis(42));
    //     let costmap = costmap_ref.lock().get_costmap();
    //     let obstacles = costmap_ref.costmap_to_obstacle(&costmap, 0.5, 0.0, 0.0);
    //     let img = costmap_ref.obstacles_to_img(&obstacles);

    //     costmap_writer.write_frame(img.into()).unwrap();
    // });

    let mut apriltag = AprilTagDetector::new(640.0, 1280, 720, camera_element.get_ref());
    apriltag.add_tag(Default::default(), Default::default(), 0.134, 0);
    // let mut pc_sub = Subscriber::default();
    // camera.accept_cloud_received_sub(pc_sub.create_subscription(1));

    // let las_node = FnNode::new(|_| async move {
    //     let mut i = 0;
    //     loop {
    //         let PointCloud { points } = pc_sub.recv().await;
    //         unros::tokio_rayon::spawn(move || {
    //             let mut header = las::Builder::default();
    //             header.point_format = las::point::Format {
    //                 has_color: true,
    //                 ..Default::default()
    //             };
    //             let mut writer = las::Writer::from_path(format!("{i}.las"), header.into_header().unwrap()).unwrap();
    //             use las::Write;
    //             for (point, color) in points.iter() {
    //                 let mut point = las::Point { x: point.x as f64, z: point.y as f64, y: point.z as f64, ..Default::default() };
    //                 point.color = Some(las::Color { red: color.0[0] as u16 * 255, green: color.0[1] as u16 * 255, blue: color.0[2] as u16 * 255 });
    //                 writer.write(point).unwrap();
    //             }
    //         }).await;
    //         i += 1;
    //     }
    // });

    let localizer: Localizer<f32, WindowLocalizer<f32, _, _, _, _>> =
        Localizer::new(robot_base, DefaultWindowConfig::default());

    apriltag
        .tag_detected_pub()
        .accept_subscription(
            localizer
                .create_position_sub()
                .map(|pose: PoseObservation| PositionFrame {
                    position: nalgebra::convert(Point3::from(pose.pose.translation.vector)),
                    variance: 0.1,
                    robot_element: pose.robot_element,
                }),
        );

    apriltag
        .tag_detected_pub()
        .accept_subscription(
            localizer
                .create_orientation_sub()
                .map(|pose: PoseObservation| OrientationFrame {
                    orientation: nalgebra::convert(pose.pose.rotation),
                    variance: 0.1,
                    robot_element: pose.robot_element,
                }),
        );

    #[cfg(unix)]
    {
        use costmap::Points;
        match discover_all_realsense()?.next() {
            Some(mut camera) => {
                camera.set_robot_element_ref(camera_element.get_ref());
                let camera_element_ref = camera_element.get_ref();
                camera.cloud_received_pub().accept_subscription(
                    costmap
                        .create_points_sub(0.1)
                        .map(move |points: RealSensePoints| Points {
                            points,
                            robot_element: camera_element_ref.clone(),
                        }),
                );

                camera.image_received_pub().accept_subscription(
                    apriltag
                        .create_image_subscription()
                        .set_name("RealSense Apriltag Image"),
                );
                camera
                    .imu_frame_received_pub()
                    .accept_subscription(localizer.create_imu_sub().set_name("RealSense IMU"));
                camera.spawn(context.make_context("realsense_camera"));
            }
            None => {
                error!("No realsense camera");
            }
        }
    }

    // let imu01 = open_imu(
    //     "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00",
    //     imu01,
    // )
    // .await;
    // imu01
    //     .msg_received_pub()
    //     .accept_subscription(localizer.create_imu_sub().set_name("imu01"));

    let pathfinder: Pathfinder =
        Pathfinder::new_with_engine(0.7, Default::default(), robot_base_ref.clone());
    let driver = DifferentialDriver::new(robot_base_ref.clone());

    // pathfinder.get_path_pub().accept_subscription(driver.create_path_sub());

    // let mut data_dump = DataDump::new_file("motion.csv", &context).await?;
    // writeln!(
    //     data_dump,
    //     "imu_ax,imu_ay,imu_az,imu_rvw,imu_rvi,imu_rvj,imu_rvk,vx,vy,vz,x,y,z,w,i,j,k,delta"
    // )
    // .unwrap();
    // let imu_sub = Subscriber::<IMUFrame>::new(32);
    // #[cfg(unix)]
    // realsense_camera
    //     .imu_frame_received_pub()
    //     .accept_subscription(imu_sub.create_subscription());
    //     app.add_task(|_| async move {
    //     let start = Instant::now();
    //     let mut elapsed = Duration::ZERO;

    //     loop {
    //         let imu = imu_sub.recv().await;
    //         let Isometry {
    //             translation: pos,
    //             rotation,
    //         } = robot_base_ref.get_isometry();
    //         let vel = robot_base_ref.get_linear_velocity();
    //         let now = start.elapsed();
    //         writeln!(
    //             data_dump,
    //             "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
    //             imu.acceleration.x,
    //             imu.acceleration.y,
    //             imu.acceleration.z,
    //             imu.angular_velocity.w,
    //             imu.angular_velocity.i,
    //             imu.angular_velocity.j,
    //             imu.angular_velocity.k,
    //             vel.x, vel.y, vel.z,
    //             pos.x, pos.y, pos.z,
    //             rotation.w, rotation.i, rotation.j, rotation.k,
    //             (now - elapsed).as_secs_f32()
    //         ).unwrap();
    //         elapsed = now;
    //     }
    // }, "telemetry-dump");

    apriltag.spawn(context.make_context("apriltag"));
    localizer.spawn(context.make_context("localizer"));
    driver.spawn(context.make_context("driver"));
    pathfinder.spawn(context.make_context("pathfinder"));
    telemetry.spawn(context.make_context("telemetry"));
    costmap.spawn(context.make_context("costmap"));

    context.wait_for_exit().await;
    Ok(())
}
