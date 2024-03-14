use std::{
    io::Write,
    net::SocketAddrV4,
    str::FromStr,
    sync::Arc,
    time::{Duration, Instant},
};

use apriltag::{AprilTagDetector, PoseObservation};
use camera::{discover_all_cameras, Camera};
use costmap::local::LocalCostmap;
use fxhash::FxBuildHasher;
use localization::{
    frames::{IMUFrame, OrientationFrame, PositionFrame},
    Localizer,
};
use nalgebra::{Isometry, Point3};
use navigator::{pathfinders::DirectPathfinder, DifferentialDriver};
#[cfg(unix)]
use realsense::{discover_all_realsense, PointCloud};
use rig::Robot;
use telemetry::Telemetry;
use unros::{
    anyhow,
    log::info,
    logging::dump::{DataDump, VideoDataDump},
    pubsub::Subscriber,
    rayon, Application, Node,
};

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera"])?;
    let camera_element = elements.remove("camera").unwrap();
    let robot_base_ref = robot_base.get_ref();

    let costmap = LocalCostmap::new(80, 0.05, 0.01, robot_base.get_ref());

    #[cfg(unix)]
    let costmap = costmap;

    let camera_count = discover_all_cameras()?
        .map(|mut x| x.get_intrinsics().ignore_drop())
        .count();
    info!("Discovered {camera_count} cameras");
    let mut camera = Camera::new(0)?;

    #[cfg(unix)]
    let mut realsense_camera = {
        use costmap::Points;
        let mut camera = discover_all_realsense()?
            .next()
            .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

        camera.set_robot_element_ref(camera_element.get_ref());
        let camera_element_ref = camera_element.get_ref();
        camera.accept_cloud_received_sub(costmap.create_points_sub().map(move |x: PointCloud| {
            Points {
                points: x.iter().map(|x| x.0),
                robot_element: camera_element_ref.clone(),
            }
        }));

        camera
    };

    let costmap = Arc::new(costmap);

    let telemetry = Telemetry::new(
        SocketAddrV4::from_str("10.8.0.6:43721").unwrap(),
        1920,
        1200,
        20,
    )
    .await?;
    camera.accept_image_received_sub(telemetry.create_image_subscription());

    // let costmap_ref = costmap.clone();

    // let mut costmap_writer = VideoDataDump::new_display(80, 80, 24)?;
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

    let localizer = Localizer::new(robot_base, 0.4);

    apriltag.accept_tag_detected_sub(localizer.create_position_sub().map(
        |pose: PoseObservation| PositionFrame {
            position: nalgebra::convert(Point3::from(pose.pose.translation.vector)),
            variance: 0.1,
            robot_element: pose.robot_element,
        },
    ));

    apriltag.accept_tag_detected_sub(localizer.create_orientation_sub().map(
        |pose: PoseObservation| OrientationFrame {
            orientation: nalgebra::convert(pose.pose.rotation),
            variance: 0.1,
            robot_element: pose.robot_element,
        },
    ));

    #[cfg(unix)]
    {
        realsense_camera.accept_image_received_sub(
            apriltag
                .create_image_subscription()
                .set_name("RealSense Apriltag Image"),
        );
        // camera.accept_imu_frame_received_sub(localizer.create_imu_sub().set_name("RealSense IMU"));
    }

    let mut navigator = DirectPathfinder::new(robot_base_ref.clone(), costmap.clone(), 0.5, 0.15);
    let driver = DifferentialDriver::new(robot_base_ref.clone());

    navigator.accept_path_sub(driver.create_path_sub());

    let mut data_dump = DataDump::new_file("motion.csv").await?;
    writeln!(
        data_dump,
        "imu_ax,imu_ay,imu_az,imu_rvw,imu_rvi,imu_rvj,imu_rvk,vx,vy,vz,x,y,z,w,i,j,k,delta"
    )
    .unwrap();
    let mut imu_sub = Subscriber::<IMUFrame>::new(32);
    #[cfg(unix)]
    realsense_camera.accept_imu_frame_received_sub(imu_sub.create_subscription());
    app.add_task(|_| async move {
    let start = Instant::now();
    let mut elapsed = Duration::ZERO;

    loop {
        let imu = imu_sub.recv().await;
        let Isometry {
            translation: pos,
            rotation,
        } = robot_base_ref.get_isometry();
        let vel = robot_base_ref.get_linear_velocity();
        let now = start.elapsed();
        writeln!(
            data_dump,
            "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            imu.acceleration.x,
            imu.acceleration.y,
            imu.acceleration.z,
            imu.angular_velocity.w,
            imu.angular_velocity.i,
            imu.angular_velocity.j,
            imu.angular_velocity.k,
            vel.x, vel.y, vel.z,
            pos.x, pos.y, pos.z,
            rotation.w, rotation.i, rotation.j, rotation.k,
            (now - elapsed).as_secs_f32()
        ).unwrap();
        elapsed = now;
    }
}, "telemetry-dump");

    app.add_node(apriltag);
    app.add_node(localizer);
    app.add_node(driver);
    app.add_node(navigator);
    app.add_node(telemetry);
    app.add_node(camera);
    #[cfg(unix)]
    app.add_node(realsense_camera);

    Ok(app)
}
