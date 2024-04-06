use std::{
    io::Write,
    net::SocketAddrV4,
    str::FromStr,
    time::{Duration, Instant},
};

use apriltag::{AprilTagDetector, PoseObservation};
use camera::discover_all_cameras;
use costmap::CostmapGenerator;
use fxhash::FxBuildHasher;
use imu::open_imu;
use localization::{
    frames::{IMUFrame, OrientationFrame, PositionFrame},
    Localizer,
};
use nalgebra::{Isometry, Point3};
use navigator::{pathfinding::Pathfinder, DifferentialDriver};
#[cfg(unix)]
use realsense::{discover_all_realsense, PointCloud};
use rig::Robot;
use telemetry::Telemetry;
use unros::{
    anyhow,
    log::info,
    logging::dump::DataDump,
    pubsub::{subs::Subscription, Subscriber},
    Application, Node,
};

use crate::{actuators::Arms, drive::Drive};

mod actuators;
mod drive;
mod imu;
mod telemetry;

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "imu01"])?;
    let camera_element = elements.remove("camera").unwrap();
    let robot_base_ref = robot_base.get_ref();
    let imu01 = elements.remove("imu01").unwrap().get_ref();

    let costmap = CostmapGenerator::new(10);

    #[cfg(unix)]
    let costmap = costmap;

    let mut cameras: Vec<_> = discover_all_cameras()?
        .map(|mut x| {
            info!(
                "Discovered {} at {}",
                x.get_camera_name(),
                x.get_camera_uri()
            );
            x.get_intrinsics().ignore_drop();
            x
        })
        .collect();
    info!("Discovered {} cameras", cameras.len());

    #[cfg(unix)]
    let realsense_camera =
        {
            use costmap::Points;
            let mut camera = discover_all_realsense()?
                .next()
                .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

            camera.set_robot_element_ref(camera_element.get_ref());
            let camera_element_ref = camera_element.get_ref();
            camera
                .cloud_received_pub()
                .accept_subscription(costmap.create_points_sub(0.1).map(move |x: PointCloud| {
                    Points {
                        points: x.iter().map(|x| x.0),
                        robot_element: camera_element_ref.clone(),
                    }
                }));

            camera
        };

    let telemetry = Telemetry::new(
        SocketAddrV4::from_str("10.8.0.6:43721").unwrap(),
        1280,
        720,
        20,
    )
    .await?;

    let arm_sub = Subscriber::new(8);
    telemetry
        .arm_pub()
        .accept_subscription(arm_sub.create_subscription());
    arm_sub
        .into_logger(|x| format!("{x:?}"), "arms.logs")
        .await?;

    let drive = Drive::new()?;
    telemetry
        .steering_pub()
        .accept_subscription(drive.get_steering_sub());

    let arms = Arms::new()?;
    telemetry.arm_pub().accept_subscription(arms.get_arm_sub());

    let mut teleop_camera = cameras.remove(0);
    teleop_camera.res_x = 1280;
    teleop_camera.res_y = 720;
    teleop_camera
        .image_received_pub()
        .accept_subscription(telemetry.create_image_subscription());

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

    let localizer = Localizer::new(robot_base, 0.4);

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

    let imu01 = open_imu(
        "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00",
        imu01,
    )
    .await;
    imu01
        .msg_received_pub()
        .accept_subscription(localizer.create_imu_sub().set_name("imu01"));

    #[cfg(unix)]
    {
        realsense_camera.image_received_pub().accept_subscription(
            apriltag
                .create_image_subscription()
                .set_name("RealSense Apriltag Image"),
        );
        realsense_camera
            .imu_frame_received_pub()
            .accept_subscription(localizer.create_imu_sub().set_name("RealSense IMU"));
    }

    let pathfinder: Pathfinder =
        Pathfinder::new_with_engine(Default::default(), robot_base_ref.clone());
    let driver = DifferentialDriver::new(robot_base_ref.clone());

    // pathfinder.get_path_pub().accept_subscription(driver.create_path_sub());

    let mut data_dump = DataDump::new_file("motion.csv").await?;
    writeln!(
        data_dump,
        "imu_ax,imu_ay,imu_az,imu_rvw,imu_rvi,imu_rvj,imu_rvk,vx,vy,vz,x,y,z,w,i,j,k,delta"
    )
    .unwrap();
    let imu_sub = Subscriber::<IMUFrame>::new(32);
    #[cfg(unix)]
    realsense_camera
        .imu_frame_received_pub()
        .accept_subscription(imu_sub.create_subscription());
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
    app.add_node(pathfinder);
    app.add_node(telemetry);
    app.add_node(teleop_camera);
    // app.add_node(imu01);
    app.add_node(drive);
    app.add_node(arms);
    app.add_node(costmap);
    #[cfg(unix)]
    app.add_node(realsense_camera);

    Ok(app)
}
