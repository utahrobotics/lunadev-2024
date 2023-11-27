use std::{
    io::Write,
    time::{Duration, Instant},
};

use apriltag::AprilTagDetector;
use fxhash::FxBuildHasher;
use nalgebra::Isometry;
// use navigator::{pid, WaypointDriver};
use localization::{eskf, Localizer, OrientationFrame, PositionFrame};
use navigator::{pid, WaypointDriver};
use realsense::{discover_all_realsense, PointCloud};
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::{dump::DataDump, init_logger, rate::RateLogger},
    tokio, tokio_rayon,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera"])?;
    let camera_element = elements.remove("camera").unwrap();
    let robot_base_ref = robot_base.get_ref();

    let run_options = default_run_options!();
    init_logger(&run_options)?;

    let mut camera = discover_all_realsense()?
        .next()
        .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

    camera.set_robot_element_ref(camera_element.get_ref());

    let mut apriltag = AprilTagDetector::new(
        640.0,
        1280,
        720,
        camera.image_received_signal().watch(),
        camera_element.get_ref(),
    );
    apriltag.add_tag(Default::default(), Default::default(), 0.134, 0);
    // let mut pc_sub = camera.point_cloud_received_signal().watch();
    // // let mut data_dump = DataDump::new_file("points.dat").await?;
    // tokio::spawn(async move {
    //     // let mut rate_logger = RateLogger::default();
    //     let mut i = 0;
    //     loop {
    //         let PointCloud { points } = pc_sub.wait_for_change().await;
    //         tokio_rayon::spawn(move || {
    //             let mut header = las::Builder::default();
    //             header.point_format = las::point::Format {
    //                 has_color: true,
    //                 ..Default::default()
    //             };
    //             let mut writer = las::Writer::from_path(format!("{i}.las"), header.into_header().unwrap()).unwrap();
    //             use las::Write;
    //             for (point, color) in points.iter() {
    //                 let mut point = las::Point { x: point.x as f64, y: point.y as f64, z: point.z as f64, ..Default::default() };
    //                 point.color = Some(las::Color { red: color.0[0] as u16 * 255, green: color.0[1] as u16 * 255, blue: color.0[2] as u16 * 255 });
    //                 writer.write(point).unwrap();
    //             }
    //         }).await;
    //         i += 1;
    //         // rate_logger.increment();
    //         // data_dump.write_all(&points.len().to_be_bytes()).unwrap();
    //         // for point in points.iter() {
    //         //     data_dump.write_all(&point.x.to_be_bytes()).unwrap();
    //         //     data_dump.write_all(&point.y.to_be_bytes()).unwrap();
    //         //     data_dump.write_all(&point.z.to_be_bytes()).unwrap();
    //         // }
    //     }
    // });

    let mut positioning = Localizer::new(robot_base);
    positioning.add_position_sub(
        apriltag
            .tag_detected_signal()
            .subscribe_unbounded()
            .map(|tag| PositionFrame {
                position: tag.position,
                velocity: tag.velocity,
                variance: eskf::ESKF::variance_from_element(0.03),
                robot_element: tag.robot_element,
            }),
    );
    positioning.add_orientation_sub(apriltag.tag_detected_signal().subscribe_unbounded().map(
        |tag| OrientationFrame {
            orientation: tag.orientation,
            variance: eskf::ESKF::variance_from_element(0.2),
            robot_element: tag.robot_element,
        },
    ));
    positioning.add_imu_sub(camera.imu_frame_received().subscribe_unbounded());
    // let mut pos = positioning.get_position_signal().watch();
    // tokio::spawn(async move {
    //     loop {
    //         let pos = pos.wait_for_change().await;
    //         println!("{:.2}, {:.2}, {:.2}", pos.x, pos.y, pos.z);
    //     }
    // });

    let mut pid = pid::Pid::new(0.0, 100.0);
    pid.p(1.0, 100.0).i(1.0, 100.0).d(1.0, 100.0);
    let navigator = WaypointDriver::new(robot_base_ref.clone(), pid);

    let mut data_dump = DataDump::new_file("motion.csv").await?;
    writeln!(
        data_dump,
        "imu_ax,imu_ay,imu_az,imu_rvx,imu_rvy,imu_rvz,vx,vy,vz,x,y,z,w,i,j,k,delta"
    )
    .unwrap();
    let mut imu_sub = camera.imu_frame_received().watch();
    tokio::spawn(async move {
        let start = Instant::now();
        let mut elapsed = Duration::ZERO;

        loop {
            let imu = imu_sub.wait_for_change().await;
            let Isometry {
                translation: pos,
                rotation,
            } = robot_base_ref.get_isometry();
            let vel = robot_base_ref.get_linear_velocity();
            let now = start.elapsed();
            writeln!(
                data_dump,
                "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                imu.acceleration.x,
                imu.acceleration.y,
                imu.acceleration.z,
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
                vel.x, vel.y, vel.z,
                pos.x, pos.y, pos.z,
                rotation.w, rotation.i, rotation.j, rotation.k,
                (now - elapsed).as_secs_f32()
            ).unwrap();
            elapsed = now;
        }
    });

    async_run_all(
        [
            camera.into(),
            apriltag.into(),
            positioning.into(),
            navigator.into(),
        ],
        run_options,
    )
    .await
}
