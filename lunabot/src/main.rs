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
use realsense::discover_all_realsense;
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::{dump::DataDump, init_logger, rate::RateLogger},
    tokio,
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
    apriltag.add_tag(Default::default(), Default::default(), 0.107, 0);
    let mut img_sub = camera.image_received_signal().watch();
    tokio::spawn(async move {
        let mut rate_logger = RateLogger::default();
        loop {
            img_sub.wait_for_change().await;
            rate_logger.increment();
        }
    });

    let mut positioning = Localizer::new(robot_base);
    positioning.add_position_sub(
        apriltag
            .tag_detected_signal()
            .subscribe_unbounded()
            .map(|tag| PositionFrame {
                position: tag.position,
                variance: eskf::ESKF::variance_from_element(0.05),
                robot_element: tag.robot_element,
            }),
    );
    positioning.add_orientation_sub(apriltag.tag_detected_signal().subscribe_unbounded().map(
        |tag| OrientationFrame {
            orientation: tag.orientation,
            variance: eskf::ESKF::variance_from_element(0.05),
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

    let mut data_dump = DataDump::new_file("data.csv").await?;
    writeln!(
        data_dump,
        "imu_ax,imu_ay,imu_az,imu_rvx,imu_rvy,imu_rvz,vx,vy,vz,x,y,z,roll,pitch,yaw,delta"
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
            let (roll, pitch, yaw) = rotation.euler_angles();
            let now = start.elapsed();
            writeln!(
                data_dump,
                "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                imu.acceleration.x,
                imu.acceleration.y,
                imu.acceleration.z,
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
                vel.x, vel.y, vel.z,
                pos.x, pos.y, pos.z,
                roll, pitch, yaw,
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
