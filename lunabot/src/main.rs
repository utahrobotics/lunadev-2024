use std::{
    io::Write,
    time::{Duration, Instant},
};

use apriltag::AprilTagDetector;
use nalgebra::Vector3;
use navigator::{pid, WaypointDriver};
use positioning::{eskf, OrientationFrame, PositionFrame, Positioner};
use realsense::discover_all_realsense;
use unros_core::{
    anyhow, async_run_all, default_run_options, dump::DataDump, init_logger, tokio,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = default_run_options!();
    init_logger(&run_options)?;

    let mut camera = discover_all_realsense()?
        .next()
        .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

    let mut apriltag =
        AprilTagDetector::new(640.0, 1280, 720, camera.image_received_signal().watch());
    apriltag.add_tag(Default::default(), Default::default(), 0.107, 0);

    let mut positioning = Positioner::default();
    positioning.add_position_sub(
        apriltag
            .tag_detected_signal()
            .subscribe_unbounded()
            .map(|tag| PositionFrame {
                position: tag.position,
                variance: eskf::ESKF::variance_from_element(0.05),
            }),
    );
    positioning.add_orientation_sub(apriltag.tag_detected_signal().subscribe_unbounded().map(
        |tag| OrientationFrame {
            orientation: tag.orientation,
            variance: eskf::ESKF::variance_from_element(0.05),
        },
    ));
    positioning.add_imu_sub(
        camera
            .imu_frame_received()
            .subscribe_unbounded()
            .map(|imu| positioning::IMUFrame {
                acceleration: Vector3::new(
                    imu.acceleration.x as f64,
                    imu.acceleration.y as f64,
                    imu.acceleration.z as f64,
                ),
                angular_velocity: Vector3::new(
                    imu.angular_velocity.x as f64,
                    imu.angular_velocity.y as f64,
                    imu.angular_velocity.z as f64,
                ),
                acceleration_variance: Some(Vector3::new(0.01, 0.01, 0.01)),
                angular_velocity_variance: Some(Vector3::new(0.01, 0.01, 0.01)),
            }),
    );
    // let mut pos = positioning.get_position_signal().watch();
    // tokio::spawn(async move {
    //     loop {
    //         let pos = pos.wait_for_change().await;
    //         println!("{:.2}, {:.2}, {:.2}", pos.x, pos.y, pos.z);
    //     }
    // });

    let mut pid = pid::Pid::new(0.0, 100.0);
    pid.p(1.0, 100.0).i(1.0, 100.0).d(1.0, 100.0);
    let mut navigator = WaypointDriver::new(
        positioning.get_position_signal().watch(),
        positioning.get_velocity_signal().watch(),
        positioning.get_orientation_signal().watch(),
        pid,
    );

    let mut data_dump = DataDump::new_file("data.csv").await?;
    writeln!(
        data_dump,
        "imu_ax,imu_ay,imu_az,imu_rvx,imu_rvy,imu_rvz,vx,vy,vz,x,y,z,roll,pitch,yaw,delta"
    )
    .unwrap();
    let mut data_sub = camera
        .imu_frame_received()
        .watch()
        .zip(positioning.get_velocity_signal().watch())
        .zip(positioning.get_position_signal().watch())
        .zip(positioning.get_orientation_signal().watch());
    tokio::spawn(async move {
        let start = Instant::now();
        let mut elapsed = Duration::ZERO;

        loop {
            let (((imu, vel), pos), orientation) = data_sub.wait_for_change().await;
            let (roll, pitch, yaw) = orientation.euler_angles();
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
