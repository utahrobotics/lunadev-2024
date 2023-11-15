use apriltag::AprilTagDetector;
use nalgebra::Vector3;
use navigator::{pid, WaypointDriver};
use positioning::{eskf, OrientationFrame, PositionFrame, Positioner};
use realsense::discover_all_realsense;
use unros_core::{anyhow, async_run_all, default_run_options, tokio};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut camera = discover_all_realsense()?
        .next()
        .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

    let mut camera_imu = camera.imu_frame_received().watch();
    tokio::spawn(async move {
        loop {
            let accel = camera_imu.wait_for_change().await;
            println!("real {}", accel.acceleration);
        }
    });

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

    async_run_all(
        [
            camera.into(),
            apriltag.into(),
            positioning.into(),
            navigator.into(),
        ],
        default_run_options!(),
    )
    .await
}
