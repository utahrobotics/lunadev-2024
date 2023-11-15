use std::net::{Ipv4Addr, SocketAddrV4};

use apriltag::AprilTagDetector;
use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use navigator::{pid, WaypointDriver};
use positioning::{eskf, OrientationFrame, PositionFrame, Positioner};
use realsense::discover_all_realsense;
use unros_core::{anyhow, async_run_all, default_run_options, log::info, tokio};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut camera = discover_all_realsense()?
        .next()
        .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

    let mut apriltag =
        AprilTagDetector::new(1000.0, 1280, 720, camera.image_received_signal().watch());
    apriltag.add_tag(Default::default(), Default::default(), 0.2, 0);

    let mut positioning = Positioner::default();
    positioning.add_position_sub(
        apriltag
            .tag_detected_signal()
            .subscribe_unbounded()
            .map(|tag| PositionFrame {
                position: Point3::new(
                    tag.position.x as f32,
                    tag.position.y as f32,
                    tag.position.z as f32,
                ),
                variance: eskf::ESKF::variance_from_element(0.05),
            }),
    );
    positioning.add_orientation_sub(apriltag.tag_detected_signal().subscribe_unbounded().map(
        |tag| OrientationFrame {
            orientation: UnitQuaternion::new_normalize(Quaternion::new(
                tag.orientation.w as f32,
                tag.orientation.i as f32,
                tag.orientation.j as f32,
                tag.orientation.k as f32,
            )),
            variance: eskf::ESKF::variance_from_element(0.05),
        },
    ));
    // positioning.add_imu_sub(camera.)

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
            positioning.into(),
            navigator.into(),
            apriltag.into(),
        ],
        default_run_options!(),
    )
    .await
}
