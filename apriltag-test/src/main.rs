use std::f32::consts::PI;

use apriltag::AprilTagDetector;
use camera::discover_all_cameras;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Quaternion};
use positioning::{PositionFrame, Positioner, OrientationFrame};
use unros_core::{
    anyhow::{self, Context},
    async_run_all, tokio, FinalizedNode, RunOptions,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut positioning = Positioner::default();

    let mut runnables: Vec<_> = discover_all_cameras()
        .context("Failed to discover cameras")?
        .map(|mut camera| {
            let mut apriltag =
                AprilTagDetector::new(1108.4, 1280, 960, camera.image_received_signal().watch());
            apriltag.add_tag(Default::default(), Default::default(), 0.111, 0);

            positioning.add_position_sub(apriltag.tag_detected_signal().subscribe_bounded().map(
                |pose| PositionFrame {
                    position: Point3::new(pose.position.x as f32, pose.position.y as f32, pose.position.y as f32),
                    variance: Matrix3::identity() * 0.05,
                },
            ));

            positioning.add_orientation_sub(apriltag.tag_detected_signal().subscribe_bounded().map(
                |pose| OrientationFrame {
                    orientation: UnitQuaternion::<f32>::new_normalize(Quaternion::new(pose.orientation.w as f32, pose.orientation.i as f32, pose.orientation.j as f32, pose.orientation.k as f32)),
                    variance: Matrix3::identity() * 1.0,
                },
            ));

            let mut pos_sub = positioning.get_position_signal().watch();
            let mut orientation_sub = positioning.get_orientation_signal().watch();

            tokio::spawn(async move {
                loop {
                    let pos = pos_sub.wait_for_change().await;
                    let (mut r, mut p, mut y) = orientation_sub.get().await.euler_angles();
                    r *= 180.0 / PI;
                    p *= 180.0 / PI;
                    y *= 180.0 / PI;
                    println!("({:.2}, {:.2}, {:.2}) roll: {r:.0} pitch: {p:.0} yaw: {y:.0}", pos.x, pos.y, pos.z);
                }
            });

            [FinalizedNode::from(camera), FinalizedNode::from(apriltag)]
        })
        .flatten()
        .collect();

    runnables.push(positioning.into());

    let run_options = RunOptions {
        ..Default::default()
    };
    async_run_all(runnables, run_options).await
}
