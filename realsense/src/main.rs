use std::f32::consts::PI;

use realsense::discover_all_realsense;
use unros_core::{anyhow, async_run_all, default_run_options, tokio, FinalizedNode};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cameras = discover_all_realsense()?;

    // let frame_count = AtomicUsize::new(0);
    // let frame_count: &_ = Box::leak(Box::new(frame_count));
    async_run_all(
        cameras.map(|mut x| {
            // let mut img_sub = x.image_received_signal().subscribe_unbounded();
            let mut imu_sub = x.imu_frame_received().watch();
            tokio::spawn(async move {
                loop {
                    let imu = imu_sub.wait_for_change().await;
                    println!(
                        "ang_vel: {} accel: {}",
                        imu.angular_velocity / PI * 180.0,
                        imu.acceleration
                    );
                }
            });
            FinalizedNode::from(x)
        }),
        default_run_options!(),
    )
    .await
}
