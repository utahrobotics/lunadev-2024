use std::f32::consts::PI;

use realsense::discover_all_realsense;
use unros_core::{anyhow, async_run_all, tokio, FinalizedNode, default_run_options};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cameras = discover_all_realsense()?;

    // let frame_count = AtomicUsize::new(0);
    // let frame_count: &_ = Box::leak(Box::new(frame_count));
    async_run_all(
        cameras.map(|mut x| {
            // let mut img_sub = x.image_received_signal().subscribe_unbounded();
            let mut accel_sub = x.acceleration_received_signal().watch();
            let mut ang_sub = x.angular_velocity_received_signal().watch();
            tokio::spawn(async move {
                loop {
                    // let img = img_sub.recv().await;
                    // let frame_count = frame_count.fetch_add(1, Ordering::Relaxed) + 1;
                    // if frame_count % 20 == 0 {
                    //     img.save(format!("{frame_count}.png")).unwrap();
                    //     info!("{frame_count}");
                    // }
                    tokio::select! {
                        ang = ang_sub.wait_for_change() => {
                            let (mut p, mut y, mut r) = (ang.x, ang.y, ang.z);
                            r *= 180.0 / PI;
                            p *= 180.0 / PI;
                            y *= 180.0 / PI;
                            println!("roll: {r:.0} pitch: {p:.0} yaw: {y:.0}");
                        }
                        accel = accel_sub.wait_for_change() => {
                            println!("({:.2}, {:.2}, {:.2})", accel.x, accel.y, accel.z);
                        }
                    }
                }
            });
            FinalizedNode::from(x)
        }),
        default_run_options!(),
    )
    .await
}
