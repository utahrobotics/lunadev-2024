use std::sync::atomic::{AtomicUsize, Ordering};

use realsense::discover_all_realsense;
use unros_core::{anyhow, log::info, run_all, FinalizedNode, RunOptions, Signal};

fn main() -> anyhow::Result<()> {
    let cameras = discover_all_realsense()?;

    let frame_count = AtomicUsize::new(0);
    let frame_count: &_ = Box::leak(Box::new(frame_count));
    let run_options = RunOptions {
        ..Default::default()
    };
    run_all(
        cameras.map(|mut x| {
            x.image_received_signal().connect_to(|img| {
                let frame_count = frame_count.fetch_add(1, Ordering::Relaxed) + 1;
                if frame_count % 20 == 0 {
                    img.save(format!("{frame_count}.png")).unwrap();
                    info!("{frame_count}");
                }
            });
            FinalizedNode::from(x)
        }),
        // [camera.into()],
        run_options,
    )
}
