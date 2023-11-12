use std::sync::atomic::{AtomicUsize, Ordering};

use camera::Camera;
use unros_core::{anyhow, log::info, async_run_all, RunOptions, tokio};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let frame_count = AtomicUsize::new(0);
    let fps = 20;
    let mut camera = Camera::new(1024, 600, fps, 0);
    let mut sub = camera.image_received_signal().subscribe_unbounded();
    tokio::spawn(async move {
        loop {
            sub.recv().await;
            let frame_count = frame_count.fetch_add(1, Ordering::Relaxed) + 1;
            if frame_count % (fps as usize) == 0 {
                info!("{frame_count}");
            }
        }
    });
    let run_options = RunOptions {
        ..Default::default()
    };
    async_run_all([camera.into()], run_options).await
}
