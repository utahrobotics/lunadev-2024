use std::sync::atomic::{AtomicUsize, Ordering};

use camera::discover_all_cameras;
use unros_core::{
    anyhow::{self, Context},
    default_run_options,
    log::info,
    pubsub::Subscriber,
    start_unros_runtime, tokio, Application,
};

fn main() -> anyhow::Result<()> {
    start_unros_runtime(
        || async {
            let fps = 20;
            let mut grp = Application::default();
            discover_all_cameras()
                .context("Failed to discover cameras")?
                .for_each(|mut camera| {
                    let mut sub = Subscriber::new(8);
                    camera.accept_image_received_sub(sub.create_subscription());
                    let frame_count = AtomicUsize::new(0);
                    let idx = camera.camera_index;
                    tokio::spawn(async move {
                        loop {
                            sub.recv().await;
                            let frame_count = frame_count.fetch_add(1, Ordering::Relaxed) + 1;
                            if frame_count % (fps as usize) == 0 {
                                info!("idx: {idx} -> {frame_count}");
                            }
                        }
                    });
                    grp.add_node(camera);
                });

            grp.run().await
        },
        default_run_options!(),
    )
}
