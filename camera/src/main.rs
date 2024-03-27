use std::sync::atomic::{AtomicUsize, Ordering};

use camera::discover_all_cameras;
use unros::{
    anyhow::{self, Context},
    log::info,
    pubsub::Subscriber,
    tokio, Application,
};

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let fps = 20;
    discover_all_cameras()
        .context("Failed to discover cameras")?
        .for_each(|camera| {
            let mut sub = Subscriber::new(8);
            camera
                .image_received_pub()
                .accept_subscription(sub.create_subscription());
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
            app.add_node(camera);
        });

    Ok(app)
}
