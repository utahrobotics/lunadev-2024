use std::sync::atomic::{AtomicUsize, Ordering};

use camera::Camera;
use unros_core::{anyhow, log::info, run_all, Node, RunOptions};

fn main() -> anyhow::Result<()> {
    let frame_count = AtomicUsize::new(0);
    let fps = 20;
    let mut camera = Camera::new(1024, 600, fps, 0);
    camera.set_name("camera_counter".into());
    camera.image_received_signal().connect_to(move |_| {
        let frame_count = frame_count.fetch_add(1, Ordering::Relaxed) + 1;
        if frame_count % (fps as usize) == 0 {
            info!("{frame_count}");
        }
    });
    let run_options = RunOptions {
        ..Default::default()
    };
    run_all([camera], run_options)
}
