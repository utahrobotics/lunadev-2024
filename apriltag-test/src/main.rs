use apriltag::AprilTagDetector;
use camera::discover_all_cameras;
use unros_core::{
    anyhow::{self, Context},
    async_run_all, tokio, FinalizedNode, default_run_options,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let runnables = discover_all_cameras()
        .context("Failed to discover cameras")?
        .map(|mut camera| {
            let mut apriltag =
                AprilTagDetector::new(1108.4, 1280, 960, camera.image_received_signal().watch());
            apriltag.add_tag(Default::default(), Default::default(), 0.103, 0);

            let mut sub = apriltag.tag_detected_signal().subscribe_unbounded();

            tokio::spawn(async move {
                loop {
                    let pose = sub.recv().await;
                    println!("{pose}");
                }
            });

            FinalizedNode::from(camera) + FinalizedNode::from(apriltag)
        });

    async_run_all(runnables, default_run_options!()).await
}
