use apriltag::AprilTagDetector;
use camera::discover_all_cameras;
use fxhash::FxBuildHasher;
use rig::Robot;
use unros_core::{
    anyhow::{self, Context},
    async_run_all, default_run_options,
    logging::init_logger,
    pubsub::Subscriber,
    tokio, FnNode,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut run_options = default_run_options!();
    run_options.enable_console_subscriber = false;
    init_logger(&run_options)?;
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, _) = rig.destructure::<FxBuildHasher>(["camera"])?;
    let camera_element = elements.remove("camera").unwrap();
    let mut apriltag = AprilTagDetector::new(1108.4, 1280, 960, camera_element.get_ref());
    let mut camera = discover_all_cameras()
        .context("Failed to discover cameras")?
        .next()
        .context("No camera found")?;
    camera.accept_image_received_sub(apriltag.create_image_subscription());
    let mut pose_sub = Subscriber::new(1);
    apriltag.add_tag(Default::default(), Default::default(), 0.12, 0);
    apriltag.accept_tag_detected_sub(pose_sub.create_subscription());
    let pose_sub = FnNode::new(|_| async move {
        loop {
            let pose = pose_sub.recv().await;
            println!("{pose}");
        }
    });

    async_run_all(
        [camera.into(), apriltag.into(), pose_sub.into()],
        run_options,
    )
    .await
}
