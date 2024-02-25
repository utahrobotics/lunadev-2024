use apriltag::AprilTagDetector;
use camera::discover_all_cameras;
use fxhash::FxBuildHasher;
use rig::Robot;
use unros::{
    anyhow::{self, Context},
    pubsub::Subscriber,
    Application,
};

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
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
    app.add_task(
        |_| async move {
            loop {
                let pose = pose_sub.recv().await;
                println!("{pose}");
            }
        },
        "pose",
    );

    app.add_node(camera);
    app.add_node(apriltag);
    Ok(app)
}
