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
    // Realsense: 1108.4
    // HD Global Shutter: 506.0
    // Wide Angle:
    let mut apriltag = AprilTagDetector::new(506.0, 1920, 1200, camera_element.get_ref());
    let camera = discover_all_cameras()
        .context("Failed to discover cameras")?
        .next()
        .context("Failed to discover cameras")?;
    camera
        .image_received_pub()
        .accept_subscription(apriltag.create_image_subscription());
    let pose_sub = Subscriber::new(1);
    apriltag.add_tag(Default::default(), Default::default(), 0.137, 0);
    apriltag
        .tag_detected_pub()
        .accept_subscription(pose_sub.create_subscription());
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
