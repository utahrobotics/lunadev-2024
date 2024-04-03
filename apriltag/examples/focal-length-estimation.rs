use std::time::Duration;

use apriltag::AprilTagDetector;
use camera::discover_all_cameras;
use fxhash::FxBuildHasher;
use rig::Robot;
use unros::{
    anyhow::{self, Context},
    pubsub::{Publisher, Subscriber},
    setup_logging, tokio, Application, Node,
};

const TAG_DISTANCE: f64 = 2.98;
const TAG_WIDTH: f64 = 0.137;
const TAG_ID: usize = 0;
const IMAGE_WIDTH: u32 = 1920;
const IMAGE_HEIGHT: u32 = 1200;
const CAMERA_INDEX: usize = 0;

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, _) = rig.destructure::<FxBuildHasher>(["camera"])?;
    let camera_element = elements.remove("camera").unwrap();
    let mut cameras: Vec<_> = discover_all_cameras()
        .context("Failed to discover cameras")?
        .map(|mut x| {
            x.get_intrinsics().ignore_drop();
            x
        })
        .collect();
    let camera = cameras.remove(CAMERA_INDEX);
    let camera_sub = Subscriber::new(1);
    camera
        .image_received_pub()
        .accept_subscription(camera_sub.create_subscription());

    app.add_task(
        move |context| async move {
            setup_logging!(context);
            let pose_sub = Subscriber::new(1);
            let mut length = IMAGE_HEIGHT.max(IMAGE_WIDTH) as f64 / 2.0;
            let mut close_enoughs = 0usize;

            loop {
                info!("Current length: {length:.2}");
                let img_pub = Publisher::default();
                let mut apriltag = AprilTagDetector::new(
                    length,
                    IMAGE_WIDTH,
                    IMAGE_HEIGHT,
                    camera_element.get_ref(),
                );
                img_pub.accept_subscription(apriltag.create_image_subscription());

                apriltag.add_tag(Default::default(), Default::default(), TAG_WIDTH, TAG_ID);
                apriltag
                    .tag_detected_pub()
                    .accept_subscription(pose_sub.create_subscription());
                context.spawn_node(apriltag);

                let img_fut = async {
                    loop {
                        img_pub.set(camera_sub.recv().await);
                    }
                };
                let mut distance = 0.0;
                let mut observations = 0usize;
                let pose_fut = async {
                    loop {
                        let pose = pose_sub.recv().await;
                        distance += pose.pose.translation.vector.magnitude();
                        observations += 1;
                    }
                };
                tokio::select! {
                    _ = img_fut => unreachable!(),
                    _ = pose_fut => unreachable!(),
                    _ = tokio::time::sleep(Duration::from_secs(3)) => {}
                }
                if observations == 0 {
                    error!("Received no observations!");
                    continue;
                }
                distance /= observations as f64;
                let new_length = length * TAG_DISTANCE / distance;

                if (new_length - length).abs() < 1.0 {
                    close_enoughs += 1;

                    if close_enoughs >= 5 {
                        info!("Final length: {new_length:.2}");
                        break Ok(());
                    }
                } else {
                    close_enoughs = 0;
                }

                length = new_length;
            }
        },
        "estimator",
    );

    app.add_node(camera);
    Ok(app)
}
