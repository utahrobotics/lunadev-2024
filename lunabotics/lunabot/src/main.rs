use std::sync::{
    atomic::{AtomicU8, Ordering},
    Arc,
};

// use apriltag::{AprilTagDetector, PoseObservation};
use camera::discover_all_cameras;
use fxhash::FxBuildHasher;
use localization::{
    engines::window::{DefaultWindowConfig, WindowLocalizer},
    Localizer,
};
use rig::Robot;
use telemetry::Telemetry;
use unros::{
    anyhow,
    node::{AsyncNode, SyncNode},
    pubsub::{subs::Subscription, Subscriber},
    runtime::MainRuntimeContext,
    setup_logging,
};

use crate::audio::init_buzz;

mod actuators;
mod drive;
mod serial;
// mod imu;
mod audio;
mod telemetry;

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    setup_logging!(context);
    init_buzz()?;

    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "imu01"])?;
    let _camera_element = elements.remove("camera").unwrap();
    let _robot_base_ref = robot_base.get_ref();
    // let imu01 = elements.remove("imu01").unwrap().get_ref();
    let camera_index = Arc::new(AtomicU8::new(0));

    let mut telemetry = Telemetry::new(1280, 720, 20, camera_index.clone()).await?;

    let camera_count = discover_all_cameras()?
        .filter(|x| !x.get_camera_name().contains("RealSense"))
        .enumerate()
        .map(|(i, cam)| {
            info!(
                "Discovered {} at {}",
                cam.get_camera_name(),
                cam.get_camera_uri()
            );
            let camera_index = camera_index.clone();
            cam.image_received_pub().accept_subscription(
                telemetry
                    .create_image_subscription()
                    .filter_map(move |img| {
                        if camera_index.load(Ordering::Relaxed) == i as u8 {
                            Some(img)
                        } else {
                            None
                        }
                    }),
            );
            cam.spawn(context.make_context(format!("teleop-camera-{i}")));
        })
        .count();
    info!("Discovered {camera_count} cameras");
    telemetry.camera_count = camera_count as u8;

    let arm_sub = Subscriber::new(8);
    telemetry
        .arm_pub()
        .accept_subscription(arm_sub.create_subscription());
    arm_sub
        .into_logger(|x| format!("{x:?}"), "arms.logs", &context)
        .await?;

    match serial::connect_to_serial() {
        Ok((arms, drive)) => {
            telemetry
                .steering_pub()
                .accept_subscription(drive.get_steering_sub());

            telemetry.arm_pub().accept_subscription(arms.get_arm_sub());
            drive.spawn(context.make_context("drive"));
            arms.spawn(context.make_context("arms"));
        }
        Err(e) => {
            error!("{e}");
        }
    }

    let localizer: Localizer<f32, WindowLocalizer<f32, _, _, _, _>> =
        Localizer::new(robot_base, DefaultWindowConfig::default());

    // let mut apriltag = AprilTagDetector::new(640.0, 1280, 720, camera_element.get_ref());
    // apriltag.add_tag(Default::default(), Default::default(), 0.134, 0);

    // apriltag
    //     .tag_detected_pub()
    //     .accept_subscription(
    //         localizer
    //             .create_position_sub()
    //             .map(|pose: PoseObservation| PositionFrame {
    //                 position: nalgebra::convert(Point3::from(pose.pose.translation.vector)),
    //                 variance: 0.1,
    //                 robot_element: pose.robot_element,
    //             }),
    //     );

    // apriltag
    //     .tag_detected_pub()
    //     .accept_subscription(
    //         localizer
    //             .create_orientation_sub()
    //             .map(|pose: PoseObservation| OrientationFrame {
    //                 orientation: nalgebra::convert(pose.pose.rotation),
    //                 variance: 0.1,
    //                 robot_element: pose.robot_element,
    //             }),
    //     );

    // apriltag.spawn(context.make_context("apriltag"));

    // let imu01 = open_imu(
    //     "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00",
    //     imu01,
    // )
    // .await;
    // imu01
    //     .msg_received_pub()
    //     .accept_subscription(localizer.create_imu_sub().set_name("imu01"));

    localizer.spawn(context.make_context("localizer"));
    telemetry.spawn(context.make_context("telemetry"));

    context.wait_for_exit().await;
    Ok(())
}
