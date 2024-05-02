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
    pubsub::Subscriber,
    runtime::MainRuntimeContext,
    setup_logging, ShouldNotDrop,
};

mod actuators;
mod drive;
mod serial;
// mod imu;
mod audio;
mod telemetry;

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    setup_logging!(context);
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "imu01"])?;
    let _camera_element = elements.remove("camera").unwrap();
    let _robot_base_ref = robot_base.get_ref();
    // let imu01 = elements.remove("imu01").unwrap().get_ref();

    let mut cameras: Vec<_> = discover_all_cameras()?
        .filter_map(|mut x| {
            if x.get_camera_name().contains("RealSense") {
                x.ignore_drop();
                None
            } else {
                info!(
                    "Discovered {} at {}",
                    x.get_camera_name(),
                    x.get_camera_uri()
                );
                Some(x)
            }
        })
        .collect();
    info!("Discovered {} cameras", cameras.len());

    let telemetry = Telemetry::new(1280, 720, 20).await?;

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

    match cameras
        .iter()
        .enumerate()
        .filter_map(|(i, cam)| {
            if cam.get_camera_name() == "HD USB CAMERA: HD USB CAMERA" {
                Some(i)
            } else {
                None
            }
        })
        .next()
    {
        Some(i) => {
            let mut teleop_camera = cameras.remove(i);
            teleop_camera.res_x = 1280;
            teleop_camera.res_y = 720;
            teleop_camera
                .image_received_pub()
                .accept_subscription(telemetry.create_image_subscription());
            teleop_camera.spawn(context.make_context("teleop_camera"));
        }
        None => {
            error!("Teleop camera not found");
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
