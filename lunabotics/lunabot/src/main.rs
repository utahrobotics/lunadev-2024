use std::sync::Arc;

use camera::discover_all_cameras;
// use apriltag::{AprilTagDetector, PoseObservation};
use fxhash::FxBuildHasher;
use image::DynamicImage;
#[cfg(unix)]
use realsense::discover_all_realsense;
// use localization::{
//     engines::window::{DefaultWindowConfig, WindowLocalizer},
//     Localizer,
// };
use rig::Robot;
use telemetry::Telemetry;
use unros::{
    anyhow,
    node::{AsyncNode, SyncNode},
    pubsub::{subs::Subscription, Subscriber},
    runtime::MainRuntimeContext,
    setup_logging, ShouldNotDrop,
};

use crate::audio::init_buzz;

mod actuators;
mod drive;
mod serial;
// mod imu;
mod audio;
// mod mosaic;
mod telemetry;

const MAX_CAMERA_COUNT: usize = 5;
const ROW_LENGTH: usize = 3;

const CAMERA_WIDTH: u32 = 640;
const CAMERA_HEIGHT: u32 = 360;
const ROW_DATA_LENGTH: usize = CAMERA_WIDTH as usize * 3;
const EMPTY_ROW: [u8; ROW_DATA_LENGTH] = [0; ROW_DATA_LENGTH];

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    setup_logging!(context);
    init_buzz();

    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "imu01"])?;
    let _camera_element = elements.remove("camera").unwrap();
    let _robot_base_ref = robot_base.get_ref();

    let mut camera_subs = vec![];

    for mut cam in discover_all_cameras()?.filter_map(|mut cam| {
        if cam.get_camera_name().contains("RealSense") {
            cam.ignore_drop();
            None
        } else {
            Some(cam)
        }
    }) {
        info!(
            "Discovered {} at {}",
            cam.get_camera_name(),
            cam.get_camera_uri()
        );
        cam.res_x = CAMERA_WIDTH;
        cam.res_y = CAMERA_HEIGHT;
        cam.fps = 20;
        let subscriber = Subscriber::new(1);
        cam.image_received_pub().accept_subscription(
            subscriber
                .create_subscription()
                .map(|img: Arc<DynamicImage>| img.to_rgb8()),
        );
        let context = context.make_context(cam.get_camera_name());
        cam.spawn(context);
        if let Ok(sub) = subscriber
        .into_watch_or_closed()
        .await {
            camera_subs.push(sub);
        }
    }

    #[cfg(unix)]
    for mut cam in discover_all_realsense()?.filter_map(|mut cam| {
        if cam.get_name().to_string_lossy().contains("RealSense") {
            Some(cam)
        } else {
            cam.ignore_drop();
            None
        }
    }) {
        info!("Discovered {:?} at {:?}", cam.get_name(), cam.get_path());
        cam.res_x = CAMERA_WIDTH;
        cam.res_y = CAMERA_HEIGHT;
        // cam.fps = 20;
        let subscriber = Subscriber::new(1);
        cam.image_received_pub().accept_subscription(
            subscriber
                .create_subscription()
                .map(|img: Arc<DynamicImage>| img.to_rgb8()),
        );
        let context = context.make_context(cam.get_name().to_string_lossy());
        cam.spawn(context);
        if let Ok(sub) = subscriber
        .into_watch_or_closed()
        .await {
            camera_subs.push(sub);
        }
    }

    let mut telemetry = Telemetry::new(20, camera_subs).await?;

    match serial::connect_to_serial() {
        Ok((arms, drive)) => {
            if let Some(drive) = drive {
                telemetry
                    .steering_pub()
                    .accept_subscription(drive.get_steering_sub());
                let sub = Subscriber::new(4);
                drive.get_current_pub().accept_subscription(sub.create_subscription());
                let _ = sub.into_logger(|(left, right)| format!("{left} {right}"), "currents.log", &context).await;
                let sub = Subscriber::new(4);
                drive.get_current_pub().accept_subscription(sub.create_subscription());
                drive.spawn(context.make_context("drive"));
                unros::tokio::spawn(async move {
                    loop {
                        let (left_current, right_current) = sub.recv().await;
                        unros::log::info!("{left_current} {right_current}");
                    }
                });
            }

            if let Some(arms) = arms {
                telemetry.arm_pub().accept_subscription(arms.get_arm_sub());
                telemetry.odometry_sub(arms.get_odometry_pub());
                arms.spawn(context.make_context("arms"));
            }
        }
        Err(e) => {
            error!("{e}");
        }
    }

    telemetry.spawn(context.make_context("telemetry"));

    context.wait_for_exit().await;
    Ok(())
}
