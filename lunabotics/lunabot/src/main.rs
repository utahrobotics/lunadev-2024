// use apriltag::{AprilTagDetector, PoseObservation};
use fxhash::FxBuildHasher;
// use localization::{
//     engines::window::{DefaultWindowConfig, WindowLocalizer},
//     Localizer,
// };
use rig::Robot;
use telemetry::Telemetry;
use unros::{anyhow, node::AsyncNode, runtime::MainRuntimeContext, setup_logging};

use crate::{audio::init_buzz, mosaic::setup_teleop_cameras};

mod actuators;
mod drive;
mod serial;
// mod imu;
mod audio;
mod mosaic;
mod telemetry;

const MAX_CAMERA_COUNT: usize = 5;
const ROW_LENGTH: usize = 3;

const CAMERA_WIDTH: u32 = 640;
const CAMERA_HEIGHT: u32 = 360;

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    setup_logging!(context);
    init_buzz()?;

    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "imu01"])?;
    let _camera_element = elements.remove("camera").unwrap();
    let _robot_base_ref = robot_base.get_ref();

    let teleop_cam_width = CAMERA_WIDTH * ROW_LENGTH as u32;
    let teleop_cam_height = CAMERA_HEIGHT * MAX_CAMERA_COUNT.div_ceil(ROW_LENGTH) as u32;

    let telemetry = Telemetry::new(
        teleop_cam_width, teleop_cam_height,
        20,
    )
    .await?;

    setup_teleop_cameras(teleop_cam_width, teleop_cam_height, &context, addr);

    match serial::connect_to_serial() {
        Ok((arms, drive)) => {
            if let Some(drive) = drive {
                telemetry
                    .steering_pub()
                    .accept_subscription(drive.get_steering_sub());
                drive.spawn(context.make_context("drive"));
            }

            if let Some(arms) = arms {
                telemetry.arm_pub().accept_subscription(arms.get_arm_sub());
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
