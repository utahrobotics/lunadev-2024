use std::sync::Arc;

use camera::discover_all_cameras;
use ffmpeg_sidecar::command::FfmpegCommand;
use image::DynamicImage;
use unros::{
    anyhow, log::info, node::SyncNode, pubsub::{subs::Subscription, MonoPublisher, Subscriber, WatchSubscriber}, runtime::MainRuntimeContext, ShouldNotDrop
};

use crate::{CAMERA_HEIGHT, CAMERA_WIDTH};

pub async fn setup_teleop_cameras(
    cam_width: u32,
    cam_height: u32,
    context: &MainRuntimeContext,
    sub: impl Subscription<Item = Arc<DynamicImage>>,
) -> anyhow::Result<()> {
    let mut img_pub = MonoPublisher::from(sub);
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
        let subscriber = Subscriber::new(1);
        cam.image_received_pub().accept_subscription(subscriber.create_subscription());
        let context = context.make_context(cam.get_camera_name());
        cam.spawn(context);
        camera_subs.push(subscriber.into_watch().await);
    }

    ffmpeg_sidecar::download::auto_download()?;

    let output = FfmpegCommand::new()
        .hwaccel("auto")
        .format("rawvideo")
        .pix_fmt("rgb24")
        .size(cam_width, cam_height)
        .input("-")
        .codec_video("libx264")
        .pix_fmt("yuv420p")
        .args([
            "-crf",
            "35",
            "-an",
            "-vf",
            "fps=20",
            "-sws_flags",
            "neighbor",
        ])
        .args([
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-strict",
            "2",
            "-avioflags",
            "direct",
            "-rtsp_transport",
            "udp",
        ])
        // .args(["-sdp_file", "sdp.txt"])
        .format("rtp")
        .output(format!("rtp://{addr}"))
        .spawn()
        .map_err(VideoDumpInitError::IOError)?;

    Self::new(
        in_width,
        in_height,
        fps,
        VideoDataDumpType::Rtp(addr),
        output,
        context,
    )

    std::thread::spawn(move || loop {
        let updated = camera_subs.iter_mut().any(|sub| WatchSubscriber::try_update(sub));
        if updated {

        }
    });

    Ok(())
}
