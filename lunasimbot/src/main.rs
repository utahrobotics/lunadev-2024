use std::{
    io::Write,
    time::{Duration, Instant},
};

use costmap::Costmap;
use fxhash::FxBuildHasher;
use nalgebra::{Isometry, Vector3};
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::{
        dump::{DataDump, VideoDataDump},
        init_logger,
    },
    rayon::iter::ParallelIterator,
    signal::unbounded::UnboundedSubscription,
    tokio, FnNode,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = default_run_options!();
    init_logger(&run_options)?;

    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "debug"])?;
    let camera_element = elements.remove("camera").unwrap();
    let debug_element = elements.remove("debug").unwrap();
    let robot_base_ref = robot_base.get_ref();

    let mut costmap = Costmap::new(40, 40, 0.05, 1.9, 0.0);

    costmap.add_points_sub(
        camera
            .point_cloud_received_signal()
            .subscribe_unbounded()
            .map(|x| x.par_iter().map(|x| x.0)),
    );
    let costmap_ref = costmap.get_ref();

    let mut costmap_writer = VideoDataDump::new(720, 720, "costmap.mkv")?;

    let video_maker = FnNode::new(|_| async move {
        loop {
            tokio::time::sleep(Duration::from_millis(42)).await;
            costmap_writer
                .write_frame(costmap_ref.get_costmap_img().into())
                .unwrap();
        }
    });

    async_run_all(
        [
            video_maker.into(),
            costmap.into(),
        ],
        run_options,
    )
    .await
}
