use std::{
    io::Write,
    time::{Duration, Instant},
};

use costmap::Costmap;
use fxhash::FxBuildHasher;
use nalgebra::{Isometry, Vector3, Quaternion, Normed, UnitQuaternion, Translation};
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::{
        dump::{DataDump, VideoDataDump},
        init_logger,
    },
    rayon::iter::ParallelIterator,
    signal::unbounded::UnboundedSubscription,
    tokio::{self, net::{TcpListener, tcp}, io::AsyncReadExt}, FnNode,
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

    let tcp_listener = TcpListener::bind("0.0.0.0:11433").await?;
    let sim_conn = FnNode::new(|_| async move {
        loop {
            let (mut stream, _) = tcp_listener.accept().await.expect("Connection should have succeeded");
            let mut buf = [0u8; 32];
            loop {
                stream.read_exact(&mut buf).await.expect("Failed to receive packet");
                let x = f32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
                let y = f32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]);
                let z = f32::from_be_bytes([buf[8], buf[9], buf[10], buf[11]]);
                let w = f32::from_be_bytes([buf[12], buf[13], buf[14], buf[15]]);
                let i = f32::from_be_bytes([buf[16], buf[17], buf[18], buf[19]]);
                let j = f32::from_be_bytes([buf[20], buf[21], buf[22], buf[23]]);
                let k = f32::from_be_bytes([buf[24], buf[25], buf[26], buf[27]]);
                let n = u32::from_be_bytes([buf[28], buf[29], buf[30], buf[31]]);
                robot_base.set_isometry(Isometry { rotation: UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k)), translation: Translation::new(x, y, z) });
            }
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
