use std::time::Duration;

use costmap::Costmap;
use fxhash::FxBuildHasher;
use nalgebra::{Isometry, Point3, Quaternion, Translation3, UnitQuaternion};
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::{dump::VideoDataDump, init_logger},
    signal::Signal,
    tokio::{
        self,
        io::{AsyncReadExt, BufReader},
        net::TcpListener,
    },
    FnNode,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = default_run_options!();
    init_logger(&run_options)?;

    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (_, robot_base) = rig.destructure::<FxBuildHasher>([])?;

    let mut costmap = Costmap::new(40, 40, 0.05, 1.9, 0.0);
    let mut points_signal = Signal::<Vec<Point3<f32>>>::default();

    costmap.add_points_sub(points_signal.get_ref().subscribe_unbounded());
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
            let (stream, _) = tcp_listener
                .accept()
                .await
                .expect("Connection should have succeeded");
            let mut stream = BufReader::new(stream);
            let mut buf = [0u8; 32];
            let mut points = vec![];

            loop {
                stream
                    .read_exact(&mut buf)
                    .await
                    .expect("Failed to receive packet");
                let x = f32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]) as f64;
                let y = f32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]) as f64;
                let z = f32::from_be_bytes([buf[8], buf[9], buf[10], buf[11]]) as f64;
                let w = f32::from_be_bytes([buf[12], buf[13], buf[14], buf[15]]) as f64;
                let i = f32::from_be_bytes([buf[16], buf[17], buf[18], buf[19]]) as f64;
                let j = f32::from_be_bytes([buf[20], buf[21], buf[22], buf[23]]) as f64;
                let k = f32::from_be_bytes([buf[24], buf[25], buf[26], buf[27]]) as f64;
                let n = u32::from_be_bytes([buf[28], buf[29], buf[30], buf[31]]) as usize;
                robot_base.set_isometry(Isometry {
                    rotation: UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k)),
                    translation: Translation3::new(x, y, z),
                });

                points.reserve(n - points.capacity());
                let mut buf = [0u8; 12];
                for _ in 0..n {
                    stream
                        .read_exact(&mut buf)
                        .await
                        .expect("Failed to receive point");
                    let x = f32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);
                    let y = f32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]);
                    let z = f32::from_be_bytes([buf[8], buf[9], buf[10], buf[11]]);
                    points.push(Point3::new(x, y, z));
                }

                let capacity = points.capacity();
                points_signal.set(points);
                points = Vec::with_capacity(capacity);
            }
        }
    });

    async_run_all(
        [video_maker.into(), costmap.into(), sim_conn.into()],
        run_options,
    )
    .await
}
