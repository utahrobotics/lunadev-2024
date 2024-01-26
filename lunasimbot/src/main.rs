use std::time::Duration;

use costmap::Costmap;
use fxhash::FxBuildHasher;
use nalgebra::{Isometry, Point3, Quaternion, Translation3, UnitQuaternion};
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::{dump::VideoDataDump, init_logger},
    pubsub::Publisher,
    tokio::{
        self,
        io::{AsyncReadExt, BufReader},
        net::TcpListener,
    },
    FnNode,
};

type Float = f32;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = default_run_options!();
    init_logger(&run_options)?;

    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (_, robot_base) = rig.destructure::<FxBuildHasher>([])?;

    let mut costmap = Costmap::new(40, 40, 0.5, 10.0, 10.0, 0.01);
    let mut points_signal = Publisher::<Vec<Point3<f32>>>::default();

    points_signal.accept_subscription(costmap.create_points_sub());
    let costmap_ref = costmap.get_ref();

    let mut costmap_writer = VideoDataDump::new(720, 720, "costmap.mkv")?;
    let mut subtitle_writer = costmap_writer.init_subtitles().await?;

    let video_maker = FnNode::new(|_| async move {
        let mut i = 0;
        loop {
            tokio::time::sleep(Duration::from_millis(42)).await;
            let costmap = costmap_ref.get_costmap();
            let (img, max) = costmap_ref.costmap_to_img(costmap);
            i += 1;
            let _ = img.save(format!("img{i}.png"));

            costmap_writer.write_frame(img.into()).unwrap();

            subtitle_writer.write_subtitle(format!("{max:.2}")).unwrap();
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
                let x = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as Float;
                let y = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]) as Float;
                let z = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]) as Float;
                let w = f32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]) as Float;
                let i = f32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]) as Float;
                let j = f32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]) as Float;
                let k = f32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]) as Float;
                let n = u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]) as usize;
                robot_base.set_isometry(Isometry {
                    rotation: UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k)),
                    translation: Translation3::new(x, y, z),
                });

                points.reserve(n.saturating_sub(points.capacity()));
                let mut buf = [0u8; 12];
                for _ in 0..n {
                    stream
                        .read_exact(&mut buf)
                        .await
                        .expect("Failed to receive point");
                    let x = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                    let y = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                    let z = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
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
