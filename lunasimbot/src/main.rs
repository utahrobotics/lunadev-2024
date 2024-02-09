use costmap::Costmap;
use fxhash::FxBuildHasher;
use nalgebra::{Isometry, Point2, Point3, Quaternion, Translation3, UnitQuaternion};
use navigator::{DrivingTaskScheduleData, WaypointDriver};
use rig::Robot;
use unros_core::{
    anyhow, async_run_all, default_run_options,
    logging::init_logger,
    pubsub::{Publisher, Subscriber},
    task::Task,
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt, BufStream},
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

    let mut costmap = Costmap::new(400, 400, 0.05, 10.0, 10.0, 0.01);
    let mut points_signal = Publisher::<Vec<Point3<f32>>>::default();

    points_signal.accept_subscription(costmap.create_points_sub());
    // let costmap_ref = costmap.get_ref();

    // let mut costmap_writer = unros_core::logging::dump::VideoDataDump::new(720, 720, "costmap.mkv")?;

    // let video_maker = FnNode::new(|_| async move {
    //     // let mut i = 0;
    //     loop {
    //         tokio::time::sleep(std::time::Duration::from_millis(42)).await;
    //         let costmap = costmap_ref.get_costmap();
    //         let obstacles = costmap_ref.costmap_to_obstacle(&costmap, 0.2, 0.0, 0.3);
    //         let img = costmap_ref.obstacles_to_img(&obstacles);

    //         i += 1;
    //         let _ = img.save(format!("img{i}.png"));

    //         // costmap_writer.write_frame(img.into()).unwrap();
    //     }
    // });

    let mut navigator = WaypointDriver::new(robot_base.get_ref(), costmap.get_ref(), 0.65, 0.025, 0.3);
    // navigator.can_reverse = false;
    let nav_task = navigator.get_driving_task().clone();

    let mut steering_sub = Subscriber::default();
    navigator.accept_steering_sub(steering_sub.create_subscription(1));

    let tcp_listener = TcpListener::bind("0.0.0.0:11433").await?;
    let sim_conn = FnNode::new(|_| async move {
        loop {
            let (stream, _) = tcp_listener
                .accept()
                .await
                .expect("Connection should have succeeded");
            let mut stream = BufStream::new(stream);
            let mut points = vec![];
            let mut last_left_steering = 0.0;
            let mut last_right_steering = 0.0;

            loop {
                let x = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let _y = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let z = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let w = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let i = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let j = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let k = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let n = stream
                    .read_u32_le()
                    .await
                    .expect("Failed to receive packet") as usize;
                robot_base.set_isometry(Isometry {
                    rotation: UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k)),
                    translation: Translation3::new(x, 0.0, z),
                });

                points.reserve(n.saturating_sub(points.capacity()));
                for _ in 0..n {
                    let x = stream.read_f32_le().await.expect("Failed to receive point");
                    let y = stream.read_f32_le().await.expect("Failed to receive point");
                    let z = stream.read_f32_le().await.expect("Failed to receive point");
                    points.push(Point3::new(x, y, z));
                }

                let capacity = points.capacity();
                points_signal.set(points);
                points = Vec::with_capacity(capacity);

                if stream
                    .read_u8()
                    .await
                    .expect("Failed to receive waypoint byte")
                    == 255
                {
                    let x = stream.read_f32_le().await.expect("Failed to receive point");
                    let y = stream.read_f32_le().await.expect("Failed to receive point");
                    nav_task
                        .try_schedule(DrivingTaskScheduleData {
                            destination: Point2::new(x, y),
                            orientation: None,
                        })
                        .await
                        .unwrap();
                }

                if let Some(steering) = steering_sub.try_recv() {
                    last_left_steering = steering.left.into_inner();
                    last_right_steering = steering.right.into_inner();
                    stream
                        .write_f32_le(last_left_steering)
                        .await
                        .expect("Failed to write steering");
                    stream
                        .write_f32_le(last_right_steering)
                        .await
                        .expect("Failed to write steering");
                } else {
                    stream
                        .write_f32_le(last_left_steering)
                        .await
                        .expect("Failed to write steering");
                    stream
                        .write_f32_le(last_right_steering)
                        .await
                        .expect("Failed to write steering");
                }
                stream.flush().await.expect("Failed to write steering");
            }
        }
    });

    async_run_all(
        [
            // video_maker.into(),
            costmap.into(),
            sim_conn.into(),
            navigator.into(),
        ],
        run_options,
    )
    .await
}
