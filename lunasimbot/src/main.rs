use costmap::Costmap;
use fxhash::FxBuildHasher;
use localization::{
    frames::{IMUFrame, OrientationFrame, PositionFrame, VelocityFrame},
    Localizer,
};
use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use navigator::{pathfinders::DirectPathfinder, DifferentialDriver};
use rig::Robot;
use unros::{
    anyhow, log,
    pubsub::{Publisher, Subscriber},
    tokio::{
        self,
        io::{AsyncReadExt, AsyncWriteExt, BufStream},
        net::TcpListener,
    },
    Application,
};

type Float = f32;

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let rig: Robot = toml::from_str(include_str!("lunabot.toml"))?;
    let (mut elements, robot_base) = rig.destructure::<FxBuildHasher>(["camera", "debug"])?;
    let mut camera = elements.remove("camera").unwrap();
    let debug_element = elements.remove("debug").unwrap();

    let costmap = Costmap::new(400, 400, 0.05, 10.0, 10.0, 0.01);
    let mut points_signal = Publisher::<Vec<Point3<f32>>>::default();

    points_signal.accept_subscription(costmap.create_points_sub());
    // let costmap_ref = costmap.get_ref();

    // let mut costmap_writer = unros::logging::dump::VideoDataDump::new(720, 720, "costmap.mkv")?;

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

    let mut pathfinder = DirectPathfinder::new(robot_base.get_ref(), costmap.get_ref(), 0.65, 0.2);

    let mut driver = DifferentialDriver::new(robot_base.get_ref());
    // driver.can_reverse = true;
    pathfinder.accept_path_sub(driver.create_path_sub());
    let nav_task = pathfinder.get_navigation_handle();

    // let robot_base_ref = robot_base.get_ref();
    let localizer = Localizer::new(
        robot_base,
        0.0
    );

    let mut position_pub = Publisher::default();
    position_pub.accept_subscription(localizer.create_position_sub().set_name("position"));

    let mut velocity_pub = Publisher::default();
    velocity_pub.accept_subscription(localizer.create_velocity_sub().set_name("velocity"));

    let mut orientation_pub = Publisher::default();
    orientation_pub.accept_subscription(localizer.create_orientation_sub().set_name("orientation"));

    let mut imu_pub = Publisher::default();
    imu_pub.accept_subscription(localizer.create_imu_sub().set_name("imu"));

    let mut steering_sub = Subscriber::new(1);
    driver.accept_steering_sub(steering_sub.create_subscription());

    let tcp_listener = TcpListener::bind("0.0.0.0:11433").await?;
    app.add_task(
        |_| async move {
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
                let vx = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let _vy = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let vz = stream
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
                let vw = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let vi = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let vj = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let vk = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let x_rot = stream
                    .read_f32_le()
                    .await
                    .expect("Failed to receive packet") as Float;
                let n = stream
                    .read_u32_le()
                    .await
                    .expect("Failed to receive packet") as usize;

                position_pub.set(PositionFrame::rand(
                    Point3::new(x, 0.0, z),
                    0.02,
                    debug_element.get_ref(),
                ));
                velocity_pub.set(VelocityFrame::rand(
                    Vector3::new(vx, 0.0, vz),
                    0.02,
                    debug_element.get_ref(),
                ));
                orientation_pub.set(OrientationFrame::rand(
                    UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k)),
                    0.03,
                    debug_element.get_ref(),
                ));
                imu_pub.set(IMUFrame::rand(
                    Vector3::new(0.0, -9.81, 0.0),
                    0.0,
                    UnitQuaternion::new_unchecked(Quaternion::new(vw, vi, vj, vk)),
                    0.03,
                    debug_element.get_ref(),
                ));

                let mut camera_joint = match camera.get_local_joint() {
                    rig::joints::JointMut::Hinge(x) => x,
                    _ => unreachable!(),
                };

                camera_joint.set_angle(x_rot);
                let camera_isometry = camera.get_global_isometry();
                points.reserve(n.saturating_sub(points.capacity()));
                // let mut first = true;
                for _ in 0..n {
                    let x = stream.read_f32_le().await.expect("Failed to receive point");
                    let y = stream.read_f32_le().await.expect("Failed to receive point");
                    let z = stream.read_f32_le().await.expect("Failed to receive point");
                    let point = camera_isometry * Point3::new(x, y, z);
                    // if first {
                    //     println!("{point:?}");
                    //     first = false;
                    // }
                    points.push(point);
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
                    match nav_task.try_schedule(Point3::new(x, 0.0, y)).await {
                        Ok(handle) => {
                            tokio::spawn(async move {
                                match handle.wait().await {
                                    Ok(()) => log::info!("Navigation complete"),
                                    Err(e) => log::error!("{e}"),
                                }
                            });
                        }
                        Err(e) => log::error!("{e}"),
                    }
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

                let isometry = camera.get_isometry_of_base();

                stream
                    .write_f32_le(isometry.translation.x)
                    .await
                    .expect("Failed to write position");
                stream
                    .write_f32_le(isometry.translation.y)
                    .await
                    .expect("Failed to write position");
                stream
                    .write_f32_le(isometry.translation.z)
                    .await
                    .expect("Failed to write position");

                stream
                    .write_f32_le(isometry.rotation.w)
                    .await
                    .expect("Failed to write orientation");
                stream
                    .write_f32_le(isometry.rotation.i)
                    .await
                    .expect("Failed to write orientation");
                stream
                    .write_f32_le(isometry.rotation.j)
                    .await
                    .expect("Failed to write orientation");
                stream
                    .write_f32_le(isometry.rotation.k)
                    .await
                    .expect("Failed to write orientation");

                stream.flush().await.expect("Failed to write steering");
            }
        },
        "telemetry",
    );

    app.add_node(costmap);
    app.add_node(driver);
    app.add_node(pathfinder);
    app.add_node(localizer);

    Ok(app)
}
