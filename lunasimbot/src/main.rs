use std::{ops::DerefMut, sync::Arc};

use costmap::{CostmapGenerator, Points};
use fxhash::FxBuildHasher;
use localization::{
    frames::{IMUFrame, OrientationFrame, PositionFrame, VelocityFrame},
    Localizer,
};
use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use navigator::{pathfinding::Pathfinder, DifferentialDriver};
// use navigator::{pathfinders::DirectPathfinder, DifferentialDriver};
use rand_distr::{Distribution, Normal};
use rig::Robot;
use unros::{
    anyhow, log,
    pubsub::{subs::Subscription, Publisher, Subscriber},
    rayon,
    rng::quick_rng,
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
    let camera_ref = camera.get_ref();
    let debug_element = elements.remove("debug").unwrap();

    let costmap = CostmapGenerator::new(10);
    let points_signal = Publisher::<Vec<Point3<f32>>>::default();

    points_signal.accept_subscription(costmap.create_points_sub(0.05).map(move |points| Points {
        points,
        robot_element: camera_ref.clone(),
    }));

    let costmap_sub = Subscriber::new(1);
    costmap
        .get_costmap_pub()
        .accept_subscription(costmap_sub.create_subscription());

    let mut costmap_display = unros::logging::dump::VideoDataDump::new_display(400, 400, 24)?;
    let debug_element_ref = debug_element.get_ref();

    rayon::spawn(move || {
        loop {
            std::thread::sleep(std::time::Duration::from_millis(100));
            let Some(costmap) = costmap_sub.try_recv() else {
                if costmap_sub.get_pub_count() == 0 {
                    break;
                } else {
                    continue;
                }
            };
            let position = debug_element_ref.get_global_isometry().translation.vector;
            let img = costmap.get_obstacle_map(position.into(), 0.015, 400, 400, 0.3, 0.15);
            // let img = costmap.get_cost_map(position.into(), 0.015, 400, 400);
            costmap_display.write_frame(Arc::new(img.into())).unwrap();
        }
    });

    let pathfinder: Pathfinder =
        Pathfinder::new_with_engine(0.5, Default::default(), robot_base.get_ref());
    costmap
        .get_costmap_pub()
        .accept_subscription(pathfinder.create_costmap_sub());
    let path_sub = Subscriber::new(1);
    pathfinder
        .get_path_pub()
        .accept_subscription(path_sub.create_subscription());

    let driver = DifferentialDriver::new(robot_base.get_ref());
    // driver.can_reverse = true;
    pathfinder
        .get_path_pub()
        .accept_subscription(driver.create_path_sub());
    let nav_task = pathfinder.get_navigation_handle();

    let localizer = Localizer::new(robot_base, 0.0);

    let position_pub = Publisher::default();
    position_pub.accept_subscription(localizer.create_position_sub().set_name("position"));

    let velocity_pub = Publisher::default();
    velocity_pub.accept_subscription(localizer.create_velocity_sub().set_name("velocity"));

    let orientation_pub = Publisher::default();
    orientation_pub.accept_subscription(localizer.create_orientation_sub().set_name("orientation"));

    let imu_pub = Publisher::default();
    imu_pub.accept_subscription(localizer.create_imu_sub().set_name("imu"));

    let steering_sub = Subscriber::new(1);
    driver
        .steering_pub()
        .accept_subscription(steering_sub.create_subscription());

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
                    0.03,
                    debug_element.get_ref(),
                ));
                velocity_pub.set(VelocityFrame::rand(
                    Vector3::new(vx, 0.0, vz),
                    0.03,
                    debug_element.get_ref(),
                ));
                let orientation = UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k));
                orientation_pub.set(OrientationFrame::rand(
                    orientation,
                    0.0,
                    debug_element.get_ref(),
                ));
                imu_pub.set(IMUFrame::rand(
                    orientation * Vector3::new(0.0, -9.81, 0.0),
                    0.0,
                    UnitQuaternion::new_unchecked(Quaternion::new(vw, vi, vj, vk)),
                    0.0,
                    debug_element.get_ref(),
                ));

                let mut camera_joint = match camera.get_local_joint() {
                    rig::joints::JointMut::Hinge(x) => x,
                    _ => unreachable!(),
                };

                camera_joint.set_angle(x_rot);
                points.reserve(n.saturating_sub(points.capacity()));
                let distr = Normal::new(0.0, 0.05).unwrap();
                let mut rng = quick_rng();
                for _ in 0..n {
                    let x = stream.read_f32_le().await.expect("Failed to receive point");
                    let y = stream.read_f32_le().await.expect("Failed to receive point");
                    let z = stream.read_f32_le().await.expect("Failed to receive point");

                    let mut vec = Vector3::new(x, y, z);

                    vec.scale_mut(1.0 + distr.sample(rng.deref_mut()));

                    points.push(vec.into());
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
                    match nav_task
                        .try_schedule_or_closed(Point3::new(x, 0.0, y))
                        .await
                    {
                        Some(Ok(handle)) => {
                            tokio::spawn(async move {
                                match handle.wait().await {
                                    Ok(()) => log::info!("Navigation complete"),
                                    Err(e) => log::error!("{e}"),
                                }
                            });
                        }
                        Some(Err(e)) => log::error!("{e}"),
                        None => log::error!("Navigation task closed"),
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

                if let Some(path) = path_sub.try_recv() {
                    stream
                        .write_u16_le(path.len() as u16)
                        .await
                        .expect("Failed to write path length");
                    for point in path.iter() {
                        stream
                            .write_f32_le(point.x)
                            .await
                            .expect("Failed to write point.x");
                        stream
                            .write_f32_le(point.z)
                            .await
                            .expect("Failed to write point.z");
                    }
                } else {
                    stream
                        .write_u16_le(0)
                        .await
                        .expect("Failed to write path length");
                }

                stream.flush().await.expect("Failed to write path");
            }
        },
        "telemetry",
    );

    app.add_node(driver);
    app.add_node(pathfinder);
    app.add_node(localizer);
    app.add_node(costmap);

    Ok(app)
}
