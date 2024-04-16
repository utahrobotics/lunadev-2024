use std::sync::Arc;
use unros::rayon;
use realsense::discover_all_realsense;
use realsense::PointCloud;
use costmap::CostmapGenerator;
use unros::anyhow;
use unros::runtime::MainRuntimeContext;
use unros::node::SyncNode;
use unros::node::AsyncNode;
use unros::pubsub::subs::Subscription;
use rig::Robot;
use std::hash::BuildHasherDefault;
use std::hash::DefaultHasher;
use unros::pubsub::Subscriber;

#[unros::main]
async fn main(context: MainRuntimeContext) -> anyhow::Result<()> {
    let mut rig: Robot = Robot::default();
    rig.add_center_element();
    let (mut elements, _robot_base) = rig.destructure::<BuildHasherDefault<DefaultHasher>>(["center"])?;
    let camera_element = elements.remove("center").unwrap();

    let costmap = CostmapGenerator::new(10);

    let realsense_camera =
        {
            use costmap::Points;
            let mut camera = discover_all_realsense()?
                .next()
                .ok_or_else(|| anyhow::anyhow!("No realsense camera"))?;

            camera.set_robot_element_ref(camera_element.get_ref());
            let camera_element_ref = camera_element.get_ref();
            camera
                .cloud_received_pub()
                .accept_subscription(costmap.create_points_sub(0.1).map(move |x: PointCloud| {
                    Points {
                        points: x.iter().map(|x| x.0),
                        robot_element: camera_element_ref.clone(),
                    }
                }));

            camera
        };
    
    let costmap_sub = Subscriber::new(1);
    costmap
        .get_costmap_pub()
        .accept_subscription(costmap_sub.create_subscription());
    let mut costmap_display =
        unros::logging::dump::VideoDataDump::new_display(400, 400, 24, &context)?;

    costmap.spawn(context.make_context("costmap"));
    realsense_camera.spawn(context.make_context("realsense_camera"));

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
            let position = camera_element.get_global_isometry().translation.vector;
            // let img = costmap.get_obstacle_map(position.into(), 0.015, 400, 400, 0.5, 0.05);
            let img = costmap.get_cost_map(position.into(), 0.015, 400, 400);
            costmap_display.write_frame(Arc::new(img.into())).unwrap();
        }
    });

    context.wait_for_exit().await;
    Ok(())
}