use realsense::discover_all_realsense;
use unros_core::{
    anyhow, async_run_all, default_run_options, pubsub::Subscriber, tokio, FinalizedNode,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cameras = discover_all_realsense()?;

    // let frame_count = AtomicUsize::new(0);
    // let frame_count: &_ = Box::leak(Box::new(frame_count));
    async_run_all(
        cameras.map(|mut x| {
            let mut img_sub = Subscriber::default();
            x.accept_image_received_sub(img_sub.create_subscription(4));
            // let mut imu_sub = x.imu_frame_received().watch();
            tokio::spawn(async move {
                let mut i = 0;
                loop {
                    let img = img_sub.recv().await;
                    img.save(format!("{i}.png")).unwrap();
                    i += 1;
                    // println!("{:?}", img_sub.recv().await.dimensions());
                    // let imu = imu_sub.wait_for_change().await;
                    // println!(
                    //     "ang_vel: {} accel: {}",
                    //     imu.angular_velocity / PI * 180.0,
                    //     imu.acceleration
                    // );
                }
            });
            FinalizedNode::from(x)
        }),
        default_run_options!(),
    )
    .await
}
