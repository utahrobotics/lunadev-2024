#[cfg(unix)]
#[unros::main]
async fn main(mut app: unros::Application) -> unros::anyhow::Result<unros::Application> {
    use unros::{pubsub::Subscriber, tokio};

    realsense::discover_all_realsense()?.for_each(|mut x| {
        let mut img_sub = Subscriber::new(4);
        x.accept_image_received_sub(img_sub.create_subscription());
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
        app.add_node(x);
    });
    Ok(app)
}

#[cfg(not(unix))]
fn main() {
    unimplemented!("Realsense is not implemented on non-unix systems")
}
