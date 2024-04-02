use std::io::stdin;

use camera::discover_all_cameras;
use camera_info::interactive_examine;
use unros::{
    anyhow::{self, Context}, tokio::{self, task::JoinHandle}, Application, Node
};

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let mut cameras: Vec<_> = discover_all_cameras()
        .context("Failed to discover cameras")?
        .enumerate()
        .map(|(i, mut x)| {
            println!("Discovered {} at {i}", x.get_camera_name());
            x.get_intrinsics().ignore_drop();
            x
        })
        .collect();

    let join: JoinHandle<Result<_, anyhow::Error>> = tokio::task::spawn_blocking(|| {
        let stdin = stdin();
        println!("Please provide a camera index");
        let mut input = String::new();
        let index = loop {
            stdin.read_line(&mut input)?;
            let Ok(index) = input.trim().parse::<usize>() else {
                println!("Invalid integer!");
                input.clear();
                continue;
            };

            break index;
        };

        Ok(index)
    });

    let index = join.await.unwrap()?;

    let camera = cameras.remove(index);
    let camera_name = camera.get_camera_name().to_string();
    interactive_examine(
        &mut app,
        |x| camera.image_received_pub().accept_subscription(x),
        camera_name,
    )
    .await;
    app.add_node(camera);

    Ok(app)
}
