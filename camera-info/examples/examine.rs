use std::{io::stdin, num::NonZeroUsize, time::Duration};

use camera::{discover_all_cameras, Camera};
use camera_info::CameraInfo;
use serde_json::to_string_pretty;
use unros::{
    anyhow::{self, Context}, pubsub::Subscriber, setup_logging, tokio::{self, task::JoinHandle}, Application
};

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    discover_all_cameras()
        .context("Failed to discover cameras")?
        .for_each(|_| {});

    let join: JoinHandle<Result<_, anyhow::Error>> = tokio::task::spawn_blocking(|| {
        let stdin = stdin();
        println!("Please provide a camera index");
        let mut input = String::new();
        let index = loop {
            stdin.read_line(&mut input)?;
            let Ok(index) = input.trim().parse::<u32>() else {
                println!("Invalid integer!");
                input.clear();
                continue;
            };
        
            break index;
        };
    
        let estimate_fps;
    
        loop {
            println!("Estimate fps (Y/N)?");
            input.clear();
            stdin.read_line(&mut input)?;
            match input.to_ascii_lowercase().as_str() {
                "y\n" => {
                    estimate_fps = true;
                }
                "n\n" => estimate_fps = false,
                _ => continue,
            }
            break;
        }
    
        let chessboard;
    
        loop {
            println!("Will you be displaying a chessboard (Y/N)?");
            input.clear();
            stdin.read_line(&mut input)?;
            match input.to_ascii_lowercase().as_str() {
                "y\n" => {
                    chessboard = true;
                    println!("Prepare to hold it up in...");
                    const SEC: Duration = Duration::from_secs(1);
                    std::thread::sleep(SEC);
                    println!("3");
                    std::thread::sleep(SEC);
                    println!("2");
                    std::thread::sleep(SEC);
                    println!("1");
                    std::thread::sleep(SEC);
                }
                "n\n" => chessboard = false,
                _ => continue,
            }
            break;
        }

        Ok((index, chessboard, estimate_fps))
    });
    
    let (index, chessboard, estimate_fps) = join.await.unwrap()?;

    println!("Capturing initial photo");

    let mut camera = Camera::new(index)?;
    let mut sub = Subscriber::new(1);
    camera.accept_image_received_sub(sub.create_subscription());
    let camera_name = camera.get_camera_name().to_string();

    app.add_node(camera);
    app.add_task(
        move |context| async move {
            setup_logging!(context);

            let Some(img) = sub.recv_or_closed().await else {
                return Err(anyhow::anyhow!("Camera did not produce any frames!"));
            };

            let mut camera_info = CameraInfo { width: img.width(), height: img.height(), fps: None };

            if estimate_fps {
                loop {
                    println!("Estimating fps across 5 seconds");
                    let mut count = 0;
                    let fut = async {
                        loop {
                            let Some(_) = sub.recv_or_closed().await else {
                                return anyhow::anyhow!("Camera stopped unexpectedly!");
                            };
                            count += 1;
                        }
                    };
                    tokio::select! {
                        e = fut => return Err(e),
                        _ = tokio::time::sleep(Duration::from_secs(5)) => {
    
                        }
                    }
                    let fps = (count as f32 / 5.0).round() as usize;
                    if fps == 0 {
                        println!("Received no frames during fps testing!");
                    }
                    camera_info.fps = Some(NonZeroUsize::new(fps).unwrap());
                    break;
                }
            }

            if chessboard {
                todo!();
            }

            let display = to_string_pretty(&camera_info).unwrap();

            println!("Finished examination of: {camera_name}");
            println!("{display}");

            Ok(())
        },
        "examiner",
    );

    Ok(app)
}
