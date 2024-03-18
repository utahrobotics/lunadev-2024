use std::{io::stdin, num::NonZeroUsize, time::Duration};

use camera::{discover_all_cameras, Camera};
use camera_info::CameraInfo;
use opencv::{calib3d::{calibrate_camera, find_chessboard_corners, CALIB_CB_ADAPTIVE_THRESH, CALIB_CB_NORMALIZE_IMAGE}, core::{Mat, Point3d, Size, TermCriteria, Vector}, imgproc::corner_sub_pix, types::{VectorOfMat, VectorOfPoint2d, VectorOfPoint3d, VectorOfVec3d}};
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


    let mut camera = Camera::new(index)?;
    let mut sub = Subscriber::new(1);
    camera.accept_image_received_sub(sub.create_subscription());
    let camera_name = camera.get_camera_name().to_string();
    println!("Capturing initial photo");

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
                        _ = tokio::time::sleep(Duration::from_secs(5)) => {}
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
                let img = img.to_luma8();
                let img_size = Size::new(img.width() as i32, img.height() as i32);
                let img = Mat::from_slice_rows_cols(&img, img.height() as usize, img.width() as usize).expect("Image should have been converted into a matrix");

                let mut corners = VectorOfPoint2d::new();
                println!("Finding chessboard corners...");
                let success = find_chessboard_corners(&img, Size::new(7, 6), &mut corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE).expect("Failed to execute find_chessboard_corners");

                if !success {
                    return Err(anyhow::anyhow!("Failed to find a chessboard!"));
                }

                println!("Refining corners...");
                let criteria = TermCriteria::default().expect("Failed to generate default TermCriteria");
                corner_sub_pix(&img, &mut corners, Size::new(11, 11), Size::new(-1, -1), criteria).expect("Failed to execute corner_sub_pix");
                
                let mut object_points = VectorOfPoint3d::new();
                for y in 0..6 {
                    for x in 0..7 {
                        object_points.push(Point3d::new(x as f64, y as f64, 0.0));
                    }
                }

                let mut image_points = Vector::<VectorOfPoint2d>::new();
                image_points.push(corners);
                let mut camera_matrix = Mat::from_slice_rows_cols(&[0, 0, 0, 0, 0, 0, 0, 0, 0], 3, 3).unwrap();
                let mut dist_coeffs = Vector::<f64>::new();
                let mut rvecs = VectorOfMat::new();
                let mut tvecs = VectorOfVec3d::new();

                println!("Calculating distortion...");
                let err = calibrate_camera(&object_points, &image_points, img_size, &mut camera_matrix, &mut dist_coeffs, &mut rvecs, &mut tvecs, 0, criteria).expect("Failed to execute calibrate_camera");
                println!("{err}\n{camera_matrix:?}\n{dist_coeffs:?}");
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
