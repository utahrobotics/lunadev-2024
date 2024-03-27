use std::{
    array,
    collections::HashMap,
    hash::{BuildHasherDefault, DefaultHasher},
    io::stdin,
    num::NonZeroUsize,
    path::Path,
    sync::{Arc, OnceLock},
    time::Duration,
};

use apriltag::AprilTagDetector;
use image::DynamicImage;
use opencv::{
    calib3d::{
        calibrate_camera, find_chessboard_corners, get_optimal_new_camera_matrix, CALIB_CB_ADAPTIVE_THRESH, CALIB_CB_NORMALIZE_IMAGE,
    },
    core::{
        Mat, MatTraitConst, Point3f, Rect, Size, TermCriteria,
        Vector,
    },
    imgproc::corner_sub_pix,
    types::{VectorOfMat, VectorOfPoint2f, VectorOfPoint3f, VectorOfVec3d},
};
use rig::Robot;
use serde::{Deserialize, Serialize};
use serde_json::{from_reader, to_string_pretty, to_writer_pretty};
use sub::Undistorter;
use unros::{
    anyhow::{self, Context},
    log,
    pubsub::{
        subs::{DirectSubscription, Subscription},
        Publisher, Subscriber,
    },
    setup_logging,
    tokio::{self, task::JoinHandle},
    Application,
};

pub mod sub;

static CAMERA_DB: OnceLock<HashMap<String, Arc<CameraInfo>>> = OnceLock::new();

const DEFAULT_CAMERA_FOLDER: &str = "camera-db";

fn get_camera_db() -> &'static HashMap<String, Arc<CameraInfo>> {
    CAMERA_DB.get_or_init(|| {
        let mut map: HashMap<String, Arc<CameraInfo>> = HashMap::default();

        let paths = match std::fs::read_dir(DEFAULT_CAMERA_FOLDER) {
            Ok(x) => x,
            Err(e) => {
                log::error!("Faced the following error while trying to listdir: {DEFAULT_CAMERA_FOLDER}: {e}");
                return map;
            }
        };

        for path in paths {
            let path = match path {
                Ok(x) => x.path(),
                Err(e) => {
                    log::error!("Faced the following error while trying to listdir: {DEFAULT_CAMERA_FOLDER}: {e}");
                    continue;
                }
            };
            let file = match std::fs::File::open(&path) {
                Ok(x) => x,
                Err(e) => {
                    log::error!("Faced the following error while trying to read: {path:?}: {e}");
                    continue;
                }
            };
            let submap: HashMap<String, CameraInfo> = match from_reader(file) {
                Ok(x) => x,
                Err(e) => {
                    log::error!("Faced the following error while trying to parse: {path:?}: {e}");
                    continue;
                }
            };

            for key in submap.keys() {
                if map.contains_key(key) {
                    log::warn!("Found duplicate entry for {key}. Replacing...");
                }
            }

            map.extend(submap.into_iter().map(|(a, b)| (a, Arc::new(b))));
        }

        map
    })
}

#[derive(Serialize, Deserialize, Clone)]
struct DistortionData {
    distortion_coefficients: Vec<f64>,
    camera_matrix: [f64; 9],
    new_camera_matrix: [f64; 9],
    roi_x: u32,
    roi_y: u32,
    roi_width: u32,
    roi_height: u32,
}

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct CameraInfo {
    pub width: u32,
    pub height: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub fps: Option<NonZeroUsize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub focal_length_px: Option<NonZeroUsize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    distortion_data: Option<Arc<DistortionData>>,
}

impl CameraInfo {
    pub fn from_name(name: &str) -> Option<Arc<Self>> {
        get_camera_db().get(name).cloned()
    }

    pub fn undistort_subscription<T: Subscription<Item=Arc<DynamicImage>>>(
        &self,
        sub: T,
    ) -> Result<Undistorter<T>, T>
    {
        if let Some(distortion_data) = self.distortion_data.clone() {
            Ok(Undistorter {
                inner: sub,
                distortion_data
            })
        } else {
            Err(sub)
        }
    }

    pub fn get_undistorted_size(&self) -> (u32, u32) {
        if let Some(data) = &self.distortion_data {
            (data.roi_width, data.roi_height)
        } else {
            (self.width, self.height)
        }
    }

    pub fn has_distortion_data(&self) -> bool {
        self.distortion_data.is_some()
    }
}

impl Drop for CameraInfo {
    fn drop(&mut self) {
        let display = to_string_pretty(self).unwrap();
        println!("{display}");
    }
}

struct FocalLengthEstimate {
    tag_distance: f64,
    width: f64,
    id: usize,
}

/// <https://raw.githubusercontent.com/opencv/opencv/4.x/doc/pattern.png>
pub async fn interactive_examine(
    app: &mut Application,
    accept_sub: impl FnOnce(DirectSubscription<Arc<DynamicImage>>),
    camera_name: String,
) {
    let join: JoinHandle<Result<_, anyhow::Error>> = tokio::task::spawn_blocking(|| {
        let stdin = stdin();
        let mut input = String::new();

        let estimate_fps;

        loop {
            println!("Estimate fps (Y/N)?");
            input.clear();
            stdin.read_line(&mut input)?;
            match input.to_ascii_lowercase().trim() {
                "y" => {
                    estimate_fps = true;
                }
                "n" => estimate_fps = false,
                _ => continue,
            }
            break;
        }

        let chessboard;

        loop {
            println!("Will you be displaying a chessboard (Y/N)?");
            input.clear();
            stdin.read_line(&mut input)?;
            match input.to_ascii_lowercase().trim() {
                "y" => chessboard = true,
                "n" => chessboard = false,
                _ => continue,
            }
            break;
        }

        let focal_length_estimate;

        loop {
            println!("Estimate focal length (Y/N)?");
            input.clear();
            stdin.read_line(&mut input)?;
            match input.to_ascii_lowercase().trim() {
                "y" => {
                    let tag_distance = loop {
                        println!("What is the distance to the tag in meters?");
                        input.clear();
                        stdin.read_line(&mut input)?;
                        let Ok(tag_distance) = input.trim().parse::<f64>() else {
                            println!("Invalid float!");
                            input.clear();
                            continue;
                        };
                        break tag_distance;
                    };
                    let width = loop {
                        println!("What is the width of the tag in meters?");
                        input.clear();
                        stdin.read_line(&mut input)?;
                        let Ok(width) = input.trim().parse::<f64>() else {
                            println!("Invalid float!");
                            input.clear();
                            continue;
                        };
                        break width;
                    };
                    let id = loop {
                        println!("What is the id of the tag?");
                        input.clear();
                        stdin.read_line(&mut input)?;
                        let Ok(id) = input.trim().parse::<usize>() else {
                            println!("Invalid float!");
                            input.clear();
                            continue;
                        };
                        break id;
                    };
                    focal_length_estimate = Some(FocalLengthEstimate {
                        tag_distance,
                        width,
                        id,
                    });
                }
                "n" => focal_length_estimate = None,
                _ => continue,
            }
            break;
        }

        Ok((chessboard, estimate_fps, focal_length_estimate))
    });

    let (chessboard, estimate_fps, focal_length_estimate) = join.await.unwrap().unwrap();

    let mut rig = Robot::default();
    rig.add_center_element();
    let (mut elements, _) = rig
        .destructure::<BuildHasherDefault<DefaultHasher>>(["center"])
        .unwrap();
    let camera_element = elements.remove("center").unwrap();

    let mut camera_sub = Subscriber::new(1);
    accept_sub(camera_sub.create_subscription());

    app.add_task(
        move |context| async move {
            setup_logging!(context);

            let Some(img) = camera_sub.recv_or_closed().await else {
                return Err(anyhow::anyhow!("Camera did not produce any frames!"));
            };

            let mut camera_info = CameraInfo {
                width: img.width(),
                height: img.height(),
                fps: None,
                focal_length_px: None,
                distortion_data: None,
            };

            if chessboard {
                let mut object_point = VectorOfPoint3f::new();
                for y in 0..6 {
                    for x in 0..7 {
                        object_point.push(Point3f::new(x as f32, y as f32, 0.0));
                    }
                }
                let mut object_points = Vector::<VectorOfPoint3f>::new();
                let mut image_points = Vector::<VectorOfPoint2f>::new();

                let img_size = Size::new(img.width() as i32, img.height() as i32);
                let criteria =
                    TermCriteria::default().expect("Failed to generate default TermCriteria");

                for iteration in 0..10 {
                    println!("{iteration}: Finding chessboard corners");
                    loop {
                        let Some(img) = camera_sub.recv_or_closed().await else {
                            return Err(anyhow::anyhow!("Camera did not produce any frames!"));
                        };
                        let img = img.to_luma8();
                        let img = Mat::from_slice_rows_cols(
                            &img,
                            img.height() as usize,
                            img.width() as usize,
                        )
                        .expect("Image should have been converted into a matrix");

                        let mut corners = VectorOfPoint2f::new();
                        let success = find_chessboard_corners(
                            &img,
                            Size::new(7, 6),
                            &mut corners,
                            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE,
                        )
                        .expect("Failed to execute find_chessboard_corners");

                        if !success {
                            continue;
                        }

                        println!("{iteration}: Refining corners");
                        corner_sub_pix(
                            &img,
                            &mut corners,
                            Size::new(11, 11),
                            Size::new(-1, -1),
                            criteria,
                        )
                        .expect("Failed to execute corner_sub_pix");

                        object_points.push(object_point.clone());
                        image_points.push(corners);
                        break;
                    }
                }

                let mut camera_matrix =
                    Mat::from_slice_rows_cols(&[0, 0, 0, 0, 0, 0, 0, 0, 0], 3, 3).unwrap();
                let mut dist_coeffs = Vector::<f64>::new();
                let mut rvecs = VectorOfMat::new();
                let mut tvecs = VectorOfVec3d::new();

                println!("Calculating distortion");
                let err = calibrate_camera(
                    &object_points,
                    &image_points,
                    img_size,
                    &mut camera_matrix,
                    &mut dist_coeffs,
                    &mut rvecs,
                    &mut tvecs,
                    0,
                    criteria,
                )
                .expect("Failed to execute calibrate_camera");
                println!("RMS re-projection error: {err}");
                let mut roi = Rect::new(0, 0, img.width() as i32, img.height() as i32);
                let new_camera_matrix = get_optimal_new_camera_matrix(
                    &camera_matrix,
                    &dist_coeffs,
                    img_size,
                    1.0,
                    img_size,
                    Some(&mut roi),
                    false,
                )
                .expect("Failed to execute get_optimal_new_camera_matrix");

                let distortion_data = DistortionData {
                    distortion_coefficients: dist_coeffs.into_iter().collect(),
                    camera_matrix: array::from_fn(|i| *camera_matrix.at(i as i32).unwrap()),
                    new_camera_matrix: array::from_fn(|i| *new_camera_matrix.at(i as i32).unwrap()),
                    roi_x: roi.x as u32,
                    roi_y: roi.y as u32,
                    roi_width: roi.width as u32,
                    roi_height: roi.height as u32,
                };

                camera_info.distortion_data = Some(distortion_data.into());
            }

            if let Some(FocalLengthEstimate {
                tag_distance,
                width,
                id,
            }) = focal_length_estimate
            {
                let mut pose_sub = Subscriber::new(1);
                let mut length = img.width().max(img.height()) as f64 / 2.0;
                let mut close_enoughs = 0usize;
                let mut first = true;
                let mut fails = 0;

                loop {
                    let mut img_pub = Publisher::default();
                    let mut apriltag = AprilTagDetector::new(
                        length,
                        img.width(),
                        img.height(),
                        camera_element.get_ref(),
                    );
                    img_pub.accept_subscription(apriltag.create_image_subscription());

                    apriltag.add_tag(Default::default(), Default::default(), width, id);
                    apriltag
                        .tag_detected_pub()
                        .accept_subscription(pose_sub.create_subscription());
                    context.spawn_node(apriltag);

                    if first {
                        first = false;
                        println!("Waiting for initial apriltag observation...");
                        let img_fut = async {
                            loop {
                                img_pub.set(camera_sub.recv().await);
                            }
                        };
                        tokio::select! {
                            _ = img_fut => unreachable!(),
                            _ = pose_sub.recv() => {}
                        }
                    }
                    println!("Current length: {length:.2}");

                    let img_fut = async {
                        loop {
                            img_pub.set(camera_sub.recv().await);
                        }
                    };
                    let mut distance = 0.0;
                    let mut observations = 0usize;
                    let pose_fut = async {
                        loop {
                            let pose = pose_sub.recv().await;
                            distance += pose.pose.translation.vector.magnitude();
                            observations += 1;
                        }
                    };
                    tokio::select! {
                        _ = img_fut => unreachable!(),
                        _ = pose_fut => unreachable!(),
                        _ = tokio::time::sleep(Duration::from_secs(3)) => {}
                    }
                    if observations == 0 {
                        println!("Received no observations!");
                        fails += 1;
                        if fails == 3 {
                            println!("Resetting length...");
                            length = img.width().max(img.height()) as f64 / 2.0;
                        }
                        continue;
                    }
                    distance /= observations as f64;
                    let new_length = length * tag_distance / distance;

                    if (new_length - length).abs() < 1.0 {
                        close_enoughs += 1;

                        if close_enoughs >= 5 {
                            camera_info.focal_length_px =
                                Some(NonZeroUsize::new(new_length.round() as usize).unwrap());
                        }
                    } else {
                        close_enoughs = 0;
                    }

                    length = new_length;
                }
            }

            if estimate_fps {
                loop {
                    println!("Estimating fps across 5 seconds");
                    tokio::time::sleep(Duration::from_secs(2)).await;
                    let mut count = 0;
                    let fut = async {
                        loop {
                            let Some(_) = camera_sub.recv_or_closed().await else {
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

            println!("Finished examination of: {camera_name}");

            let join = tokio::task::spawn_blocking(move || {
                let stdin = stdin();
                let mut input = String::new();

                loop {
                    println!("Save file (Y/N)?");
                    input.clear();
                    stdin.read_line(&mut input).expect("Failed to read stdin");
                    match input.to_ascii_lowercase().trim() {
                        "y" => {
                            let mut submap = HashMap::with_capacity(1);
                            submap.insert(camera_name, camera_info);

                            std::fs::DirBuilder::new()
                                .recursive(true)
                                .create(DEFAULT_CAMERA_FOLDER)
                                .with_context(|| {
                                    format!("{DEFAULT_CAMERA_FOLDER} should be writable")
                                })
                                .unwrap();

                            for i in 0.. {
                                let path =
                                    Path::new(DEFAULT_CAMERA_FOLDER).join(format!("block{i}.json"));
                                let file = match std::fs::File::create(&path) {
                                    Ok(x) => x,
                                    Err(e) => match e.kind() {
                                        std::io::ErrorKind::AlreadyExists => todo!(),
                                        _ => continue,
                                    },
                                };
                                to_writer_pretty(file, &submap)
                                    .with_context(|| format!("{path:?} should be writable"))
                                    .unwrap();
                                break;
                            }

                            std::mem::forget(submap);
                        }
                        "n" => {}
                        _ => continue,
                    }
                    break;
                }
            });

            join.await.unwrap();

            Ok(())
        },
        "examiner",
    );
}
