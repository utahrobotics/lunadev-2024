//! This crate provides a node that can identify apriltags
//! in images.

use std::{
    f64::consts::PI,
    fmt::{Debug, Display},
    sync::{mpsc::sync_channel, Arc},
    // time::Instant,
};

use apriltag_image::{image::DynamicImage, ImageExt};
use apriltag_inner::{families::Tag16h5, DetectorBuilder, Image, TagParams};
use apriltag_nalgebra::PoseExt;
use fxhash::FxHashMap;
use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use rig::RobotElementRef;
use unros_core::{
    anyhow, async_trait,
    pubsub::{Publisher, Subscriber, Subscription},
    setup_logging,
    tokio::{self, sync::mpsc::channel},
    tokio_rayon, Node, RuntimeContext,
};

/// An observation of the global orientation and position
/// of the camera that observed an apriltag.
#[derive(Clone)]
pub struct PoseObservation {
    pub pose: Isometry3<f64>,
    // pub velocity: Option<Vector3<f64>>,
    /// The goodness of an observation.
    ///
    /// This is a value generated by the apriltag detector.
    pub decision_margin: f32,
    pub robot_element: RobotElementRef,
}

impl Debug for PoseObservation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoseObservation")
            .field("pose", &self.pose)
            .field("decision_margin", &self.decision_margin)
            .finish()
    }
}

impl Display for PoseObservation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let axis = self
            .pose
            .rotation
            .axis()
            .unwrap_or_else(|| Vector3::x_axis());
        write!(f, "Observer pos: ({:.2}, {:.2}, {:.2}), axis: ({:.2}, {:.2}, {:.2}), angle: {:.0}, margin: {:.0}", self.pose.translation.x, self.pose.translation.y, self.pose.translation.z, axis.x, axis.y, axis.z, self.pose.rotation.angle() / PI * 180.0, self.decision_margin)
        // write!(f, "Observer pos: ({:.2}, {:.2}, {:.2}), quat: ({:.2}, {:.2}, {:.2}, {:.2}), margin: {:.0}", self.pose.translation.x, self.pose.translation.y, self.pose.translation.z, self.pose.rotation.w, self.pose.rotation.i, self.pose.rotation.j, self.pose.rotation.k, self.decision_margin)
    }
}

struct KnownTag {
    pose: Isometry3<f64>,
    tag_params: TagParams,
}

/// A Node that can detect apriltags in images.
///
/// Actual detection does not occur until the node
/// is running.
pub struct AprilTagDetector {
    image_sub: Subscriber<Arc<DynamicImage>>,
    tag_detected: Publisher<PoseObservation>,
    known_tags: FxHashMap<usize, KnownTag>,
    focal_length_px: f64,
    image_width: u32,
    image_height: u32,
    robot_element: RobotElementRef,
    pub velocity_window: usize,
}

impl AprilTagDetector {
    /// Creates a new detector for a specific camera.
    ///
    /// The given `image_sub` must produce images that
    /// have a width of `image_width` and a height of
    /// `image_height`, and the camera that produced it
    /// must have a focal length, in pixels, of `focal_length_px`.
    ///
    /// As such, it is strongly encouraged that the subscription
    /// should not be a sum of multiple subscriptions.
    pub fn new(
        focal_length_px: f64,
        image_width: u32,
        image_height: u32,
        robot_element: RobotElementRef,
    ) -> Self {
        Self {
            image_sub: Subscriber::default(),
            tag_detected: Default::default(),
            known_tags: Default::default(),
            focal_length_px,
            image_width,
            image_height,
            robot_element,
            velocity_window: 200,
        }
    }

    /// Add a tag to look out for.
    ///
    /// Orientations and positions should be
    /// in global space.
    pub fn add_tag(
        &mut self,
        tag_position: Point3<f64>,
        tag_orientation: UnitQuaternion<f64>,
        tag_width: f64,
        tag_id: usize,
    ) {
        self.known_tags.insert(
            tag_id,
            KnownTag {
                pose: Isometry3::from_parts(tag_position.into(), tag_orientation),
                tag_params: TagParams {
                    tagsize: tag_width,
                    fx: self.focal_length_px,
                    fy: self.focal_length_px,
                    cx: self.image_width as f64 / 2.0,
                    cy: self.image_height as f64 / 2.0,
                },
            },
        );
    }

    pub fn accept_tag_detected_sub(&mut self, sub: Subscription<PoseObservation>) {
        self.tag_detected.accept_subscription(sub);
    }

    pub fn create_image_subscription(&mut self) -> Subscription<Arc<DynamicImage>> {
        self.image_sub.create_subscription(4)
    }
}

#[async_trait]
impl Node for AprilTagDetector {
    const DEFAULT_NAME: &'static str = "apriltag";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        let (err_sender, mut err_receiver) = channel(1);
        let (img_sender, img_receiver) = sync_channel::<Arc<DynamicImage>>(0);

        let _ = tokio_rayon::spawn(move || {
            setup_logging!(context);

            macro_rules! unwrap {
                ($result: expr) => {
                    match $result {
                        Ok(x) => x,
                        Err(e) => {
                            let _ = err_sender.blocking_send(e.into());
                            return;
                        }
                    }
                };
            }
            let mut detector = unwrap!(DetectorBuilder::new()
                .add_family_bits(Tag16h5::default(), 1)
                .build());

            // let mut seen: FxHashMap<usize, (Instant, Vector3<f64>)> = FxHashMap::default();

            loop {
                let img = match img_receiver.recv() {
                    Ok(x) => x,
                    Err(_) => break,
                };

                let img = img.to_luma8();
                if img.width() != self.image_width || img.height() != self.image_height {
                    error!(
                        "Received incorrectly sized image: {}x{}",
                        img.width(),
                        img.height()
                    );
                    continue;
                }
                let img = Image::from_image_buffer(&img);

                for detection in detector.detect(&img) {
                    if detection.decision_margin() < 130.0 {
                        continue;
                    }
                    let Some(known) = self.known_tags.get(&detection.id()) else {
                        continue;
                    };
                    let Some(robot_pose) = detection.estimate_tag_pose(&known.tag_params) else {
                        warn!("Failed to estimate pose of {}", detection.id());
                        continue;
                    };

                    let mut robot_pose = robot_pose.to_na();

                    robot_pose.translation.vector = known.pose.translation.vector
                        + known.pose.rotation
                            * robot_pose.rotation.inverse()
                            * robot_pose.translation.vector;
                    robot_pose.rotation = known.pose.rotation
                        * UnitQuaternion::from_axis_angle(
                            &(robot_pose.rotation * Vector3::y_axis()),
                            PI,
                        )
                        * robot_pose.rotation;
                    // let velocity;

                    // if let Some((time, old_pos)) = seen.get_mut(&detection.id()) {
                    //     let elapsed = time.elapsed().as_millis();

                    //     if elapsed >= 100 {
                    //         if elapsed <= self.velocity_window as u128 {
                    //             velocity = Some(
                    //                 (robot_pose.translation.vector - *old_pos) * (1000.0 / elapsed as f64),
                    //             );
                    //         } else {
                    //             velocity = None;
                    //         }
                    //         *time = Instant::now();
                    //         *old_pos = robot_pose.translation.vector;
                    //     } else {
                    //         velocity = None;
                    //     }
                    // } else {
                    //     seen.insert(detection.id(), (Instant::now(), robot_pose.translation.vector));
                    //     velocity = None;
                    // }

                    self.tag_detected.set(PoseObservation {
                        pose: robot_pose,
                        // velocity,
                        decision_margin: detection.decision_margin(),
                        robot_element: self.robot_element.clone(),
                    });
                }
            }
        });

        loop {
            tokio::select! {
                img = self.image_sub.recv() => {
                    let _ = img_sender.try_send(img);
                }
                result = err_receiver.recv() => {
                    let e = result.unwrap();
                    break Err(e);
                }
            }
        }
    }
}
