use std::{sync::{Arc, mpsc::sync_channel}, fmt::Display, f64::consts::PI};

use apriltag::{families::Tag16h5, DetectorBuilder, Image, TagParams};
use apriltag_image::{image::DynamicImage, ImageExt};
use apriltag_nalgebra::PoseExt;
use fxhash::FxHashMap;
use nalgebra::{Point3, UnitQuaternion};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{Signal, SignalRef, watched::WatchedSubscription},
    tokio_rayon, Node, RuntimeContext, tokio::{sync::mpsc::channel, self},
};

#[derive(Clone, Copy, Debug)]
pub struct PoseObservation {
    pub position: Point3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub decision_margin: f32
}


impl Display for PoseObservation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (mut r, mut p, mut y) = self.orientation.euler_angles();
        r *= 180.0 / PI;
        p *= 180.0 / PI;
        y *= 180.0 / PI;
        write!(f, "Observer pos: ({:.2}, {:.2}, {:.2}), roll: {r:.0}, pitch: {p:.0}, yaw: {y:.0}, margin: {:.0}", self.position.x, self.position.y, self.position.z, self.decision_margin)
    }
}


struct KnownTag {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,
    tag_params: TagParams,
}

pub struct AprilTagDetector {
    image_sub: WatchedSubscription<Arc<DynamicImage>>,
    tag_detected: Signal<PoseObservation>,
    known_tags: FxHashMap<usize, KnownTag>,
    focal_length_px: f64,
    image_width: u32,
    image_height: u32,
}

impl AprilTagDetector {
    pub fn new(focal_length_px: f64, image_width: u32, image_height: u32, image_sub: WatchedSubscription<Arc<DynamicImage>>) -> Self {
        Self {
            image_sub,
            tag_detected: Default::default(),
            known_tags: Default::default(),
            focal_length_px,
            image_width,
            image_height
        }
    }

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
                position: tag_position,
                orientation: tag_orientation,
                tag_params: TagParams { tagsize: tag_width, fx: self.focal_length_px, fy: self.focal_length_px, cx: self.image_width as f64 / 2.0, cy: self.image_height as f64 / 2.0 },
            },
        );
    }

    pub fn tag_detected_signal(&mut self) -> SignalRef<PoseObservation> {
        self.tag_detected.get_ref()
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

            loop {
                let img = match img_receiver.recv() {
                    Ok(x) => x,
                    Err(_) => break,
                };

                let img = img.to_luma8();
                if img.width() != self.image_width || img.height() != self.image_height {
                    error!("Received incorrectly sized image: {}x{}", img.width(), img.height());
                    continue;
                }
                let img = Image::from_image_buffer(&img);

                for detection in detector.detect(&img) {
                    let Some(known) = self.known_tags.get(&detection.id()) else {
                        continue;
                    };
                    let Some(tag_pose) = detection.estimate_tag_pose(&known.tag_params) else {
                        warn!("Failed to estimate pose of {}", detection.id());
                        continue;
                    };
                    let tag_pose = tag_pose.to_na();

                    self.tag_detected.set(PoseObservation {
                        position: (known.position.coords + tag_pose.translation.vector).into(),
                        orientation: tag_pose.rotation * known.orientation,
                        decision_margin: detection.decision_margin()
                    });
                }
            }
        });

        loop {
            tokio::select! {
                img = self.image_sub.wait_for_change() => {
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
