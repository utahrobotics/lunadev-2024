use std::{num::NonZeroU32, sync::Arc};

pub use apriltag::TagParams;
use apriltag::{families::Tag16h5, DetectorBuilder, Image};
use apriltag_image::{image::DynamicImage, ImageExt};
use apriltag_nalgebra::PoseExt;
use fxhash::FxHashMap;
use nalgebra::{Point3, UnitQuaternion};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{bounded::BoundedSubscription, Signal, SignalRef},
    tokio_rayon, Node, RuntimeContext,
};

#[derive(Clone, Copy)]
pub struct PoseObservation {
    pub position: Point3<f64>,
    pub orientation: UnitQuaternion<f64>,
}

struct KnownTag {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,
    tag_params: TagParams,
}

pub struct AprilTagDetector {
    image_sub: BoundedSubscription<Arc<DynamicImage>>,
    tag_detected: Signal<PoseObservation>,
    known_tags: FxHashMap<usize, KnownTag>,
}

impl AprilTagDetector {
    pub fn new() -> Self {
        Self {
            image_sub: BoundedSubscription::none(),
            tag_detected: Default::default(),
            known_tags: Default::default(),
        }
    }

    pub fn add_tag(
        &mut self,
        tag_position: Point3<f64>,
        tag_orientation: UnitQuaternion<f64>,
        tag_params: TagParams,
        tag_id: usize,
    ) {
        self.known_tags.insert(
            tag_id,
            KnownTag {
                position: tag_position,
                orientation: tag_orientation,
                tag_params,
            },
        );
    }

    pub fn subscribe_to_image(&mut self, signal: &mut SignalRef<Arc<DynamicImage>>) {
        self.image_sub += signal.subscribe_bounded(NonZeroU32::new(8).unwrap());
    }
}

#[async_trait]
impl Node for AprilTagDetector {
    const DEFAULT_NAME: &'static str = "apriltag";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        tokio_rayon::spawn(move || {
            let mut detector = DetectorBuilder::new()
                .add_family_bits(Tag16h5::default(), 1)
                .build()?;

            loop {
                let img = match self.image_sub.blocking_recv() {
                    Some(Ok(x)) => x,
                    Some(Err(n)) => {
                        warn!("Lagged behind by {n} frames");
                        continue;
                    }
                    None => break,
                };
                let img = Image::from_image_buffer(&img.to_luma8());

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
                        position: (known.position.coords - tag_pose.translation.vector).into(),
                        orientation: tag_pose.rotation * known.orientation,
                    });
                }
            }

            warn!("No more image senders! Exiting...");
            Ok(())
        })
        .await
    }
}
