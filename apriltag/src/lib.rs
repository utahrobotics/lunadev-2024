use std::sync::{mpsc, Arc};

use apriltag::{families::Tag16h5, DetectorBuilder, Image};
use apriltag_image::{
    image::{ImageBuffer, Luma, DynamicImage},
    ImageExt,
};
use unros_core::{
    anyhow, async_trait, node_info, tokio::runtime::Handle, tokio_rayon, Node, OwnedSignal, Signal,
};

#[derive(Clone, Copy)]
pub struct DetectedAprilTag {
    pub center: [f64; 2],
    pub decision_margin: f32,
    pub id: usize,
}

pub struct AprilTagDetector {
    name: String,
    image_sender: mpsc::SyncSender<ImageBuffer<Luma<u8>, Vec<u8>>>,
    image_receiver: mpsc::Receiver<ImageBuffer<Luma<u8>, Vec<u8>>>,
    tag_detected: Option<OwnedSignal<DetectedAprilTag>>,
}


impl AprilTagDetector {
    pub fn new() -> Self {
        let (image_sender, image_receiver) = mpsc::sync_channel(10);
        Self {
            name: "apriltag".into(),
            image_sender,
            image_receiver,
            tag_detected: Some(OwnedSignal::default())
        }
    }

    pub fn connect_from(&self, image_signal: &mut impl Signal<Arc<DynamicImage>>) {
        let sender = self.image_sender.clone();
        image_signal.connect_to(move |x| {
            let _ = sender.send(x.to_luma8());
        });
    }
}


#[async_trait]
impl Node for AprilTagDetector {
    fn set_name(&mut self, name: String) {
        self.name = name
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    async fn run(mut self) -> anyhow::Result<()> {
        let handle = Handle::current();

        tokio_rayon::spawn(move || {
            let signal = Arc::new(self.tag_detected.take().unwrap());
            let mut detector = DetectorBuilder::new()
                .add_family_bits(Tag16h5::default(), 1)
                .build()?;

            loop {
                let Ok(img) = self.image_receiver.recv() else {
                    break;
                };
                let img = Image::from_image_buffer(&img);

                for detection in detector.detect(&img) {
                    let detection = DetectedAprilTag {
                        center: detection.center(),
                        decision_margin: detection.decision_margin(),
                        id: detection.id(),
                    };
                    let signal = signal.clone();
                    handle.spawn(async move {
                        signal.emit(detection).await;
                    });
                }
            }

            node_info!(self, "No more image senders! Exiting...");
            Ok(())
        })
        .await
    }
}
