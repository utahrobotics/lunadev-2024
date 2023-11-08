use std::time::{Instant, Duration};

use nalgebra::{Vector3, Matrix3, Point3};
use unros_core::{async_trait, Node, anyhow, tokio::{sync::mpsc::{Sender, Receiver}, self}, node_warn};


pub struct PositionFrame {
    position: Point3<f32>,
    variance: Matrix3<f32>
}


pub struct IMUFrame {
    acceleration: Vector3<f32>,
    rotation: Vector3<f32>
}


pub struct Positioner {
    name: String,
    pub builder: eskf::Builder,
    imu_frame_sender: Sender<IMUFrame>,
    imu_frame_receiver: Receiver<IMUFrame>,
    position_frame_sender: Sender<PositionFrame>,
    position_frame_receiver: Receiver<PositionFrame>,
}


impl Positioner {
    // pub fn connect_imu_from(&mut self)
}


#[async_trait]
impl Node for Positioner {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }
    fn get_name(&self) -> &str {
        &self.name
    }
    async fn run(mut self) -> anyhow::Result<()> {
        let mut eskf = self.builder.build();
        let start = Instant::now();
        let mut last_elapsed = Duration::ZERO;
        
        loop {
            tokio::select! {
                frame = self.imu_frame_receiver.recv() => {
                    let frame = frame.unwrap();
                    let now = start.elapsed();
                    eskf.predict(frame.acceleration, frame.rotation, now - last_elapsed);
                    last_elapsed = now;
                }
                frame = self.position_frame_receiver.recv() => {
                    let frame = frame.unwrap();

                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        node_warn!(self, "Failed to observe position: {e:#?}");
                    }
                }
            }
        }

        Ok(())
    }
}
