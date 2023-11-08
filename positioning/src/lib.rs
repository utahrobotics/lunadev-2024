use std::time::{Duration, Instant};

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use unros_core::{
    anyhow, async_trait, node_warn,
    tokio::{
        self,
        sync::mpsc::{Receiver, Sender, channel},
    },
    Node, PublicValue, Signal, OwnedWatchedPublicValue,
};

pub struct PositionFrame {
    position: Point3<f32>,
    variance: Matrix3<f32>,
}

pub struct IMUFrame {
    acceleration: Vector3<f32>,
    rotation: Vector3<f32>,
}

pub struct Positioner {
    name: String,
    pub builder: eskf::Builder,
    imu_frame_sender: Sender<IMUFrame>,
    imu_frame_receiver: Receiver<IMUFrame>,
    position_frame_sender: Sender<PositionFrame>,
    position_frame_receiver: Receiver<PositionFrame>,

    position: PublicValue<Point3<f32>>,
    velocity: PublicValue<Vector3<f32>>,
    orientation: PublicValue<UnitQuaternion<f32>>,
}

impl Default for Positioner {
    fn default() -> Self {
        let (imu_frame_sender, imu_frame_receiver) = channel(256);
        let (position_frame_sender, position_frame_receiver) = channel(256);

        Self {
            name: Default::default(),
            builder: Default::default(),
            imu_frame_sender,
            imu_frame_receiver,
            position_frame_sender,
            position_frame_receiver,
            position: PublicValue::new(Point3::new(0.0, 0.0, 0.0)),
            velocity: PublicValue::new(Vector3::new(0.0, 0.0, 0.0)),
            orientation: Default::default(),
        }
    }
}

impl Positioner {
    pub fn connect_imu_from(&self, signal: &mut impl Signal<IMUFrame>) {
        let imu_frame_sender = self.imu_frame_sender.clone();
        signal.connect_to(move |x| {
            let _ = imu_frame_sender.try_send(x);
        });
    }

    pub fn connect_position_from(&self, signal: &mut impl Signal<PositionFrame>) {
        let position_frame_sender = self.position_frame_sender.clone();
        signal.connect_to(move |x| {
            let _ = position_frame_sender.try_send(x);
        });
    }

    pub fn watch_position(&self) -> OwnedWatchedPublicValue<Point3<f32>> {
        self.position.watch()
    }

    pub fn watch_velocity(&self) -> OwnedWatchedPublicValue<Vector3<f32>> {
        self.velocity.watch()
    }

    pub fn watch_orientation(&self) -> OwnedWatchedPublicValue<UnitQuaternion<f32>> {
        self.orientation.watch()
    }
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

                    self.position.replace(eskf.position);
                    self.velocity.replace(eskf.velocity);
                    self.orientation.replace(eskf.orientation);
                }
                frame = self.position_frame_receiver.recv() => {
                    let frame = frame.unwrap();

                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        node_warn!(self, "Failed to observe position: {e:#?}");
                        continue;
                    }

                    self.position.replace(eskf.position);
                    self.velocity.replace(eskf.velocity);
                    self.orientation.replace(eskf.orientation);
                }
            }
        }
    }
}
