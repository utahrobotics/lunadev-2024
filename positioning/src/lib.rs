use std::{
    num::NonZeroU32,
    time::{Duration, Instant},
};

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{bounded::BoundedSubscription, Signal, SignalRef},
    tokio, Node, RuntimeContext,
};

#[derive(Clone, Copy)]
pub struct PositionFrame {
    position: Point3<f32>,
    variance: Matrix3<f32>,
}

#[derive(Clone, Copy)]
pub struct OrientationFrame {
    orientation: UnitQuaternion<f32>,
    variance: Matrix3<f32>,
}

#[derive(Clone, Copy)]
pub struct IMUFrame {
    acceleration: Vector3<f32>,
    rotation: Vector3<f32>,
}

pub struct Positioner {
    pub builder: eskf::Builder,
    imu_sub: BoundedSubscription<IMUFrame>,
    position_sub: BoundedSubscription<PositionFrame>,
    orientation_sub: BoundedSubscription<OrientationFrame>,

    position: Signal<Point3<f32>>,
    velocity: Signal<Vector3<f32>>,
    orientation: Signal<UnitQuaternion<f32>>,
}

impl Default for Positioner {
    fn default() -> Self {
        Self {
            builder: Default::default(),
            imu_sub: BoundedSubscription::none(),
            position_sub: BoundedSubscription::none(),
            orientation_sub: BoundedSubscription::none(),

            position: Default::default(),
            velocity: Default::default(),
            orientation: Default::default(),
        }
    }
}

impl Positioner {
    pub fn add_imu_sub(&mut self, signal: &mut SignalRef<IMUFrame>) {
        self.imu_sub += signal.subscribe_bounded(NonZeroU32::new(8).unwrap());
    }

    pub fn add_position_sub(&mut self, signal: &mut SignalRef<PositionFrame>) {
        self.position_sub += signal.subscribe_bounded(NonZeroU32::new(8).unwrap());
    }

    pub fn add_orientation_sub(&mut self, signal: &mut SignalRef<OrientationFrame>) {
        self.orientation_sub += signal.subscribe_bounded(NonZeroU32::new(8).unwrap());
    }

    pub fn get_position_signal(&mut self) -> SignalRef<Point3<f32>> {
        self.position.get_ref()
    }

    pub fn get_velocity_signal(&mut self) -> SignalRef<Vector3<f32>> {
        self.velocity.get_ref()
    }

    pub fn get_orientation_signal(&mut self) -> SignalRef<UnitQuaternion<f32>> {
        self.orientation.get_ref()
    }
}

#[async_trait]
impl Node for Positioner {
    const DEFAULT_NAME: &'static str = "realsense";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let mut eskf = self.builder.build();
        let start = Instant::now();
        let mut last_elapsed = Duration::ZERO;

        loop {
            tokio::select! {
                result = self.imu_sub.recv() => {
                    let frame = match result {
                        Ok(x) => x,
                        Err(n) => {
                            warn!("Lagged behind by {n} frames");
                            continue;
                        }
                    };
                    let now = start.elapsed();
                    eskf.predict(frame.acceleration, frame.rotation, now - last_elapsed);
                    last_elapsed = now;

                    self.position.set(eskf.position);
                    self.velocity.set(eskf.velocity);
                    self.orientation.set(eskf.orientation);
                }
                result = self.position_sub.recv() => {
                    let frame = match result {
                        Ok(x) => x,
                        Err(n) => {
                            warn!("Lagged behind by {n} frames");
                            continue;
                        }
                    };

                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        error!("Failed to observe position: {e:#?}");
                        continue;
                    }

                    self.position.set(eskf.position);
                    self.velocity.set(eskf.velocity);
                    self.orientation.set(eskf.orientation);
                }
                result = self.orientation_sub.recv() => {
                    let frame = match result {
                        Ok(x) => x,
                        Err(n) => {
                            warn!("Lagged behind by {n} frames");
                            continue;
                        }
                    };

                    if let Err(e) = eskf.observe_orientation(frame.orientation, frame.variance) {
                        error!("Failed to observe orientation: {e:#?}");
                        continue;
                    }

                    self.position.set(eskf.position);
                    self.velocity.set(eskf.velocity);
                    self.orientation.set(eskf.orientation);
                }
            }
        }
    }
}
