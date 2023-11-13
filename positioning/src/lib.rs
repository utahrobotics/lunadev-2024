use std::time::{Duration, Instant};

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{bounded::BoundedSubscription, Signal, SignalRef},
    tokio, Node, RuntimeContext,
};

#[derive(Clone, Copy)]
pub struct PositionFrame {
    pub position: Point3<f32>,
    pub variance: Matrix3<f32>,
}

#[derive(Clone, Copy)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion<f32>,
    pub variance: Matrix3<f32>,
}

#[derive(Clone, Copy)]
pub struct IMUFrame {
    pub acceleration: Vector3<f32>,
    /// XYZ rotation order
    pub angular_velocity: Vector3<f32>,

    pub acceleration_variance: Option<Vector3<f32>>,
    pub angular_velocity_variance: Option<Vector3<f32>>,
}

pub struct Positioner {
    pub builder: eskf::Builder,
    imu_sub: BoundedSubscription<IMUFrame, 8>,
    position_sub: BoundedSubscription<PositionFrame, 8>,
    orientation_sub: BoundedSubscription<OrientationFrame, 8>,

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
    pub fn add_imu_sub(&mut self, sub: BoundedSubscription<IMUFrame, 8>) {
        self.imu_sub += sub;
    }

    pub fn add_position_sub(&mut self, sub: BoundedSubscription<PositionFrame, 8>) {
        self.position_sub += sub;
    }

    pub fn add_orientation_sub(&mut self, sub: BoundedSubscription<OrientationFrame, 8>) {
        self.orientation_sub += sub;
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
    const DEFAULT_NAME: &'static str = "positioning";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let mut eskf = self.builder.build();
        let start = Instant::now();
        let mut last_elapsed = Duration::ZERO;

        let mut imu_sub = self.imu_sub.to_watched().await;
        let mut position_sub = self.position_sub.to_watched().await;
        let mut orientation_sub = self.orientation_sub.to_watched().await;

        loop {
            tokio::select! {
                mut frame = imu_sub.wait_for_change() => {
                    let now = start.elapsed();
                    let delta = now - last_elapsed;
                    frame.angular_velocity *= delta.as_secs_f32();

                    if let Some(acceleration_variance) = frame.acceleration_variance {
                        eskf.set_acceleration_variance(acceleration_variance);
                    }

                    if let Some(angular_velocity_variance) = frame.angular_velocity_variance {
                        eskf.set_rotational_variance(angular_velocity_variance);
                    }

                    eskf.predict(
                        frame.acceleration,
                        (UnitQuaternion::from_axis_angle(&Vector3::x_axis(), frame.angular_velocity.x) * 
                        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), frame.angular_velocity.y) * 
                        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), frame.angular_velocity.z) *
                        eskf.orientation).scaled_axis(),
                        delta
                    );
                    last_elapsed = now;

                    self.position.set(eskf.position);
                    self.velocity.set(eskf.velocity);
                    self.orientation.set(eskf.orientation);
                }
                frame = position_sub.wait_for_change() => {
                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        error!("Failed to observe position: {e:#?}");
                        continue;
                    }

                    self.position.set(eskf.position);
                    self.velocity.set(eskf.velocity);
                    self.orientation.set(eskf.orientation);
                }
                frame = orientation_sub.wait_for_change() => {
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
