use std::time::{Duration, Instant};

pub use eskf;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use unros_core::{
    anyhow, async_trait, setup_logging,
    signal::{unbounded::UnboundedSubscription, Signal, SignalRef},
    tokio, Node, RuntimeContext,
};

#[derive(Clone, Copy)]
pub struct PositionFrame {
    pub position: Point3<f64>,
    pub variance: Matrix3<f64>,
}

#[derive(Clone, Copy)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion<f64>,
    pub variance: Matrix3<f64>,
}

#[derive(Clone, Copy)]
pub struct IMUFrame {
    pub acceleration: Vector3<f64>,
    /// XYZ rotation order
    pub angular_velocity: Vector3<f64>,

    pub acceleration_variance: Option<Vector3<f64>>,
    pub angular_velocity_variance: Option<Vector3<f64>>,
}

pub struct Positioner {
    pub builder: eskf::Builder,
    imu_sub: UnboundedSubscription<IMUFrame>,
    position_sub: UnboundedSubscription<PositionFrame>,
    orientation_sub: UnboundedSubscription<OrientationFrame>,

    position: Signal<Point3<f64>>,
    velocity: Signal<Vector3<f64>>,
    orientation: Signal<UnitQuaternion<f64>>,
}

impl Default for Positioner {
    fn default() -> Self {
        Self {
            builder: Default::default(),
            imu_sub: UnboundedSubscription::none(),
            position_sub: UnboundedSubscription::none(),
            orientation_sub: UnboundedSubscription::none(),

            position: Default::default(),
            velocity: Default::default(),
            orientation: Default::default(),
        }
    }
}

impl Positioner {
    pub fn add_imu_sub(&mut self, sub: UnboundedSubscription<IMUFrame>) {
        self.imu_sub += sub;
    }

    pub fn add_position_sub(&mut self, sub: UnboundedSubscription<PositionFrame>) {
        self.position_sub += sub;
    }

    pub fn add_orientation_sub(&mut self, sub: UnboundedSubscription<OrientationFrame>) {
        self.orientation_sub += sub;
    }

    pub fn get_position_signal(&mut self) -> SignalRef<Point3<f64>> {
        self.position.get_ref()
    }

    pub fn get_velocity_signal(&mut self) -> SignalRef<Vector3<f64>> {
        self.velocity.get_ref()
    }

    pub fn get_orientation_signal(&mut self) -> SignalRef<UnitQuaternion<f64>> {
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

        let mut imu_sub_counter = 0u64;
        let mut imu_sub = self.imu_sub.to_watched().await.map(move |x| {
            let out = (x, imu_sub_counter);
            imu_sub_counter += 1;
            out
        });
        let mut position_sub_counter = 0u64;
        let mut position_sub = self.position_sub.to_watched().await.map(move |x| {
            let out = (x, position_sub_counter);
            position_sub_counter += 1;
            out
        });
        let mut orientation_sub_counter = 0u64;
        let mut orientation_sub = self.orientation_sub.to_watched().await.map(move |x| {
            let out = (x, orientation_sub_counter);
            orientation_sub_counter += 1;
            out
        });

        let mut imu_recv_counter = 0u64;
        let mut position_recv_counter = 0u64;
        let mut orientation_recv_counter = 0u64;

        loop {
            tokio::select! {
                (mut frame, imu_sub_counter) = imu_sub.wait_for_change() => {
                    if imu_sub_counter != imu_recv_counter {
                        warn!("Lagged behind by {} imu frames", imu_recv_counter - imu_sub_counter);
                        imu_recv_counter = imu_sub_counter + 1;
                    } else {
                        imu_recv_counter += 1;
                    }

                    if let Some(acceleration_variance) = frame.acceleration_variance {
                        eskf.set_acceleration_variance(acceleration_variance);
                    }

                    let now = start.elapsed();
                    let delta = now - last_elapsed;

                    if let Some(angular_velocity_variance) = frame.angular_velocity_variance {
                        eskf.set_rotational_variance(eskf.orientation_uncertainty() + angular_velocity_variance * delta.as_secs_f64());
                    } else {
                        eskf.set_rotational_variance(eskf.orientation_uncertainty());
                    }

                    frame.angular_velocity *= delta.as_secs_f64();

                    eskf.gravity = eskf.orientation * Vector3::y_axis().into_inner() * -9.81;

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
                (frame, position_sub_counter) = position_sub.wait_for_change() => {
                    if position_sub_counter != position_recv_counter {
                        warn!("Lagged behind by {} position frames", position_sub_counter - position_recv_counter);
                        position_recv_counter = position_sub_counter + 1;
                    } else {
                        position_recv_counter += 1;
                    }

                    if let Err(e) = eskf.observe_position(frame.position, frame.variance) {
                        error!("Failed to observe position: {e:#?}");
                        continue;
                    }

                    self.position.set(eskf.position);
                    self.velocity.set(eskf.velocity);
                    self.orientation.set(eskf.orientation);
                }
                (frame, orientation_sub_counter) = orientation_sub.wait_for_change() => {
                    if orientation_sub_counter != orientation_recv_counter {
                        warn!("Lagged behind by {} orientation frames", imu_recv_counter - orientation_sub_counter);
                        orientation_recv_counter = orientation_sub_counter + 1;
                    } else {
                        orientation_recv_counter += 1;
                    }

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
