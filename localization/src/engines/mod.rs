use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use rig::RobotBaseRef;

use crate::Float;

// pub mod mcl;
pub mod window;

pub trait LocalizerEngine<N: Float>: Send + 'static {
    type Config: Send;
    fn from_config(config: &Self::Config, robot_base: RobotBaseRef) -> Self;

    fn observe_linear_acceleration(&mut self, accel: Vector3<N>, variance: N);
    fn observe_linear_velocity(&mut self, vel: Vector3<N>, variance: N);
    fn observe_position(&mut self, pos: Point3<N>, variance: N);
    fn observe_angular_velocity(&mut self, ang_vel: UnitQuaternion<N>, variance: N);
    fn observe_orientation(&mut self, orient: UnitQuaternion<N>, variance: N);
    fn no_observation(&mut self);

    fn get_isometry(&self) -> Isometry3<N>;
    fn get_linear_velocity(&self) -> Vector3<N>;
    fn get_linear_acceleration(&self) -> Vector3<N>;
    fn get_angular_velocity(&self) -> UnitQuaternion<N>;
}
