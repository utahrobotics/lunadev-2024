use std::ops::DerefMut;

use nalgebra::{convert as nconvert, Point3, UnitQuaternion, Vector3};
use rand_distr::{Distribution, Normal};
use rig::RobotElementRef;
use unros::rng::quick_rng;

use crate::{random_unit_vector, utils::Float};

/// A position and variance measurement.
#[derive(Clone)]
pub struct PositionFrame<N: Float> {
    /// Position in meters
    pub position: Point3<N>,
    /// Variance centered around position in meters
    pub variance: N,
    pub robot_element: RobotElementRef,
}

impl<N: Float> PositionFrame<N> {
    pub fn rand(position: Point3<N>, variance: N, robot_element: RobotElementRef) -> Self {
        let mut rng = quick_rng();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();

        Self {
            position: position
                + random_unit_vector(rng.deref_mut())
                    .scale(nconvert(distr.sample(rng.deref_mut()))),
            variance,
            robot_element,
        }
    }
}

/// A position and variance measurement.
#[derive(Clone)]
pub struct VelocityFrame<N: Float> {
    /// Velocity in meters
    pub velocity: Vector3<N>,
    /// Variance centered around velocity in meters per second
    pub variance: N,
    pub robot_element: RobotElementRef,
}

impl<N: Float> VelocityFrame<N> {
    pub fn rand(velocity: Vector3<N>, variance: N, robot_element: RobotElementRef) -> Self {
        let mut rng = quick_rng();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();

        Self {
            velocity: velocity
                + random_unit_vector(rng.deref_mut())
                    .scale(nconvert(distr.sample(rng.deref_mut()))),
            variance,
            robot_element,
        }
    }
}

/// An orientation and variance measurement.
#[derive(Clone)]
pub struct OrientationFrame<N: Float> {
    pub orientation: UnitQuaternion<N>,
    /// Variance of orientation in radians
    pub variance: N,
    pub robot_element: RobotElementRef,
}

impl<N: Float> OrientationFrame<N> {
    pub fn rand(
        orientation: UnitQuaternion<N>,
        variance: N,
        robot_element: RobotElementRef,
    ) -> Self {
        let mut rng = quick_rng();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev.to_f32()).unwrap();

        Self {
            orientation: UnitQuaternion::from_axis_angle(
                &random_unit_vector(rng.deref_mut()),
                nconvert(distr.sample(rng.deref_mut())),
            ) * orientation,
            variance,
            robot_element,
        }
    }
}

/// A measurement from an IMU.
#[derive(Clone)]
pub struct IMUFrame<N: Float> {
    pub acceleration: nalgebra::Vector3<N>,
    /// Variance centered around acceleration in meters per second^2
    pub acceleration_variance: N,

    pub angular_velocity: nalgebra::UnitQuaternion<N>,
    /// Variance of angular_velocity in radians per second
    pub angular_velocity_variance: N,

    pub robot_element: RobotElementRef,
}

impl<N: Float> IMUFrame<N> {
    pub fn rand(
        acceleration: Vector3<N>,
        acceleration_variance: N,
        angular_velocity: UnitQuaternion<N>,
        angular_velocity_variance: N,
        robot_element: RobotElementRef,
    ) -> Self {
        let mut rng = quick_rng();
        let accel_std_dev = acceleration_variance.sqrt();
        let accel_distr = Normal::new(0.0, accel_std_dev.to_f32()).unwrap();
        let angular_velocity_std_dev = angular_velocity_variance.sqrt();
        let ang_vel_distr = Normal::new(0.0, angular_velocity_std_dev.to_f32()).unwrap();

        IMUFrame {
            acceleration: acceleration
                + random_unit_vector(rng.deref_mut())
                    .scale(nconvert(accel_distr.sample(rng.deref_mut()))),
            angular_velocity: UnitQuaternion::from_axis_angle(
                &random_unit_vector(rng.deref_mut()),
                nconvert(ang_vel_distr.sample(rng.deref_mut())),
            ) * angular_velocity,
            acceleration_variance,
            angular_velocity_variance,
            robot_element,
        }
    }
}
