use std::ops::DerefMut;

use rand_distr::{Distribution, Normal};
use rig::RobotElementRef;
use unros::rng::quick_rng;

use crate::{random_unit_vector, Float, Point3, UnitQuaternion, Vector3};


/// A position and variance measurement.
#[derive(Clone)]
pub struct PositionFrame {
    /// Position in meters
    pub position: Point3,
    /// Variance centered around position in meters
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

impl PositionFrame {
    pub fn rand(position: Point3, variance: Float, robot_element: RobotElementRef) -> Self {
        let mut rng = quick_rng();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev).unwrap();

        Self {
            position: position
                + random_unit_vector(rng.deref_mut()).scale(distr.sample(rng.deref_mut())),
            variance,
            robot_element,
        }
    }
}

/// A position and variance measurement.
#[derive(Clone)]
pub struct VelocityFrame {
    /// Velocity in meters
    pub velocity: Vector3,
    /// Variance centered around velocity in meters per second
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

impl VelocityFrame {
    pub fn rand(velocity: Vector3, variance: Float, robot_element: RobotElementRef) -> Self {
        let mut rng = quick_rng();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev).unwrap();

        Self {
            velocity: velocity
                + random_unit_vector(rng.deref_mut()).scale(distr.sample(rng.deref_mut())),
            variance,
            robot_element,
        }
    }
}

/// An orientation and variance measurement.
#[derive(Clone)]
pub struct OrientationFrame {
    pub orientation: UnitQuaternion,
    /// Variance of orientation in radians
    pub variance: Float,
    pub robot_element: RobotElementRef,
}

impl OrientationFrame {
    pub fn rand(
        orientation: UnitQuaternion,
        variance: Float,
        robot_element: RobotElementRef,
    ) -> Self {
        let mut rng = quick_rng();
        let std_dev = variance.sqrt();
        let distr = Normal::new(0.0, std_dev).unwrap();

        Self {
            orientation: UnitQuaternion::from_axis_angle(
                &random_unit_vector(rng.deref_mut()),
                distr.sample(rng.deref_mut()),
            ) * orientation,
            variance,
            robot_element,
        }
    }
}

/// A measurement from an IMU.
#[derive(Clone)]
pub struct IMUFrame {
    pub acceleration: Vector3,
    /// Variance centered around acceleration in meters per second^2
    pub acceleration_variance: Float,

    pub angular_velocity: UnitQuaternion,
    /// Variance of angular_velocity in radians per second
    pub angular_velocity_variance: Float,

    pub robot_element: RobotElementRef,
}

impl IMUFrame {
    pub fn rand(
        acceleration: Vector3,
        acceleration_variance: Float,
        angular_velocity: UnitQuaternion,
        angular_velocity_variance: Float,
        robot_element: RobotElementRef,
    ) -> Self {
        let mut rng = quick_rng();
        let accel_std_dev = acceleration_variance.sqrt();
        let accel_distr = Normal::new(0.0, accel_std_dev).unwrap();
        let angular_velocity_std_dev = angular_velocity_variance.sqrt();
        let ang_vel_distr = Normal::new(0.0, angular_velocity_std_dev).unwrap();

        IMUFrame {
            acceleration: acceleration
                + random_unit_vector(rng.deref_mut()).scale(accel_distr.sample(rng.deref_mut())),
            angular_velocity: UnitQuaternion::from_axis_angle(
                &random_unit_vector(rng.deref_mut()),
                ang_vel_distr.sample(rng.deref_mut()),
            ) * angular_velocity,
            acceleration_variance,
            angular_velocity_variance,
            robot_element,
        }
    }
}