//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use nalgebra::{convert as nconvert, Isometry3, UnitQuaternion};
use smach::StateResult;
use unros::{setup_logging, tokio};

use crate::{engines::LocalizerEngine, Float, LocalizerBlackboard};

/// The active stage of the localizer.  
/// During this stage, the localizer accepts observations and updates its estimate of the robot's Isometry.
///
/// If recalibration is triggered, this stage exits. Otherwise, this stage runs forever.
pub(super) async fn run_localizer<N: Float, E: LocalizerEngine<N>>(
    mut bb: LocalizerBlackboard<N, E>,
) -> StateResult<LocalizerBlackboard<N, E>> {
    let context = bb.context.unwrap();
    setup_logging!(context);

    let mut engine = E::from_config(&bb.engine_config, bb.robot_base.get_ref());

    loop {
        // Simultaneously watch three different subscriptions at once.
        // 1. IMU observations
        // 2. Position observations
        // 3. Orientation observations
        tokio::select! {
            // Check for recalibration while simultaneously feeding observations into the algorithm
            () = bb.recalibrate_sub.recv() => {
                break;
            }
            // Process system if max_delta time has passed and no observations were received
            () = tokio::time::sleep(bb.max_delta) => {
                engine.no_observation();
            }
            // IMU Observations
            mut frame = bb.imu_sub.recv() => {
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                let mut ang_scaled_axis = frame.angular_velocity.scaled_axis();
                ang_scaled_axis = nconvert::<_, UnitQuaternion<N>>(inv_rotation) * ang_scaled_axis;
                frame.angular_velocity = UnitQuaternion::from_scaled_axis(ang_scaled_axis);

                frame.acceleration = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_global_isometry().rotation) * frame.acceleration;

                let calibration = bb.calibrations.get(&frame.robot_element);

                if let Some(calibration) = &calibration {
                    frame.angular_velocity = calibration.angular_velocity_bias.inverse() * frame.angular_velocity;
                    frame.acceleration = calibration.accel_correction * frame.acceleration * calibration.accel_scale;
                };

                engine.observe_linear_acceleration(frame.acceleration, frame.acceleration_variance);
                engine.observe_angular_velocity(frame.angular_velocity, frame.angular_velocity_variance);
            }

            // Position Observations
            mut frame = bb.position_sub.recv() => {
                // Find the position of the robot base based on the observation of the position of an element
                // attached to the robot base.
                let isometry = frame.robot_element.get_isometry_from_base().inverse();
                frame.position = nconvert::<_, Isometry3<N>>(isometry) * frame.position;

                engine.observe_position(frame.position, frame.variance);
            }

            // Velocity Observations
            mut frame = bb.velocity_sub.recv() => {
                // Find the velocity of the robot base based on the observation of the velocity of an element
                // attached to the robot base.
                frame.velocity = nconvert::<_, UnitQuaternion<N>>(frame.robot_element.get_isometry_from_base().rotation) * frame.velocity;
                engine.observe_linear_velocity(frame.velocity, frame.variance);
            }

            // Orientation Observations
            mut frame = bb.orientation_sub.recv() => {
                // Find the orientation of the robot base based on the observation of the orientation of an element
                // attached to the robot base.
                let inv_rotation = frame.robot_element.get_isometry_from_base().rotation.inverse();
                frame.orientation = nconvert::<_, UnitQuaternion<N>>(inv_rotation) * frame.orientation;

                engine.observe_orientation(frame.orientation, frame.variance);
            }
        }

        bb.robot_base.set_isometry(nconvert(engine.get_isometry()));
        bb.robot_base
            .set_linear_velocity(nconvert(engine.get_linear_velocity()));
    }
    bb.context = Some(context);
    bb.into()
}
