//! This crate provides a node that can digest multiple streams
//! of spatial input to determine where an object (presumably a
//! robot) is in global space.

use std::
    collections::hash_map::Entry;

use fxhash::FxHashMap;
use nalgebra::{
    convert as nconvert, Isometry3, UnitQuaternion, UnitVector3, Vector3
};
use rig::RobotElementRef;
use smach::StateResult;
use unros::{
    setup_logging, tokio,
};

use crate::{CalibratedImu, CalibratingImu, Float, LocalizerBlackboard};


/// The calibration stage of the localizer.
///
/// This stage runs for `calibration_duration` before applying the calibrations and exiting.
pub(super) async fn calibrate_localizer<N: Float>(
    mut bb: LocalizerBlackboard<N>,
) -> StateResult<LocalizerBlackboard<N>> {
    let context = bb.context.unwrap();
    setup_logging!(context);
    info!("Calibrating localizer");

    let mut imu_map = FxHashMap::<RobotElementRef, CalibratingImu<N>>::default();
    let mut total_gravity = Vector3::default();

    tokio::select! {
        _ = tokio::time::sleep(bb.calibration_duration) => {}
        _ = async { loop {
            let imu = bb.imu_sub.recv().await;
            total_gravity += imu.acceleration;
            let isometry: Isometry3<N> = nconvert(imu.robot_element.get_isometry_from_base());

            match imu_map.entry(imu.robot_element) {
                Entry::Occupied(mut x) => {
                    let x = x.get_mut();
                    x.count += 1;
                    x.accel += isometry * imu.acceleration;
                    x.angular_velocity = imu.angular_velocity * x.angular_velocity;
                }
                Entry::Vacant(x) => {
                    x.insert(CalibratingImu { count: 1, accel: imu.acceleration, angular_velocity: imu.angular_velocity });
                }
            }
        }} => {}
    }

    bb.calibrations = imu_map
        .into_iter()
        .map(|(robot_element, calibrating)| {
            let mut accel_correction = UnitQuaternion::from_axis_angle(
                &UnitVector3::new_normalize(calibrating.accel.cross(&total_gravity)),
                calibrating.accel.angle(&total_gravity),
            );

            if accel_correction.w.is_nan()
                || accel_correction.i.is_nan()
                || accel_correction.j.is_nan()
                || accel_correction.k.is_nan()
            {
                accel_correction = Default::default();
            }

            let calibrated = CalibratedImu {
                accel_scale: nconvert::<_, N>(9.81) / calibrating.accel.magnitude()
                    * nconvert(calibrating.count),
                accel_correction,
                angular_velocity_bias: UnitQuaternion::default()
                    .try_slerp(
                        &calibrating.angular_velocity,
                        N::one() / nconvert(calibrating.count),
                        nconvert(0.01),
                    )
                    .unwrap_or_default(),
            };

            (robot_element, calibrated)
        })
        .collect();

    bb.recalibrate_sub.try_recv();
    bb.start_orientation = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(total_gravity.cross(&-Vector3::y_axis())),
        total_gravity.angle(&-Vector3::y_axis()),
    );
    if !bb.start_orientation.w.is_finite()
        || !bb.start_orientation.i.is_finite()
        || !bb.start_orientation.j.is_finite()
        || !bb.start_orientation.k.is_finite()
    {
        bb.start_orientation = Default::default();
    }
    info!("Localizer calibrated");
    bb.context = Some(context);
    bb.into()
}