use std::ops::DerefMut;

use fxhash::FxHashMap;
use nalgebra::UnitQuaternion;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use rig::RobotElementRef;
use unros::{
    rayon::{self, iter::ParallelIterator},
    rng::quick_rng,
};

use crate::{normal, random_unit_vector, CalibratedImu, DataBucket, DataPoint, Float, G_VEC};

pub trait Optimizer: Send + 'static {
    fn optimize(
        &mut self,
        old: &DataBucket,
        current: &mut DataBucket,
        new: &DataBucket,
        delta: Float,
        imu_calibs: &FxHashMap<RobotElementRef, CalibratedImu>,
    );
}

pub trait StochasticOptimizer: Send + Sync + 'static {
    fn candidate_count(&self) -> usize;
    fn generate_candidate(
        &self,
        old: &DataBucket,
        current: &DataBucket,
        new: &DataBucket,
        delta: Float,
        imu_calibs: &FxHashMap<RobotElementRef, CalibratedImu>,
    ) -> DataPoint;
    fn score_candidate(
        &self,
        old: &DataBucket,
        current: &DataBucket,
        new: &DataBucket,
        candidate: &DataPoint,
        imu_calibs: &FxHashMap<RobotElementRef, CalibratedImu>,
    ) -> usize;
}

impl<T: StochasticOptimizer> Optimizer for T {
    fn optimize(
        &mut self,
        old: &DataBucket,
        current: &mut DataBucket,
        new: &DataBucket,
        delta: Float,
        imu_calibs: &FxHashMap<RobotElementRef, CalibratedImu>,
    ) {
        let Some((_, candidate)) = rayon::iter::repeatn((), self.candidate_count())
            .map(|_| {
                let candidate = self.generate_candidate(old, current, new, delta, imu_calibs);
                (
                    self.score_candidate(old, current, new, &candidate, imu_calibs),
                    candidate,
                )
            })
            .max_by_key(|(score, _)| *score)
        else {
            return;
        };
        current.point = candidate;
    }
}

pub struct StochasticSmoothnessOptimizer {
    pub candidate_count: usize,
    pub position_variance: Float,
    pub orientation_variance: Float,
}

impl StochasticOptimizer for StochasticSmoothnessOptimizer {
    fn candidate_count(&self) -> usize {
        self.candidate_count
    }

    fn generate_candidate(
        &self,
        _old: &DataBucket,
        current: &DataBucket,
        new: &DataBucket,
        delta: Float,
        _imu_calibs: &FxHashMap<RobotElementRef, CalibratedImu>,
    ) -> DataPoint {
        let mut rng = quick_rng();
        let mut candidate = current.point;
        let pos_distr = Normal::new(0.0, self.position_variance.sqrt()).unwrap();

        candidate.isometry.translation.vector +=
            random_unit_vector(&mut rng).scale(pos_distr.sample(rng.deref_mut()));

        let travel = new.point.isometry.translation.vector - candidate.isometry.translation.vector;
        candidate.linear_velocity = travel.scale(2.0 / delta) - new.point.linear_velocity;
        candidate.acceleration =
            (new.point.linear_velocity - candidate.linear_velocity).unscale(delta) + G_VEC;

        let rot_distr = Normal::new(0.0, self.orientation_variance.sqrt()).unwrap();
        candidate.isometry.rotation = candidate.isometry.rotation.append_axisangle_linearized(
            &random_unit_vector(&mut rng).scale(rot_distr.sample(rng.deref_mut())),
        );

        let travel = candidate
            .isometry
            .rotation
            .rotation_to(&new.point.isometry.rotation);
        candidate.angular_velocity = UnitQuaternion::new(travel.scaled_axis().unscale(delta));

        if rng.gen_bool(0.00002) {
            println!("{candidate:?}");
        }

        candidate
    }

    fn score_candidate(
        &self,
        _old: &DataBucket,
        current: &DataBucket,
        _new: &DataBucket,
        candidate: &DataPoint,
        imu_calibs: &FxHashMap<RobotElementRef, CalibratedImu>,
    ) -> usize {
        let mut score: Float = 0.0;

        for pos_frame in &current.position_frames {
            score += normal(
                0.0,
                pos_frame.variance.sqrt(),
                (candidate.isometry.translation.vector - pos_frame.position.coords).magnitude(),
            );
        }

        for vel_frame in &current.velocity_frames {
            score += normal(
                0.0,
                vel_frame.variance.sqrt(),
                (candidate.linear_velocity - vel_frame.velocity).magnitude(),
            );
        }

        for rot_frame in &current.orientation_frames {
            score += normal(
                0.0,
                rot_frame.variance.sqrt(),
                candidate.isometry.rotation.angle_to(&rot_frame.orientation),
            );
        }

        for mut imu_frame in current.imu_frames.iter().cloned() {
            if let Some(calibration) = imu_calibs.get(&imu_frame.robot_element) {
                imu_frame.angular_velocity =
                    calibration.angular_velocity_bias.inverse() * imu_frame.angular_velocity;
                imu_frame.acceleration = candidate.isometry.rotation
                    * imu_frame.robot_element.get_isometry_from_base().rotation
                    * calibration.accel_correction
                    * imu_frame.acceleration
                    * calibration.accel_scale;
            } else {
                imu_frame.acceleration = candidate.isometry.rotation
                    * imu_frame.robot_element.get_isometry_from_base().rotation
                    * imu_frame.acceleration;
            }

            score += normal(
                0.0,
                imu_frame.acceleration_variance.sqrt(),
                (candidate.acceleration - imu_frame.acceleration).magnitude(),
            );
            score += normal(
                0.0,
                imu_frame.angular_velocity_variance.sqrt(),
                candidate
                    .angular_velocity
                    .angle_to(&imu_frame.angular_velocity),
            );
        }

        // let mut diff: Float = 0.0;

        score.round() as usize
    }
}
