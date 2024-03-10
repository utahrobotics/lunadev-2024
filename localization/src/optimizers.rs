use std::ops::DerefMut;

use rand_distr::{Distribution, Normal};
use unros::{rayon::{self, iter::ParallelIterator}, rng::quick_rng};

use crate::{random_unit_vector, DataBucket, DataPoint, Float};


pub trait Optimizer: Send + 'static {
    fn optimize(&mut self, old: &DataBucket, current: &mut DataBucket, new: &DataBucket);
}


pub trait StochasticOptimizer: Send + Sync + 'static {
    fn candidate_count(&self) -> usize;
    fn generate_candidate(&self, old: &DataBucket, current: &DataBucket, new: &DataBucket) -> DataPoint;
    fn score_candidate(&self, old: &DataBucket, current: &DataBucket, new: &DataBucket, candidate: &DataPoint) -> usize;
}


impl<T: StochasticOptimizer> Optimizer for T {
    fn optimize(&mut self, old: &DataBucket, current: &mut DataBucket, new: &DataBucket) {
        let Some((_, candidate)) = rayon::iter::repeatn((), self.candidate_count())
            .map(|_| {
                let candidate = self.generate_candidate(old, current, new);
                (self.score_candidate(old, current, new, &candidate), candidate)
            })
            .max_by_key(|(score, _)| *score) else { return; };
        current.point = candidate;
    }
}


pub struct SmoothnessOptimizer {
    pub candidate_count: usize,
    pub position_variance: Float,
    pub orientation_variance: Float
}


impl StochasticOptimizer for SmoothnessOptimizer {
    fn candidate_count(&self) -> usize {
        self.candidate_count
    }

    fn generate_candidate(&self, _old: &DataBucket, current: &DataBucket, _new: &DataBucket) -> DataPoint {
        let mut rng = quick_rng();
        let mut new_point = current.point;
        let distr = Normal::new(0.0, self.position_variance.sqrt()).unwrap();

        new_point.isometry.translation.vector += random_unit_vector(&mut rng).scale(distr.sample(rng.deref_mut()));

        new_point
    }

    fn score_candidate(&self, old: &DataBucket, current: &DataBucket, new: &DataBucket, candidate: &DataPoint) -> usize {
        todo!()
    }
}