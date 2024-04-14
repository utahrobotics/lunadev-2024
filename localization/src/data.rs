// use nalgebra::Vector3;
use rand::prelude::SliceRandom;
use rand::{distributions::Distribution, thread_rng, Rng};
use rand_distr::Normal;
use smartcore::linalg::basic::matrix::DenseMatrix;
// use serde::{Deserialize, Serialize};
// use serde_big_array::BigArray;

const SEQ_LENGTH: usize = localization::OBSERVATIONS;
const STD_DEV_COUNT: usize = 3;
const MAX_STD_DEV: f32 = 0.4;
const MAX_DELTA: f32 = 0.1;

#[derive(Clone, Copy, Debug)]
pub struct TrainingItem {
    // #[serde(with = "BigArray")]
    pub input: [(f32, f32, f32); SEQ_LENGTH],
    pub target: f32,
}

pub fn gen() -> TrainingItem {
    let mut rng = thread_rng();
    let mut item = TrainingItem {
        input: [(0.0, 0.0, 0.0); SEQ_LENGTH],
        target: 0.0,
    };

    let mut origin = 0.0;
    let mut velocity = rng.gen_range(-1.0..1.0);

    let mut i = 0usize;

    let mut std_devs = vec![Normal::new(0.0, 0.0).unwrap()];
    for _ in 0..STD_DEV_COUNT {
        std_devs.push(Normal::new(0.0, rng.gen_range(0.0..MAX_STD_DEV)).unwrap());
    }
    while i < SEQ_LENGTH {
        let next_velocity = rng.gen_range(-1.0..1.0);
        let seq_length = rng.gen_range(0..SEQ_LENGTH);
        let accel = (next_velocity - velocity) / seq_length as f32;
        for _ in 0..seq_length {
            let normal = std_devs.choose(&mut rng).unwrap();
            let delta = rng.gen_range(0.0..MAX_DELTA);

            if i == 0 {
                item.input[0] = (
                    0.0,
                    normal.std_dev(),
                    0.0
                );
                i += 1;
                continue;
            }

            origin += velocity + accel / 2.0 * delta;
            velocity += accel * delta;

            item.input[i] = (
                origin + normal.sample(&mut rng),
                normal.std_dev(),
                delta
            );

            i += 1;
            if i >= SEQ_LENGTH {
                break;
            }
        }
    }

    item.target = origin;
    item
}


pub fn gen_rf_data(len: usize) -> (DenseMatrix<f32>, Vec<f32>) {
    let mut data: Vec<Vec<f32>> = Vec::new();
    let mut target: Vec<f32> = Vec::new();
    for _ in 0..len {
        let item = gen();
        data.push(
            item.input
                .into_iter()
                .flat_map(|(v, f, d)| [v, f, d])
                .collect(),
        );
        
        target.push(item.target);
    }
    (DenseMatrix::from_2d_vec(&data), target)
}