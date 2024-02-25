// #![feature(iter_array_chunks)]

pub mod drive;

use std::collections::VecDeque;

use crossbeam::queue::SegQueue;
use machine_academy::burn::tensor::{backend::Backend, Data, Tensor};
use machine_academy::data::DataGen;
use rand::rngs::SmallRng;
use rand::SeedableRng;
use rand::{
    distributions::{Bernoulli, Distribution},
    Rng,
};
use rand_distr::Normal;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

const SEQ_LENGTH: usize = 150;
const MAX_DELAY: f32 = 0.5;
const RESAMPLE_PROB: f64 = 0.006;
const DELTA: f32 = 3.0 / SEQ_LENGTH as f32;
const MAX_CONTROL_DRIFT: f32 = 1.0;
const MAX_CONTROL_STD_DEV: f32 = 1.0;
const MAX_MEASUREMENT_STD_DEV: f32 = 0.1;
const MAX_SLOWDOWN_STD_DEV: f32 = 0.1;

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct TrainingItem {
    #[serde(with = "BigArray")]
    input: [[f32; 2]; SEQ_LENGTH],
    target: f32,
}

impl<B: Backend> From<TrainingItem> for (Tensor<B, 2>, Tensor<B, 1>) {
    fn from(value: TrainingItem) -> Self {
        (
            Tensor::<B, 2>::from_data(Data::<f32, 2>::from(value.input).convert()),
            Tensor::<B, 1>::from_data(Data::<f32, 1>::from([value.target]).convert()),
        )
    }
}

#[derive(Default)]
pub struct TrainingItemGen {
    rngs: SegQueue<SmallRng>,
}

impl DataGen for TrainingItemGen {
    type Output = TrainingItem;

    fn gen(&self) -> Self::Output {
        let mut rng = self.rngs.pop().unwrap_or_else(SmallRng::from_entropy);
        let mut item = TrainingItem {
            input: [[0.0; 2]; SEQ_LENGTH],
            target: 0.0,
        };

        let mut left_drive;
        let mut left_control_drift;
        let mut left_control_rng;
        let measurement_rng =
            Normal::new(0.0, rng.gen_range(0.0..MAX_MEASUREMENT_STD_DEV)).unwrap();
        let slowdown_rng = Normal::new(0.0, rng.gen_range(0.0..MAX_SLOWDOWN_STD_DEV)).unwrap();

        macro_rules! resample {
            () => {
                left_control_drift = rng.gen_range(0.0..MAX_CONTROL_DRIFT);
                left_control_rng =
                    Normal::new(0.0, rng.gen_range(0.0..MAX_CONTROL_STD_DEV)).unwrap();

                left_drive = rng.gen_range(-1.0..1.0);
            };
        }

        macro_rules! next_drive {
            ($drive: expr, $drift: ident, $rng: ident) => {{
                let val = $drive + $drift * DELTA + $rng.sample(&mut rng) * DELTA;
                if val > 1.0 {
                    1.0
                } else if val < -1.0 {
                    -1.0
                } else {
                    val
                }
            }};
        }

        resample!();

        // Either side can be up to 100% slowed, meaning that it is stationary
        let left_slowdown = rng.gen_range(0.0..1.0);
        // It can take up to MAX_DELAY seconds for a drive strength to be changed
        let left_delay = rng.gen_range(DELTA..MAX_DELAY);
        // A side can accelerate anywhere from 10%/sec to 1000%/sec
        let left_accel = rng.gen_range(0.1..10.0);

        let mut true_left_drive = left_drive;

        let mut left_delay_queue = VecDeque::new();
        for _ in 0..((left_delay / DELTA).round() as usize) {
            left_drive = next_drive!(left_drive, left_control_drift, left_control_rng);
            left_delay_queue.push_back(left_drive);
        }

        let resample_rng = Bernoulli::new(RESAMPLE_PROB).unwrap();

        for i in 0..SEQ_LENGTH {
            if resample_rng.sample(&mut rng) {
                resample!();
            }

            let target_left_drive = left_delay_queue.pop_front().unwrap();

            if (target_left_drive - true_left_drive).abs() < left_accel * DELTA {
                true_left_drive = target_left_drive;
            } else {
                true_left_drive +=
                    (target_left_drive - true_left_drive).signum() * left_accel * DELTA;
            }

            left_drive = next_drive!(left_drive, left_control_drift, left_control_rng);
            left_delay_queue.push_back(left_drive);

            let mut left_measured = true_left_drive
                * (left_slowdown + slowdown_rng.sample(&mut rng))
                + measurement_rng.sample(&mut rng);

            if left_measured > 1.0 {
                left_measured = 1.0;
            } else if left_measured < -1.0 {
                left_measured = -1.0;
            }

            item.input[i] = [left_drive, left_measured];
            item.target = left_slowdown;
        }

        // 10% of not returning RNG so one is reseeded from entropy
        if rng.gen_bool(0.9) {
            self.rngs.push(rng);
        }

        item
    }

    fn skip(&mut self, _: usize) {}
}
