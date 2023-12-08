#![feature(array_chunks, iterator_try_collect)]

pub mod drive;

use std::{
    collections::VecDeque,
    fs::File,
    io::{BufReader, BufWriter},
    path::Path,
};

use burn::{
    config::Config,
    data::{
        dataloader::{batcher::Batcher, DataLoaderBuilder},
        dataset::InMemDataset,
    },
    module::Module,
    nn::{
        loss::{MSELoss, Reduction},
        Dropout, DropoutConfig, Linear, LinearConfig, Lstm, LstmConfig,
    },
    optim::AdamConfig,
    record::CompactRecorder,
    tensor::{
        activation::sigmoid,
        backend::{AutodiffBackend, Backend},
        Data, Tensor,
    },
    train::{
        metric::LossMetric, LearnerBuilder, RegressionOutput, TrainOutput, TrainStep, ValidStep,
    },
};
use crossbeam::queue::SegQueue;
use rand::{
    distributions::{Bernoulli, Distribution},
    rngs::SmallRng,
    Rng, SeedableRng,
};
use rand_distr::Normal;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;
use tokio_rayon::rayon::iter::{IntoParallelIterator, ParallelIterator};

#[derive(Module, Debug)]
pub struct Model<B: Backend> {
    // input: Linear<B>,
    lstm1: Lstm<B>,
    lstm2: Lstm<B>,
    lstm3: Lstm<B>,
    linear1: Linear<B>,
    linear2: Linear<B>,
    dropout: Dropout,
    hidden_size: usize,
}

#[derive(Config, Debug)]
pub struct ModelConfig {
    // num_classes: usize,
    hidden_size: usize,
    #[config(default = "0.3")]
    dropout: f64,
}

impl ModelConfig {
    /// Returns the initialized model.
    pub fn init<B: Backend>(&self) -> Model<B> {
        Model {
            lstm1: LstmConfig::new(4, self.hidden_size, true).init(),
            lstm2: LstmConfig::new(self.hidden_size, self.hidden_size, true).init(),
            lstm3: LstmConfig::new(self.hidden_size, self.hidden_size, true).init(),
            linear1: LinearConfig::new(self.hidden_size, self.hidden_size).init(),
            linear2: LinearConfig::new(self.hidden_size, 2).init(),
            dropout: DropoutConfig::new(self.dropout).init(),
            hidden_size: self.hidden_size,
        }
    }
}

impl<B: Backend> Model<B> {
    /// # Shapes
    ///   - Input [batch_size, seq_length, 2]
    ///   - Output [batch_size]
    pub fn forward(&self, input: Tensor<B, 3>) -> Tensor<B, 2> {
        let [batch_size, seq_length, 4] = input.dims() else {
            panic!("Invalid size")
        };

        // cell_states: [batch_size, sequence_length, hidden_size]
        // hidden_states: [batch_size, sequence_length, hidden_size]
        let (_cell_states, hidden_states) = self.lstm1.forward(input, None);
        let x = self.dropout.forward(hidden_states);

        let (_cell_states, hidden_states) = self.lstm2.forward(x, None);
        let x = self.dropout.forward(hidden_states);

        let (_cell_states, mut hidden_states) = self.lstm3.forward(x, None);

        hidden_states = hidden_states.slice([
            0..batch_size,
            (seq_length - 1)..seq_length,
            0..self.hidden_size,
        ]);
        let x = hidden_states.reshape([batch_size, self.hidden_size]);
        let x = self.dropout.forward(x);

        let x = self.linear1.forward(x);
        let x = self.dropout.forward(x);
        let x = self.linear2.forward(x);
        let x = x.reshape([batch_size, 2]);
        let x = self.dropout.forward(x);

        sigmoid(x)
    }

    pub fn forward_regression(
        &self,
        inputs: Tensor<B, 3>,
        targets: Tensor<B, 2>,
    ) -> RegressionOutput<B> {
        let output = self.forward(inputs);
        let loss = MSELoss::new().forward(output.clone(), targets.clone(), Reduction::Auto);

        // let dims = output.dims();
        // let output = output.reshape([dims[0], 1]);
        // let dims = targets.dims();
        // let targets = targets.reshape([dims[0], 1]);
        RegressionOutput::new(loss, output, targets)
    }
}

const SEQ_LENGTH: usize = 150;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct ModelItem {
    #[serde(with = "BigArray")]
    input: [[f32; 4]; SEQ_LENGTH],
    target: [f32; 2],
}

#[derive(Clone, Debug)]
struct ModelBatch<B: Backend> {
    inputs: Tensor<B, 3>,
    targets: Tensor<B, 2>,
}

impl<B: AutodiffBackend> TrainStep<ModelBatch<B>, RegressionOutput<B>> for Model<B> {
    fn step(&self, batch: ModelBatch<B>) -> TrainOutput<RegressionOutput<B>> {
        let item = self.forward_regression(batch.inputs, batch.targets);

        TrainOutput::new(self, item.loss.backward(), item)
    }
}

impl<B: Backend> ValidStep<ModelBatch<B>, RegressionOutput<B>> for Model<B> {
    fn step(&self, batch: ModelBatch<B>) -> RegressionOutput<B> {
        self.forward_regression(batch.inputs, batch.targets)
    }
}

pub struct ModelBatcher<B: Backend> {
    device: B::Device,
}

impl<B: Backend> ModelBatcher<B> {
    pub fn new(device: B::Device) -> Self {
        Self { device }
    }
}

impl<B: Backend> Batcher<ModelItem, ModelBatch<B>> for ModelBatcher<B> {
    fn batch(&self, items: Vec<ModelItem>) -> ModelBatch<B> {
        let inputs = items
            .iter()
            .map(|item| Data::<f32, 2>::from(item.input))
            .map(|data| Tensor::<B, 2>::from_data(data.convert()))
            .map(|tensor| tensor.reshape([1, SEQ_LENGTH, 4]))
            .collect();

        let targets = items
            .iter()
            .map(|item| Tensor::<B, 2>::from_data(Data::<f32, 2>::from([item.target]).convert()))
            .collect();

        let inputs = Tensor::cat(inputs, 0).to_device(&self.device);
        let targets = Tensor::cat(targets, 0).to_device(&self.device);

        ModelBatch { inputs, targets }
    }
}

#[derive(Config)]
pub struct TrainingConfig {
    pub model: ModelConfig,
    pub optimizer: AdamConfig,
    #[config(default = 10)]
    pub num_epochs: usize,
    #[config(default = 64)]
    pub batch_size: usize,
    #[config(default = 4)]
    pub num_workers: usize,
    #[config(default = 42)]
    pub seed: u64,
    #[config(default = 1.0e-4)]
    pub learning_rate: f64,
}

pub fn train<B: AutodiffBackend>(artifact_dir: &str, config: TrainingConfig, device: B::Device) {
    std::fs::create_dir_all(artifact_dir).ok();
    config
        .save(format!("{artifact_dir}/config.json"))
        .expect("Config should be saved successfully");

    B::seed(config.seed);

    let train_data = serde_json::from_reader(BufReader::new(
        File::open("train.json").expect("train.json should have opened successfully"),
    ))
    .expect("train.json should be valid");
    let test_data = serde_json::from_reader(BufReader::new(
        File::open("test.json").expect("test.json should have opened successfully"),
    ))
    .expect("test.json should be valid");

    let batcher_train = ModelBatcher::<B>::new(device.clone());
    let batcher_valid = ModelBatcher::<B::InnerBackend>::new(device.clone());

    let dataloader_train = DataLoaderBuilder::new(batcher_train)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(InMemDataset::new(train_data));

    let dataloader_test = DataLoaderBuilder::new(batcher_valid)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(InMemDataset::new(test_data));

    let learner = LearnerBuilder::new(artifact_dir)
        .metric_train_numeric(LossMetric::new())
        .metric_valid_numeric(LossMetric::new())
        .with_file_checkpointer(CompactRecorder::new())
        .devices(vec![device])
        .num_epochs(config.num_epochs)
        .build(
            config.model.init::<B>(),
            config.optimizer.init(),
            config.learning_rate,
        );

    let model_trained = learner.fit(dataloader_train, dataloader_test);

    model_trained
        .save_file(format!("{artifact_dir}/model"), &CompactRecorder::new())
        .expect("Trained model should be saved successfully");
}

const MAX_DELAY: f32 = 1.0;
const RESAMPLE_PROB: f64 = 0.01;
const DELTA: f32 = 0.02;
const MAX_CONTROL_DRIFT: f32 = 1.0;
const MAX_CONTROL_STD_DEV: f32 = 1.0;
const MAX_MEASUREMENT_STD_DEV: f32 = 0.25;
const MAX_SLOWDOWN_STD_DEV: f32 = 0.5;

pub fn create_dataset(len: usize, file: impl AsRef<Path> + Send + 'static) {
    let rands = SegQueue::new();
    let file = File::create(file).expect("File should have been writable");
    let file = BufWriter::new(file);

    let items: Vec<_> = (0..len)
        .into_par_iter()
        .map(|_| {
            let mut rng = rands.pop().unwrap_or_else(|| SmallRng::from_entropy());

            let mut item = ModelItem {
                input: [[0.0; 4]; SEQ_LENGTH],
                target: [0.0; 2],
            };

            let mut left_drive;
            let mut right_drive;
            let mut left_control_drift;
            let mut right_control_drift;
            let mut left_control_rng;
            let mut right_control_rng;
            let measurement_rng =
                Normal::new(0.0, rng.gen_range(0.0..MAX_MEASUREMENT_STD_DEV)).unwrap();
            let slowdown_rng = Normal::new(0.0, rng.gen_range(0.0..MAX_SLOWDOWN_STD_DEV)).unwrap();

            macro_rules! resample {
                () => {
                    left_control_drift = rng.gen_range(0.0..MAX_CONTROL_DRIFT);
                    right_control_drift = rng.gen_range(0.0..MAX_CONTROL_DRIFT);
                    left_control_rng =
                        Normal::new(0.0, rng.gen_range(0.0..MAX_CONTROL_STD_DEV)).unwrap();
                    right_control_rng =
                        Normal::new(0.0, rng.gen_range(0.0..MAX_CONTROL_STD_DEV)).unwrap();

                    if rng.gen() {
                        if rng.gen() {
                            left_drive = 1.0;
                        } else {
                            left_drive = -1.0;
                        }
                        right_drive = rng.gen_range(-1.0..1.0);
                    } else {
                        left_drive = rng.gen_range(-1.0..1.0);
                        if rng.gen() {
                            right_drive = 1.0;
                        } else {
                            right_drive = -1.0;
                        }
                    }
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
            let right_slowdown = rng.gen_range(0.0..1.0);
            // It can take up to 1 second for a drive strength to be changed
            let left_delay = rng.gen_range(DELTA..MAX_DELAY);
            let right_delay = rng.gen_range(DELTA..MAX_DELAY);
            // A side can accelerate anywhere from 10%/sec to 1000%/sec
            let left_accel = rng.gen_range(0.1..10.0);
            let right_accel = rng.gen_range(0.1..10.0);

            let mut true_left_drive = left_drive;
            let mut true_right_drive = right_drive;

            let mut left_delay_queue = VecDeque::new();
            for _ in 0..((left_delay / DELTA).round() as usize) {
                left_drive = next_drive!(left_drive, left_control_drift, left_control_rng);
                left_delay_queue.push_back(left_drive);
            }

            let mut right_delay_queue = VecDeque::new();
            for _ in 0..((right_delay / DELTA).round() as usize) {
                right_drive = next_drive!(right_drive, right_control_drift, right_control_rng);
                right_delay_queue.push_back(right_drive);
            }

            let resample_rng = Bernoulli::new(RESAMPLE_PROB).unwrap();

            for i in 0..SEQ_LENGTH {
                if resample_rng.sample(&mut rng) {
                    resample!();
                }

                let target_left_drive = left_delay_queue.pop_front().unwrap();
                let target_right_drive = right_delay_queue.pop_front().unwrap();

                if (target_left_drive - true_left_drive).abs() < left_accel * DELTA {
                    true_left_drive = target_left_drive;
                } else {
                    true_left_drive +=
                        (target_left_drive - true_left_drive).signum() * left_accel * DELTA;
                }

                if (target_right_drive - true_right_drive).abs() < left_accel * DELTA {
                    true_right_drive = target_right_drive;
                } else {
                    true_right_drive +=
                        (target_right_drive - true_right_drive).signum() * right_accel * DELTA;
                }

                left_drive = next_drive!(left_drive, left_control_drift, left_control_rng);
                left_delay_queue.push_back(left_drive);

                right_drive = next_drive!(right_drive, right_control_drift, right_control_rng);
                right_delay_queue.push_back(right_drive);

                let mut left_measured = true_left_drive * (left_slowdown + slowdown_rng.sample(&mut rng))
                + measurement_rng.sample(&mut rng);
                let mut right_measured = true_right_drive * (right_slowdown + slowdown_rng.sample(&mut rng))
                + measurement_rng.sample(&mut rng);

                if left_measured > 1.0 {
                    left_measured = 1.0;
                } else if left_measured < -1.0 {
                    left_measured = -1.0;
                }

                if right_measured > 1.0 {
                    right_measured = 1.0;
                } else if right_measured < -1.0 {
                    right_measured = -1.0;
                }

                item.input[i] = [
                    left_drive,
                    right_drive,
                    left_measured,
                    right_measured,
                ];
                item.target = [left_slowdown, right_slowdown];
            }

            // 10% chance of resampling the rng
            if rng.gen_bool(0.1) {
                rng = SmallRng::from_entropy();
            }

            rands.push(rng);
            item
        })
        .collect();

    serde_json::to_writer(file, &items).expect("serialization should have succeeded");
}
