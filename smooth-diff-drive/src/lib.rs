// #![feature(iter_array_chunks)]

pub mod drive;

use std::{
    collections::VecDeque,
    sync::mpsc::channel,
};

use burn::{
    config::Config,
    data::{
        dataloader::{batcher::Batcher, DataLoaderBuilder},
        dataset::SqliteDataset,
    },
    module::Module,
    nn::{
        loss::{MSELoss, Reduction},
        Dropout, DropoutConfig, Linear, LinearConfig, LayerNorm, LayerNormConfig, gru::{GruConfig, Gru},
    },
    optim::AdamConfig,
    record::{CompactRecorder, Recorder},
    tensor::{
        activation::gelu,
        backend::{AutodiffBackend, Backend},
        Data, Tensor,
    },
    train::{
        metric::{LossMetric, store::{Aggregate, Direction, Split}, CpuUse, CpuTemperature}, LearnerBuilder, RegressionOutput, TrainOutput, TrainStep, ValidStep, MetricEarlyStoppingStrategy, StoppingCondition,
    },
};
use bytemuck::cast_slice;
use crossbeam::queue::SegQueue;
use rand::{
    distributions::{Bernoulli, Distribution},
    rngs::SmallRng,
    Rng, SeedableRng,
};
use rand_distr::Normal;
use rusqlite::{Connection, params};
use serde::{Deserialize, Serialize};
// use serde_big_array::BigArray;
use tokio_rayon::rayon::iter::{IntoParallelIterator, ParallelIterator};

#[derive(Module, Debug)]
pub struct Model<B: Backend> {
    // input: Linear<B>,
    gru1: Gru<B>,
    norm1: LayerNorm<B>,
    gru2: Gru<B>,
    norm2: LayerNorm<B>,
    // lstm3: Gru<B>,
    // norm3: LayerNorm<B>,
    linear1: Linear<B>,
    norm4: LayerNorm<B>,
    linear2: Linear<B>,
    norm5: LayerNorm<B>,
    linear3: Linear<B>,
    norm6: LayerNorm<B>,
    dropout: Dropout,
    hidden_size: usize,
}

#[derive(Config, Debug)]
pub struct ModelConfig {
    pub hidden_size: usize,
    #[config(default = "0.1")]
    pub dropout: f64,
}

impl ModelConfig {
    /// Returns the initialized model.
    pub fn init<B: Backend>(&self) -> Model<B> {
        Model {
            gru1: GruConfig::new(2, self.hidden_size, true).init(),
            norm1: LayerNormConfig::new(self.hidden_size).init(),

            gru2: GruConfig::new(self.hidden_size, self.hidden_size, true).init(),
            norm2: LayerNormConfig::new(self.hidden_size).init(),

            // lstm3: GruConfig::new(self.hidden_size, self.hidden_size, false).init(),
            // norm3: LayerNormConfig::new(self.hidden_size).init(),

            linear1: LinearConfig::new(self.hidden_size, self.hidden_size).init(),
            norm4: LayerNormConfig::new(self.hidden_size).init(),

            linear2: LinearConfig::new(self.hidden_size, self.hidden_size).init(),
            norm5: LayerNormConfig::new(self.hidden_size).init(),

            linear3: LinearConfig::new(self.hidden_size, 1).init(),
            norm6: LayerNormConfig::new(1).init(),

            dropout: DropoutConfig::new(self.dropout).init(),
            
            hidden_size: self.hidden_size,
        }
    }

    pub fn init_with<B>(&self, record: ModelRecord<B>) -> Model<B> where B: Backend {
        Model {
            gru1: GruConfig::new(2, self.hidden_size, true).init_with(record.gru1),
            norm1: LayerNormConfig::new(self.hidden_size).init_with(record.norm1),

            gru2: GruConfig::new(self.hidden_size, self.hidden_size, true).init_with(record.gru2),
            norm2: LayerNormConfig::new(self.hidden_size).init_with(record.norm2),

            // lstm3: GruConfig::new(self.hidden_size, self.hidden_size, false).init(),
            // norm3: LayerNormConfig::new(self.hidden_size).init(),

            linear1: LinearConfig::new(self.hidden_size, self.hidden_size).init_with(record.linear1),
            norm4: LayerNormConfig::new(self.hidden_size).init_with(record.norm4),

            linear2: LinearConfig::new(self.hidden_size, self.hidden_size).init_with(record.linear2),
            norm5: LayerNormConfig::new(self.hidden_size).init_with(record.norm5),

            linear3: LinearConfig::new(self.hidden_size, 2).init_with(record.linear3),
            norm6: LayerNormConfig::new(2).init_with(record.norm6),

            dropout: DropoutConfig::new(self.dropout).init(),
            
            hidden_size: self.hidden_size,
        }
    }
}

impl<B: Backend> Model<B> {
    /// # Shapes
    ///   - Input [batch_size, seq_length, 2]
    ///   - Output [batch_size]
    fn forward(&self, input: Tensor<B, 3>) -> Tensor<B, 2> {
        let [batch_size, seq_length, 2] = input.dims() else {
            panic!("Invalid size")
        };

        // cell_states: [batch_size, sequence_length, hidden_size]
        // hidden_states: [batch_size, sequence_length, hidden_size]
        let x = self.gru1.forward(input, None);
        let x = self.norm1.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.gru2.forward(x, None);

        // let x = self.lstm3.forward(x, None);
        // let x = self.norm3.forward(x);
        // let x = gelu(x);
        // let x = self.dropout.forward(x);

        let x = x.slice([0..batch_size, (seq_length - 1)..seq_length, 0..self.hidden_size]);
        let x = x.reshape([batch_size, self.hidden_size]);
        let x = self.norm2.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.linear1.forward(x);
        // let x = self.norm4.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.linear2.forward(x);
        let x = self.norm5.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.linear3.forward(x);
        // let x = self.norm6.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        x
    }
    
    /// # Shapes
    ///   - Input [batch_size, 2]
    ///   - Output [batch_size]
    fn forward_once(&self, input: Tensor<B, 1>, state: Option<(Tensor<B, 3>, Tensor<B, 3>)>) -> (Tensor<B, 2>, (Tensor<B, 3>, Tensor<B, 3>)) {
        let [2] = input.dims() else {
            panic!("Invalid size")
        };

        let (state1, state2) = state.unzip();
        let input = input.reshape([1, 1, 2]);

        // cell_states: [batch_size, sequence_length, hidden_size]
        // hidden_states: [batch_size, sequence_length, hidden_size]
        let x = self.gru1.forward(input, state1);
        let state1 = x.clone();
        let x = self.norm1.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.gru2.forward(x, state2);
        let state2 = x.clone();

        // let x = self.lstm3.forward(x, None);
        // let x = self.norm3.forward(x);
        // let x = gelu(x);
        // let x = self.dropout.forward(x);

        let x = x.reshape([1, self.hidden_size]);
        let x = self.norm2.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.linear1.forward(x);
        // let x = self.norm4.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.linear2.forward(x);
        let x = self.norm5.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        let x = self.linear3.forward(x);
        // let x = self.norm6.forward(x);
        let x = gelu(x);
        let x = self.dropout.forward(x);

        (x, (state1, state2))
    }

    fn forward_regression(
        &self,
        inputs: Tensor<B, 3>,
        targets: Tensor<B, 2>,
    ) -> RegressionOutput<B> {
        let output = self.forward(inputs);
        let loss = MSELoss::new().forward(output.clone(), targets.clone(), Reduction::Auto);

        RegressionOutput::new(loss, output, targets)
    }
}

const SEQ_LENGTH: usize = 150;

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ModelItem {
    input: Vec<u8>,
    target: f32,
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
            .map(|item| Data::<f32, 2>::from(<[[f32; 2]; SEQ_LENGTH]>::try_from(cast_slice::<_, [f32; 2]>(&item.input)).unwrap()))
            .map(|data| Tensor::<B, 2>::from_data(data.convert()))
            .map(|tensor| tensor.reshape([1, SEQ_LENGTH, 2]))
            .collect();

        let targets = items
            .iter()
            .map(|item| Tensor::<B, 2>::from_data(Data::<f32, 2>::from([[item.target]]).convert()))
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
    #[config(default = 500)]
    pub batch_size: usize,
    #[config(default = 16)]
    pub num_workers: usize,
    #[config(default = 1342)]
    pub seed: u64,
    #[config(default = 1.0e-3)]
    pub learning_rate: f64,
}

pub fn train<B: AutodiffBackend>(artifact_dir: &str, config: TrainingConfig, device: B::Device) {
    std::fs::create_dir_all(artifact_dir).ok();
    config
        .save(format!("{artifact_dir}/config.json"))
        .expect("Config should be saved successfully");

    B::seed(config.seed);

    let batcher_train = ModelBatcher::<B>::new(device.clone());
    let batcher_valid = ModelBatcher::<B::InnerBackend>::new(device.clone());

    let dataloader_train = DataLoaderBuilder::new(batcher_train)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(SqliteDataset::from_db_file("data.sqlite", "train").expect("data.sqlite should be readable"));

    let dataloader_test = DataLoaderBuilder::new(batcher_valid)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(SqliteDataset::from_db_file("data.sqlite", "test").expect("data.sqlite should be readable"));

    let learner = LearnerBuilder::new(artifact_dir)
        .metric_train_numeric(LossMetric::new())
        // .metric_train_numeric(LearningRateMetric::new())
        .metric_train_numeric(CpuTemperature::new())
        .metric_train_numeric(CpuUse::new())
        .metric_valid_numeric(LossMetric::new())
        .early_stopping(MetricEarlyStoppingStrategy::new::<LossMetric<B>>(Aggregate::Mean, Direction::Lowest, Split::Valid, StoppingCondition::NoImprovementSince { n_epochs: 2 }))
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

pub fn test<B: Backend>(artifact_dir: &str, device: B::Device) {
    let config = TrainingConfig::load(format!("{artifact_dir}/config.json"))
        .expect("Config should exist for the model");
    let record = CompactRecorder::new()
        .load(format!("{artifact_dir}/model").into())
        .expect("Trained model should exist");

    let model = config.model.init_with::<B>(record).to_device(&device);
    
    let db = Connection::open("data.sqlite").expect("data.sqlite should have been readable");
    let item = db.query_row("SELECT * FROM train ORDER BY RANDOM() LIMIT 1;", (), |row| {
        let input: Vec<u8> = row.get("input").unwrap();
        let target: f32 = row.get("target").unwrap();
        for [a, b] in cast_slice::<_, [f32; 2]>(&input) {
            println!("[{a:.2}] [{b:.2}]");
        }
        println!("-----------");
        println!("{target:.2}");

        Ok(ModelItem {
            input,
            target
        })
    }).expect("SELECT should have succeeded");

    // let batcher = ModelBatcher::new(device);
    // let batch = batcher.batch(vec![item]);
    // let output = model.forward(batch.inputs);
    // let predicted = output.flatten::<1>(0, 1).into_scalar();
    let input = cast_slice::<_, [f32; 2]>(&item.input);

    let (_, mut state) = model.forward_once(Tensor::from_data(Data::<f32, 1>::from(input[0]).convert()), None);
    let mut output = None;

    for input in input.split_at(1).1 {
        let (new_output, new_state) = model.forward_once(Tensor::from_data(Data::<f32, 1>::from(*input).convert()), Some(state));
        state = new_state;
        output = Some(new_output.into_data().value[0]);
    }
    
    println!("{:.2}", output.unwrap());
}

const MAX_DELAY: f32 = 0.5;
const RESAMPLE_PROB: f64 = 0.006;
const DELTA: f32 = 3.0 / SEQ_LENGTH as f32;
const MAX_CONTROL_DRIFT: f32 = 1.0;
const MAX_CONTROL_STD_DEV: f32 = 1.0;
const MAX_MEASUREMENT_STD_DEV: f32 = 0.1;
const MAX_SLOWDOWN_STD_DEV: f32 = 0.1;


pub fn create_dataset(len: usize, table: &'static str) {
    let rands = SegQueue::new();
    let (sender, receiver) = channel::<ModelItem>();
    let mut db = Connection::open("data.sqlite").expect("data.sqlite should have been writable");
    let res = db.execute(&format!("DROP TABLE {table};"), params![]);
    if let Err(e) = res {
        eprintln!("Failed to delete: {table}: {e}");
    }
    db.execute(&format!("CREATE TABLE {table} (row_id INTEGER, input BLOB, target REAL);"), params![]).expect("Table should have been created");
    
    let handle = std::thread::spawn(move || {
        let mut i = 0usize;
        loop {
            let mut tx = db.transaction().expect("Transaction should have been created");
            tx.set_drop_behavior(rusqlite::DropBehavior::Commit);
            for _ in 0..1000 {
                let Ok(item) = receiver.recv() else { return; };
                i += 1;
                tx.execute(&format!("INSERT INTO {table} (row_id, input, target) VALUES (?1, ?2, ?3);"), params![i, item.input, item.target]).expect("Item should have been inserted");
            }
        }
    });

    (0..len)
        .into_par_iter()
        .for_each(move |_| {
            let mut rng = rands.pop().unwrap_or_else(|| SmallRng::from_entropy());

            let mut item = ModelItem {
                input: vec![],
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

            for _ in 0..SEQ_LENGTH {
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

                let mut left_measured = true_left_drive * (left_slowdown + slowdown_rng.sample(&mut rng))
                + measurement_rng.sample(&mut rng);

                if left_measured > 1.0 {
                    left_measured = 1.0;
                } else if left_measured < -1.0 {
                    left_measured = -1.0;
                }

                item.input.extend_from_slice(&left_drive.to_ne_bytes());
                item.input.extend_from_slice(&left_measured.to_ne_bytes());
                item.target = left_slowdown;
            }

            // 10% chance of resampling the rng
            if rng.gen_bool(0.1) {
                rng = SmallRng::from_entropy();
            }

            rands.push(rng);

            sender.send(item).unwrap();
        });
    
    handle.join().unwrap();
}
