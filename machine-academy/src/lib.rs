use std::{
    fmt::{Debug, Display},
    marker::PhantomData,
    path::{Path, PathBuf},
    sync::{Arc, Mutex}, fs::File,
    io::Write, time::Instant
};

use burn::{
    config::Config,
    data::dataloader::{batcher::Batcher, DataLoaderBuilder},
    lr_scheduler::noam::NoamLrSchedulerConfig,
    module::{AutodiffModule, Module},
    optim::AdamConfig,
    record::CompactRecorder,
    tensor::{
        backend::{AutodiffBackend, Backend},
        ElementConversion, Tensor,
    },
    train::{
        metric::{
            state::{FormatOptions, NumericMetricState},
            store::{Aggregate, Direction, Split},
            Adaptor, CpuTemperature, CpuUse, LearningRateMetric, LossMetric, Metric, MetricEntry,
            MetricMetadata, Numeric,
        },
        ClassificationOutput, LearnerBuilder, MetricEarlyStoppingStrategy, RegressionOutput,
        StoppingCondition, TrainOutput, TrainStep, ValidStep,
    },
};
use chrono::{Datelike, Timelike};
use data::AcademyDataset;
use rand::{rngs::SmallRng, RngCore, SeedableRng};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

pub use burn;

pub mod common;
pub mod data;

pub trait Model<B: Backend>: Module<B> + Display + Debug + 'static {
    type Input;
    type Output;
    type Config: Config;

    fn from_config(config: Self::Config) -> Self;
    fn forward(&self, input: Self::Input) -> Self::Output;
}

pub trait TrainableModel<B: Backend, O>: Model<B> {
    type Batch: Send;

    fn forward_training(&self, batch: Self::Batch) -> O;
}

#[derive(Clone)]
pub struct TrainingModel<M, B>(pub M, PhantomData<B>);

impl<B: AutodiffBackend, M: TrainableModel<B, RegressionOutput<B>> + AutodiffModule<B>>
    TrainStep<M::Batch, RegressionOutput<B>> for TrainingModel<M, B>
{
    fn step(&self, batch: M::Batch) -> TrainOutput<RegressionOutput<B>> {
        let item = M::forward_training(&self.0, batch);
        TrainOutput::new(&self.0, item.loss.backward(), item)
    }
}

// impl<B: AutodiffBackend, M: Model<LossOutput<B> = ClassificationOutput<B>, ModelStruct<B> = M> + AutodiffModule<B>>
//     TrainStep<M::Batch<B>, ClassificationOutput<B>> for TrainingModel<M, B>
// {
//     fn step(&self, batch: M::Batch<B>) -> TrainOutput<ClassificationOutput<B>> {
//         let item = M::forward_training(&self.0, batch);
//         TrainOutput::new(&self.0, item.loss.backward(), item)
//     }
// }

impl<B: Backend, M: TrainableModel<B, RegressionOutput<B>>>
    ValidStep<M::Batch, RegressionOutput<B>> for TrainingModel<M, B>
{
    fn step(&self, batch: M::Batch) -> RegressionOutput<B> {
        M::forward_training(&self.0, batch)
    }
}

// impl<B: Backend, M: Model<LossOutput<B> = ClassificationOutput<B>, ModelStruct<B> = M>>
//     ValidStep<M::Batch<B>, ClassificationOutput<B>> for TrainingModel<M, B>
// {
//     fn step(&self, batch: M::Batch<B>) -> ClassificationOutput<B> {
//         M::forward_training(&self.0, batch)
//     }
// }

impl<B: Backend, M: Module<B>> Module<B> for TrainingModel<M, B> {
    type Record = M::Record;

    fn collect_devices(&self, devices: burn::module::Devices<B>) -> burn::module::Devices<B> {
        self.0.collect_devices(devices)
    }

    fn fork(mut self, device: &<B as Backend>::Device) -> Self {
        self.0 = self.0.fork(device);
        self
    }

    fn to_device(mut self, device: &<B as Backend>::Device) -> Self {
        self.0 = self.0.to_device(device);
        self
    }

    fn visit<V: burn::module::ModuleVisitor<B>>(&self, visitor: &mut V) {
        self.0.visit(visitor);
    }

    fn map<MM: burn::module::ModuleMapper<B>>(mut self, mapper: &mut MM) -> Self {
        self.0 = self.0.map(mapper);
        self
    }

    fn load_record(mut self, record: Self::Record) -> Self {
        self.0 = self.0.load_record(record);
        self
    }

    fn into_record(self) -> Self::Record {
        self.0.into_record()
    }
}

impl<B: AutodiffBackend, M: AutodiffModule<B>> AutodiffModule<B> for TrainingModel<M, B> {
    type InnerModule = TrainingModel<M::InnerModule, B::InnerBackend>;

    fn valid(&self) -> Self::InnerModule {
        TrainingModel::new(self.0.valid())
    }
}

impl<B, M: Debug> Debug for TrainingModel<M, B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(&self.0, f)
    }
}

impl<B, M: Display> Display for TrainingModel<M, B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Display::fmt(&self.0, f)
    }
}

impl<M, B> TrainingModel<M, B> {
    pub fn new(model: M) -> Self {
        Self(model, PhantomData)
    }
}

#[derive(Clone, Debug)]
pub struct RegressionBatch<B: Backend, const I: usize, const T: usize> {
    pub inputs: Tensor<B, I>,
    pub targets: Tensor<B, T>,
}

pub struct RegressionBatcher<B: Backend> {
    device: B::Device,
}

impl<B: Backend> RegressionBatcher<B> {
    pub fn new(device: B::Device) -> Self {
        Self { device }
    }
}

impl<B: Backend, Item: Into<(Tensor<B, 2>, Tensor<B, 1>)>>
    Batcher<Item, RegressionBatch<B, 3, 2>> for RegressionBatcher<B>
{
    fn batch(&self, items: Vec<Item>) -> RegressionBatch<B, 3, 2> {
        let (inputs, targets) = items
            .into_iter()
            .map(|item| Into::<(Tensor<B, 2>, Tensor<B, 1>)>::into(item))
            .map(|(input, target)| {
                let [a, b] = input.dims();
                let len = a * b / 2;
                let target_len = target.dims()[0];
                (input.reshape([1, len, 2]), target.reshape([1, target_len]))
            })
            .unzip();

        let inputs = Tensor::cat(inputs, 0).to_device(&self.device);
        let targets = Tensor::cat(targets, 0).to_device(&self.device);

        RegressionBatch { inputs, targets }
    }
}

#[derive(Serialize, Deserialize)]
pub struct TrainingConfig<T> {
    pub model_config: T,
    pub optimizer: AdamConfig,
    #[serde(default = "default_num_epochs")]
    pub num_epochs: usize,
    #[serde(default = "default_batch_size")]
    pub batch_size: usize,
    #[serde(default = "default_num_workers")]
    pub num_workers: usize,
    #[serde(default = "default_seed")]
    pub seed: u64,
    #[serde(default = "default_init_learning_rate")]
    pub init_learning_rate: f64,
    #[serde(default = "default_stop_condition_epochs")]
    pub stop_condition_epochs: usize,
}

fn default_num_epochs() -> usize {
    5
}

fn default_batch_size() -> usize {
    128
}

fn default_num_workers() -> usize {
    4
}

fn default_seed() -> u64 {
    8000
}

fn default_init_learning_rate() -> f64 {
    0.001
}

fn default_stop_condition_epochs() -> usize {
    2
}

impl<T: Serialize + DeserializeOwned> Config for TrainingConfig<T> {}

impl<T> TrainingConfig<T> {
    pub fn new(model_config: T) -> Self {
        Self {
            model_config,
            optimizer: AdamConfig::new(),
            num_epochs: default_num_epochs(),
            batch_size: default_batch_size(),
            num_workers: default_num_workers(),
            seed: default_seed(),
            init_learning_rate: default_init_learning_rate(),
            stop_condition_epochs: default_stop_condition_epochs(),
        }
    }
}

/// The loss metric.
pub struct TrackedLossMetric<B: Backend> {
    state: Arc<Mutex<NumericMetricState>>,
    _b: B,
}

/// The [loss metric](LossMetric) input type.
pub struct LossInput<B: Backend> {
    tensor: Tensor<B, 1>,
}
impl<B: Backend> Adaptor<LossInput<B>> for ClassificationOutput<B> {
    fn adapt(&self) -> LossInput<B> {
        LossInput {
            tensor: self.loss.clone(),
        }
    }
}
impl<B: Backend> Adaptor<LossInput<B>> for RegressionOutput<B> {
    fn adapt(&self) -> LossInput<B> {
        LossInput {
            tensor: self.loss.clone(),
        }
    }
}

impl<B: Backend> Metric for TrackedLossMetric<B> {
    const NAME: &'static str = "Loss";

    type Input = LossInput<B>;

    fn update(&mut self, loss: &Self::Input, _metadata: &MetricMetadata) -> MetricEntry {
        let loss = f64::from_elem(loss.tensor.clone().mean().into_data().value[0]);

        self.state
            .lock()
            .unwrap()
            .update(loss, 1, FormatOptions::new(Self::NAME).precision(2))
    }

    fn clear(&mut self) {
        self.state.lock().unwrap().reset()
    }
}

impl<B: Backend> Numeric for TrackedLossMetric<B> {
    fn value(&self) -> f64 {
        self.state.lock().unwrap().value()
    }
}

#[derive(Config)]
pub struct Statistics {
    loss: f64
}

pub fn train_regression<B, T, I>(
    artifact_dir: &str,
    training_data_path: PathBuf,
    testing_data_path: PathBuf,
    max_memory_usage: usize,
    config: TrainingConfig<T::Config>,
    device: B::Device,
) -> Statistics
where
    B: AutodiffBackend,
    T: Model<B> + AutodiffModule<B>,
    TrainingModel<T, B>:
        TrainStep<RegressionBatch<B, 3, 2>, RegressionOutput<B>>,
    <TrainingModel<T, B> as AutodiffModule<B>>::InnerModule:
        ValidStep<RegressionBatch<B::InnerBackend, 3, 2>, RegressionOutput<B::InnerBackend>>,
    I: Send
        + Sync
        + Clone
        + Debug
        + Into<(Tensor<B, 2>, Tensor<B, 1>)>
        + Into<(Tensor<B::InnerBackend, 2>, Tensor<B::InnerBackend, 1>)>
        + DeserializeOwned
        + 'static,
{
    std::fs::create_dir_all(artifact_dir).expect("artifact dir should be creatable");
    config
        .save(Path::new(artifact_dir).join("config.json"))
        .expect("Config should be saved successfully");

    B::seed(config.seed);

    let batcher_train = RegressionBatcher::<B>::new(device.clone());
    let batcher_valid = RegressionBatcher::<B::InnerBackend>::new(device.clone());

    let dataloader_train = DataLoaderBuilder::<I, _>::new(batcher_train)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(AcademyDataset::new(training_data_path, max_memory_usage));

    let dataloader_test = DataLoaderBuilder::<I, _>::new(batcher_valid)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(AcademyDataset::new(testing_data_path, max_memory_usage));

    let model = T::from_config(config.model_config);
    let num_params = model.num_params();

    let loss_state: Arc<Mutex<NumericMetricState>> = Default::default();
    let learner = LearnerBuilder::new(artifact_dir)
        .metric_valid_numeric(TrackedLossMetric {
            state: loss_state.clone(),
            _b: Default::default(),
        })
        .metric_train_numeric(LossMetric::new())
        .metric_train_numeric(LearningRateMetric::new())
        .metric_train_numeric(CpuTemperature::new())
        .metric_train_numeric(CpuUse::new())
        .early_stopping(MetricEarlyStoppingStrategy::new::<LossMetric<B>>(
            Aggregate::Mean,
            Direction::Lowest,
            Split::Valid,
            StoppingCondition::NoImprovementSince {
                n_epochs: config.stop_condition_epochs,
            },
        ))
        .with_file_checkpointer(CompactRecorder::new())
        .devices(vec![device])
        .num_epochs(config.num_epochs)
        .build(
            TrainingModel::new(model),
            config.optimizer.init(),
            NoamLrSchedulerConfig::new(config.init_learning_rate)
                .with_model_size(num_params)
                .init(),
        );

    let model_trained = learner.fit(dataloader_train, dataloader_test);

    model_trained
        .save_file(
            Path::new(artifact_dir).join("model"),
            &CompactRecorder::new(),
        )
        .expect("Trained model should be saved successfully");

    let loss = loss_state.lock().unwrap().value();
    let stats = Statistics {
        loss
    };
    stats.save(Path::new(artifact_dir).join("statistics.json")).expect("Statistics file should be creatable");
    stats
}

#[derive(Serialize, Deserialize)]
pub struct SuperTrainingConfig<T> {
    pub model_config: T,
    pub optimizer: AdamConfig,
    #[serde(default = "default_num_epochs")]
    pub num_epochs: usize,
    #[serde(default = "default_min_batch_pow")]
    pub min_batch_pow: u32,
    #[serde(default = "default_max_batch_pow")]
    pub max_batch_pow: u32,
    #[serde(default = "default_num_workers")]
    pub num_workers: usize,
    #[serde(default = "default_seed")]
    pub seed: u64,
    #[serde(default = "default_seed_count")]
    pub seed_count: usize,
    #[serde(default = "default_init_learning_rate_min_pow")]
    pub init_learning_rate_min_pow: i32,
    #[serde(default = "default_init_learning_rate_max_pow")]
    pub init_learning_rate_max_pow: i32,
    #[serde(default = "default_stop_condition_epochs")]
    pub stop_condition_epochs: usize,
    #[serde(default = "default_min_grad_clipping_step")]
    pub min_grad_clipping_step: usize,
    #[serde(default = "default_max_grad_clipping_step")]
    pub max_grad_clipping_step: usize,
    #[serde(default = "default_grad_clipping_step_size")]
    pub grad_clipping_step_size: f32,
}

fn default_min_batch_pow() -> u32 {
    5
}

fn default_max_batch_pow() -> u32 {
    9
}

fn default_seed_count() -> usize {
    2
}

fn default_init_learning_rate_min_pow() -> i32 {
    -4
}

fn default_init_learning_rate_max_pow() -> i32 {
    -2
}

fn default_min_grad_clipping_step() -> usize {
    1
}

fn default_max_grad_clipping_step() -> usize {
    5
}

fn default_grad_clipping_step_size() -> f32 {
    0.2
}

impl<T: Serialize + DeserializeOwned> Config for SuperTrainingConfig<T> {}

pub fn super_train_regression<B, T, I, TC, TCI>(
    mut super_dir: String,
    max_memory_usage: usize,
    config: SuperTrainingConfig<TC>,
    training_data_path: PathBuf,
    testing_data_path: PathBuf,
    device: B::Device,
) where
    B: AutodiffBackend,
    T: Model<B> + AutodiffModule<B>,
    TrainingModel<T, B>:
        TrainStep<RegressionBatch<B, 3, 2>, RegressionOutput<B>>,
    <TrainingModel<T, B> as AutodiffModule<B>>::InnerModule:
        ValidStep<RegressionBatch<B::InnerBackend, 3, 2>, RegressionOutput<B::InnerBackend>>,
    I: Send
        + Sync
        + Clone
        + Debug
        + Into<(Tensor<B, 2>, Tensor<B, 1>)>
        + Into<(Tensor<B::InnerBackend, 2>, Tensor<B::InnerBackend, 1>)>
        + DeserializeOwned
        + 'static,
    TC: IntoIterator<IntoIter = TCI>,
    TCI: ExactSizeIterator + Iterator<Item = T::Config>,
    T::Config: Clone,
{
    let datetime = chrono::Local::now();
    let start = Instant::now();
    let log_folder_name = format!(
        "{}-{:0>2}-{:0>2}={:0>2}-{:0>2}",
        datetime.year(),
        datetime.month(),
        datetime.day(),
        datetime.hour(),
        datetime.minute()
    );

    super_dir += "/";
    super_dir += &log_folder_name;
    std::fs::create_dir_all(&super_dir).expect("super dir should be creatable");
    let mut log_file = File::create(Path::new(&super_dir).join("super.log")).expect("log file should be creatable");

    let model_config = config.model_config.into_iter();
    let max_i = model_config.len()
        * (config.max_batch_pow - config.min_batch_pow + 1) as usize
        * config.seed_count
        * (config.init_learning_rate_max_pow - config.init_learning_rate_min_pow + 1) as usize
        * (config.max_grad_clipping_step - config.min_grad_clipping_step + 1);
    let mut i = 0usize;

    model_config.for_each(|model_config| {
        (config.min_batch_pow..=config.max_batch_pow)
            .into_iter()
            .for_each(|batch_pow| {
                let mut rng = SmallRng::seed_from_u64(config.seed);
                (0..config.seed_count).into_iter().for_each(|_| {
                    let seed = rng.next_u64();
                    (config.init_learning_rate_min_pow..=config.init_learning_rate_max_pow)
                        .into_iter()
                        .for_each(|init_learning_rate_pow| {
                            (config.min_grad_clipping_step..=config.max_grad_clipping_step)
                                .into_iter()
                                .for_each(|grad_clipping_step| {
                                    let config = TrainingConfig {
                                        model_config: model_config.clone(),
                                        optimizer: config.optimizer.clone().with_grad_clipping(
                                            Some(
                                                burn::grad_clipping::GradientClippingConfig::Value(
                                                    grad_clipping_step as f32
                                                        * config.grad_clipping_step_size,
                                                ),
                                            ),
                                        ),
                                        num_epochs: config.num_epochs,
                                        batch_size: 2usize.pow(batch_pow),
                                        num_workers: config.num_workers,
                                        seed,
                                        init_learning_rate: 10.0f64.powi(init_learning_rate_pow),
                                        stop_condition_epochs: config.stop_condition_epochs,
                                    };
                                    let mut elapsed = start.elapsed().as_secs();
                                    let mut hours = elapsed / 3600;
                                    let mut mins = elapsed % 3600 / 60;
                                    let mut secs = elapsed % 3600 % 60;
                                    i += 1;
                                    let progress = i as f32 / max_i as f32 * 100.0;
                                    writeln!(log_file, "[{hours}:{mins}:{secs}] Running iter {i} of {max_i}. {progress:.2}%").expect("log file should be writable");
                                    let stats = train_regression::<B, T, I>(
                                        &format!("{super_dir}/iter_{i}"),
                                        training_data_path.clone(),
                                        testing_data_path.clone(),
                                        max_memory_usage,
                                        config,
                                        device.clone(),
                                    );
                                    elapsed = start.elapsed().as_secs();
                                    hours = elapsed / 3600;
                                    mins = elapsed % 3600 / 60;
                                    secs = elapsed % 3600 % 60;
                                    writeln!(log_file, "[{hours}:{mins}:{secs}] Loss: {:.5}", stats.loss).expect("log file should be writable");
                                });
                        });
                });
            });
    });
}
