use std::{
    fmt::{Debug, Display},
    marker::PhantomData,
    sync::{Arc, Mutex}, path::PathBuf,
};

use burn::{
    data::
        dataloader::{batcher::Batcher, DataLoaderBuilder}
    ,
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
    }, config::Config,
};
use data::AcademyDataset;
use serde::{de::DeserializeOwned, Deserialize, Serialize};

pub use burn;

pub mod common;
pub mod data;

pub trait Model<B: AutodiffBackend>: AutodiffModule<B> + Display + 'static {
    type Input;
    type Output;
    type LossOutput;
    type Batch: Send;
    type Config: Config;

    fn from_config(config: Self::Config) -> Self;
    fn forward(&self, input: Self::Input) -> Self::Output;
    fn forward_training(&self, batch: Self::Batch) -> Self::LossOutput;
}

#[derive(Clone)]
pub struct TrainingModel<M: Model<B>, B: AutodiffBackend>(pub M, PhantomData<B>);

impl<B: AutodiffBackend, M: Model<B, LossOutput = RegressionOutput<B>>>
    TrainStep<M::Batch, RegressionOutput<B>> for TrainingModel<M, B>
{
    fn step(&self, batch: M::Batch) -> TrainOutput<RegressionOutput<B>> {
        let item = self.0.forward_training(batch);
        TrainOutput::new(&self.0, item.loss.backward(), item)
    }
}

impl<B: AutodiffBackend, M: Model<B, LossOutput = ClassificationOutput<B>>>
    TrainStep<M::Batch, ClassificationOutput<B>> for TrainingModel<M, B>
{
    fn step(&self, batch: M::Batch) -> TrainOutput<ClassificationOutput<B>> {
        let item = self.0.forward_training(batch);
        TrainOutput::new(&self.0, item.loss.backward(), item)
    }
}

impl<B: AutodiffBackend, M: Model<B, LossOutput = RegressionOutput<B>>>
    ValidStep<M::Batch, RegressionOutput<B>> for TrainingModel<M, B>
{
    fn step(&self, batch: M::Batch) -> RegressionOutput<B> {
        self.0.forward_training(batch)
    }
}

impl<B: AutodiffBackend, M: Model<B, LossOutput = ClassificationOutput<B>>>
    ValidStep<M::Batch, ClassificationOutput<B>> for TrainingModel<M, B>
{
    fn step(&self, batch: M::Batch) -> ClassificationOutput<B> {
        self.0.forward_training(batch)
    }
}

impl<B: AutodiffBackend, M: Model<B>>
    Module<B> for TrainingModel<M, B>
{
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

impl<B: AutodiffBackend, M: Model<B>>
    AutodiffModule<B> for TrainingModel<M, B>
{
    type InnerModule = M::InnerModule;

    fn valid(&self) -> Self::InnerModule {
        self.0.valid()
    }
}

impl<B: AutodiffBackend, M: Model<B>>
    Debug for TrainingModel<M, B>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(&self.0, f)
    }
}

impl<B: AutodiffBackend, M: Model<B>>
    Display for TrainingModel<M, B>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Display::fmt(&self.0, f)
    }
}

impl<M: Model<B>, B: AutodiffBackend> TrainingModel<M, B> {
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

impl<B: Backend, const T: usize, Item: Into<(Tensor<B, 2>, Tensor<B, T>)>>
    Batcher<Item, RegressionBatch<B, 3, T>> for RegressionBatcher<B>
{
    fn batch(&self, items: Vec<Item>) -> RegressionBatch<B, 3, T> {
        let (inputs, targets) = items
            .into_iter()
            .map(|item| Into::<(Tensor<B, 2>, Tensor<B, T>)>::into(item))
            .map(|(input, target)| {
                let [a, b] = input.dims();
                let len = a * b / 2;
                (input.reshape([1, len, 2]), target)
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
    // #[config(default = 10)]
    pub num_epochs: usize,
    // #[config(default = 128)]
    pub batch_size: usize,
    // #[config(default = 4)]
    pub num_workers: usize,
    // #[config(default = 1342)]
    pub seed: u64,
    // #[config(default = 1.0e-3)]
    pub init_learning_rate: f64,
    // #[config(default = 2)]
    pub stop_condition_epochs: usize,
}


impl<T: Serialize + DeserializeOwned> Config for TrainingConfig<T> {}


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

pub fn train_regression<B, const TN: usize, T, I>(
    artifact_dir: &str,
    training_data_path: PathBuf,
    testing_data_path: PathBuf,
    max_memory_usage: usize,
    config: TrainingConfig<T::Config>,
    device: B::Device,
) -> f64
where
    B: AutodiffBackend,
    T: Model<B, LossOutput = RegressionOutput<B>, Batch = RegressionBatch<B, 3, TN>>,
    T::InnerModule:
        ValidStep<RegressionBatch<B::InnerBackend, 3, TN>, RegressionOutput<B::InnerBackend>>,
    I: Send
        + Sync
        + Clone
        + Debug
        + Into<(Tensor<B, 2>, Tensor<B, TN>)>
        + Into<(Tensor<B::InnerBackend, 2>, Tensor<B::InnerBackend, TN>)>
        + DeserializeOwned
        + 'static,
{
    std::fs::create_dir_all(artifact_dir).ok();
    config
        .save(format!("{artifact_dir}/config.json"))
        .expect("Config should be saved successfully");

    B::seed(config.seed);

    let batcher_train = RegressionBatcher::<B>::new(device.clone());
    let batcher_valid = RegressionBatcher::<B::InnerBackend>::new(device.clone());

    let dataloader_train = DataLoaderBuilder::<I, _>::new(batcher_train)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(
            AcademyDataset::new(training_data_path, max_memory_usage)
        );

    let dataloader_test = DataLoaderBuilder::<I, _>::new(batcher_valid)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(
            AcademyDataset::new(testing_data_path, max_memory_usage)
        );

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
        .save_file(format!("{artifact_dir}/model"), &CompactRecorder::new())
        .expect("Trained model should be saved successfully");

    let loss = loss_state.lock().unwrap().value();
    loss
}
