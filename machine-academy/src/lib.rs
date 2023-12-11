use std::{
    fmt::{Debug, Display},
    marker::PhantomData,
};

use burn::{
    config::Config,
    data::{
        dataloader::{batcher::Batcher, DataLoaderBuilder},
        dataset::SqliteDataset,
    },
    lr_scheduler::noam::NoamLrSchedulerConfig,
    module::AutodiffModule,
    optim::AdamConfig,
    record::CompactRecorder,
    tensor::{
        backend::{AutodiffBackend, Backend},
        Tensor,
    },
    train::{
        metric::{
            store::{Aggregate, Direction, Split},
            CpuTemperature, CpuUse, LossMetric,
        },
        ClassificationOutput, LearnerBuilder, MetricEarlyStoppingStrategy, RegressionOutput,
        StoppingCondition, TrainOutput, TrainStep, ValidStep,
    },
};
use serde::de::DeserializeOwned;

pub mod common;

pub trait Model<B: AutodiffBackend>: AutodiffModule<B> + Display {
    type Input;
    type Output;
    type LossOutput;
    type Batch;

    fn forward(&self, input: Self::Input) -> Self::Output;
    fn forward_training(&self, batch: Self::Batch) -> Self::LossOutput;
}

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

#[derive(Config)]
pub struct TrainingConfig {
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
    pub init_learning_rate: f64,
    #[config(default = 2)]
    pub stop_condition_epochs: usize,
}

pub fn train_regression<B, const TN: usize, T, I>(
    artifact_dir: &str,
    config: TrainingConfig,
    model: T,
    device: B::Device,
) where
    B: AutodiffBackend,
    T: AutodiffModule<B>
        + Display
        + TrainStep<RegressionBatch<B, 3, TN>, RegressionOutput<B>>
        + 'static,
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
            SqliteDataset::from_db_file("data.sqlite", "train")
                .expect("data.sqlite should be readable"),
        );

    let dataloader_test = DataLoaderBuilder::<I, _>::new(batcher_valid)
        .batch_size(config.batch_size)
        .shuffle(config.seed)
        .num_workers(config.num_workers)
        .build(
            SqliteDataset::from_db_file("data.sqlite", "test")
                .expect("data.sqlite should be readable"),
        );

    let num_params = model.num_params();

    let learner = LearnerBuilder::new(artifact_dir)
        .metric_train_numeric(LossMetric::new())
        // .metric_train_numeric(LearningRateMetric::new())
        .metric_train_numeric(CpuTemperature::new())
        .metric_train_numeric(CpuUse::new())
        .metric_valid_numeric(LossMetric::new())
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
            model,
            config.optimizer.init(),
            NoamLrSchedulerConfig::new(config.init_learning_rate)
                .with_model_size(num_params)
                .init(),
        );

    let model_trained = learner.fit(dataloader_train, dataloader_test);

    model_trained
        .save_file(format!("{artifact_dir}/model"), &CompactRecorder::new())
        .expect("Trained model should be saved successfully");
}
