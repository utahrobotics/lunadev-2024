use std::path::Path;

use clap::{command, Parser, Subcommand};
use machine_academy::{
    burn::{
        backend::{
            wgpu::{AutoGraphicsApi, WgpuDevice},
            Autodiff, Wgpu,
        },
        config::Config,
    },
    common::time_series::{GruNetwork, GruNetworkSuperConfig},
    data::create_dataset,
    super_train_regression, SuperTrainingConfig,
};
use smooth_diff_drive::{TrainingItem, TrainingItemGen};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Gen { train_len: usize, test_len: usize },
    Train,
    Test,
    Sample,
}

const BLOCK_MEM_SIZE: usize = 10_000_000;
const SUPER_DIR: &str = "smooth-diff-drive-academy";
type MyBackend = Wgpu<AutoGraphicsApi, f32, i32>;
type MyAutodiffBackend = Autodiff<MyBackend>;

fn main() {
    let cli = Cli::parse();
    let training_data_path = Path::new(SUPER_DIR).join("train");
    let testing_data_path = Path::new(SUPER_DIR).join("test");

    match cli.command {
        Commands::Gen {
            train_len,
            test_len,
        } => {
            let mut gen = TrainingItemGen::default();
            create_dataset(
                train_len,
                training_data_path,
                BLOCK_MEM_SIZE,
                machine_academy::data::DataGenerator::Immut(&mut gen),
            );
            create_dataset(
                test_len,
                testing_data_path,
                BLOCK_MEM_SIZE,
                machine_academy::data::DataGenerator::Immut(&mut gen),
            );
        }
        Commands::Train => {
            let device = WgpuDevice::default();
            let config = SuperTrainingConfig::<GruNetworkSuperConfig>::load(
                Path::new(SUPER_DIR).join("super-config.json"),
            )
            .expect("super config should be valid and readable");
            super_train_regression::<
                MyAutodiffBackend,
                GruNetwork<MyAutodiffBackend>,
                TrainingItem,
                _,
                _,
            >(
                SUPER_DIR.into(),
                100,
                config,
                training_data_path,
                testing_data_path,
                device,
            );
        }
        Commands::Test => {
            let _device = WgpuDevice::default();
            // smooth_diff_drive::test::<MyAutodiffBackend>("/tmp/smooth_diff_drive", device);
        }
        Commands::Sample => {}
    }
}
