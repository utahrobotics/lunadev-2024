use burn::backend::{wgpu::AutoGraphicsApi, Autodiff, Wgpu};
use burn::optim::AdamConfig;
use clap::{command, Parser, Subcommand};
use smooth_diff_drive::{create_dataset, train, ModelConfig, TrainingConfig};
use tokio_rayon::rayon::join;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Gen {
        #[arg(short, long)]
        train_len: usize,
        #[arg(short, long)]
        test_len: usize,
    },
    Train,
    Test,
}

type MyBackend = Wgpu<AutoGraphicsApi, f32, i32>;
type MyAutodiffBackend = Autodiff<MyBackend>;

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Gen {
            train_len,
            test_len,
        } => {
            join(
                || {
                    create_dataset(train_len, "train.json");
                },
                || {
                    create_dataset(test_len, "test.json");
                },
            );
        }
        Commands::Train => {
            let device = burn::backend::wgpu::WgpuDevice::default();
            train::<MyAutodiffBackend>(
                "/tmp/smooth_diff_drive",
                TrainingConfig::new(ModelConfig::new(8), AdamConfig::new()),
                device,
            );
        }
        Commands::Test => {
            todo!()
        }
    }
}
