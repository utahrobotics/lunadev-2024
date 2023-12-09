use burn::backend::{wgpu::AutoGraphicsApi, Autodiff, Wgpu};
use burn::grad_clipping::GradientClippingConfig;
use burn::optim::AdamConfig;
use clap::{command, Parser, Subcommand};
use smooth_diff_drive::{create_dataset, train, ModelConfig, TrainingConfig};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Gen {
        train_len: usize,
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
            create_dataset(train_len, "train");
            create_dataset(test_len, "test");
        }
        Commands::Train => {
            let device = burn::backend::wgpu::WgpuDevice::default();
            train::<MyAutodiffBackend>(
                "/tmp/smooth_diff_drive",
                TrainingConfig::new(ModelConfig::new(12), AdamConfig::new().with_grad_clipping(Some(GradientClippingConfig::Value(0.25)))),
                device,
            );
        }
        Commands::Test => {
            let device = burn::backend::wgpu::WgpuDevice::default();
            smooth_diff_drive::test::<MyAutodiffBackend>(
                "/tmp/smooth_diff_drive",
                device,
            );
        }
    }
}
