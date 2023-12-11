use burn::backend::{wgpu::AutoGraphicsApi, Autodiff, Wgpu};
use burn::grad_clipping::GradientClippingConfig;
use burn::optim::AdamConfig;
use bytemuck::cast_slice;
use clap::{command, Parser, Subcommand};
use rusqlite::Connection;
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
    Train {
        hidden_size: usize,
        learning_rate: f64,
    },
    Test,
    Sample,
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
        Commands::Train {
            learning_rate,
            hidden_size,
        } => {
            let device = burn::backend::wgpu::WgpuDevice::default();
            train::<MyAutodiffBackend>(
                "/tmp/smooth_diff_drive",
                TrainingConfig::new(
                    ModelConfig::new(hidden_size),
                    AdamConfig::new().with_grad_clipping(Some(GradientClippingConfig::Value(0.25))),
                )
                .with_learning_rate(learning_rate),
                device,
            );
        }
        Commands::Test => {
            let device = burn::backend::wgpu::WgpuDevice::default();
            smooth_diff_drive::test::<MyAutodiffBackend>("/tmp/smooth_diff_drive", device);
        }
        Commands::Sample => {
            let db =
                Connection::open("data.sqlite").expect("data.sqlite should have been readable");
            db.query_row(
                "SELECT * FROM train ORDER BY RANDOM() LIMIT 1;",
                (),
                |row| {
                    let input: Vec<u8> = row.get("input").unwrap();
                    let target: f32 = row.get("target").unwrap();
                    for [a, b] in cast_slice::<_, [f32; 2]>(&input) {
                        println!("[{a:.2}] [{b:.2}]");
                    }
                    println!("-----------");
                    println!("{target:.2}");
                    Ok(())
                },
            )
            .expect("SELECT should have succeeded");
        }
    }
}
