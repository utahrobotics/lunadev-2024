use std::net::SocketAddr;

use clap::{Parser, Subcommand};
use tokio::{io::AsyncWriteExt, net::TcpStream};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// does testing things
    Stop {
        /// lists test values
        #[arg(short, long)]
        addr: SocketAddr,
    },
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Stop { addr } => {
            let mut stream = TcpStream::connect(addr).await?;
            stream.write_all(b"stop\n").await?;
            tokio::io::copy(&mut stream, &mut tokio::io::stdout()).await?;
        }
    }

    Ok(())
}
