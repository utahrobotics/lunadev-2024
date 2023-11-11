use std::net::{Ipv4Addr, SocketAddrV4};

use telemetry::Telemetry;
use unros_core::{anyhow, log::info, async_run_all, RunOptions, tokio};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut telemetry = Telemetry::new(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 43721));
    let mut watch = telemetry.get_steering_signal().watch();
    tokio::spawn(async move {
        loop {
            info!("{:?}", watch.get().await);
        }
    });
    let run_options = RunOptions {
        ..Default::default()
    };
    async_run_all([telemetry.into()], run_options).await
}
