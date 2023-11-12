use std::net::{Ipv4Addr, SocketAddrV4};

use telemetry::Telemetry;
use unros_core::{anyhow, async_run_all, log::info, tokio, RunOptions};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut telemetry = Telemetry::new(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 43721));
    let mut watch = telemetry.get_steering_signal().watch();
    tokio::spawn(async move {
        loop {
            let msg = watch.wait_for_change().await;
            info!("{msg:?}");
        }
    });
    let run_options = RunOptions {
        ..Default::default()
    };
    async_run_all([telemetry.into()], run_options).await
}
