use std::net::{Ipv4Addr, SocketAddrV4};

use telemetry::Telemetry;
use unros_core::{anyhow, async_run_all, default_run_options, log::info, tokio};

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
    async_run_all([telemetry.into()], default_run_options!()).await
}
