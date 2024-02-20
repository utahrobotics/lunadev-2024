use std::{net::SocketAddrV4, str::FromStr};

use telemetry::Telemetry;
use unros_core::{anyhow, async_run_all, default_run_options, tokio};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let telemetry = Telemetry::new(
        SocketAddrV4::from_str("10.8.0.6:43721").unwrap(),
        1280,
        720,
        18,
    )
    .await?;
    async_run_all([telemetry.into()], default_run_options!()).await
}
