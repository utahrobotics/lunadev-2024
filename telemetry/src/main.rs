use std::{net::SocketAddrV4, str::FromStr};

use telemetry::Telemetry;
use unros::{anyhow, Application};

#[unros::main]
async fn main(mut app: Application) -> anyhow::Result<Application> {
    let telemetry = Telemetry::new(
        SocketAddrV4::from_str("10.8.0.6:43721").unwrap(),
        1280,
        720,
        18,
    )
    .await?;

    app.add_node(telemetry);
    Ok(app)
}
