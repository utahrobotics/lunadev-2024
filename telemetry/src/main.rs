use std::{net::SocketAddrV4, str::FromStr};

use telemetry::Telemetry;
use unros_core::{anyhow, default_run_options, start_unros_runtime, NodeGroup};

fn main() -> anyhow::Result<()> {
    start_unros_runtime(
        move || async {
            let telemetry = Telemetry::new(
                SocketAddrV4::from_str("10.8.0.6:43721").unwrap(),
                1280,
                720,
                18,
            )
            .await?;

            let mut grp = NodeGroup::default();
            grp.add_node(telemetry);
            grp.run().await
        },
        default_run_options!(),
    )
}
