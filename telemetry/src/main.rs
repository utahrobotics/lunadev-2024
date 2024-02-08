use std::{net::SocketAddrV4, str::FromStr};

use telemetry::Telemetry;
use unros_core::{anyhow, default_run_options, run_all};

fn main() -> anyhow::Result<()> {
    let telemetry = Telemetry::new(SocketAddrV4::from_str("10.8.0.6:43721").unwrap());
    run_all([telemetry.into()], default_run_options!())
}
