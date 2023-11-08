use std::net::{Ipv4Addr, SocketAddrV4};

use telemetry::Telemetry;
use unros_core::{anyhow, log::info, run_all, RunOptions, Signal};

fn main() -> anyhow::Result<()> {
    let mut telemetry = Telemetry::new(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 43721));
    telemetry.get_steering_signal().connect_to(|x| {
        info!("{x:?}");
    });
    let run_options = RunOptions {
        ..Default::default()
    };
    run_all([telemetry.into()], run_options)
}
