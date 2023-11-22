use std::{
    io::{stdin, stdout, BufRead, Write},
    sync::Arc,
};

use serial::SerialConnection;
use unros_core::{
    anyhow, async_run_all, default_run_options, init_logger, signal::Signal, tokio, tokio_rayon,
    FnNode, RuntimeContext,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = default_run_options!();
    init_logger(&run_options)?;
    // "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00"
    let mut serial = SerialConnection::new("/dev/ttyACM1".into(), 115200, true).await;
    let mut sub = serial.get_msg_received_signal().subscribe_unbounded();
    tokio::spawn(async move {
        loop {
            let x = sub.recv().await;
            let stdout = stdout();
            let mut stdout = stdout.lock();
            stdout
                .write_all(&x)
                .expect("Stdout should have been writable");
            stdout.flush().expect("Stdout should have been flushable");
        }
    });
    let mut write_signal: Signal<_> = Default::default();
    serial.message_to_send_subscription(write_signal.get_ref().subscribe_bounded());
    let write_signal = Arc::new(write_signal);

    let fn_node = FnNode::new(move |_context: RuntimeContext| async {
        tokio_rayon::spawn(move || {
            let stdin = stdin();
            let mut stdin = stdin.lock();

            let mut line = String::new();
            loop {
                line.clear();
                stdin.read_line(&mut line)?;
                line += "\r";
                let line = line.clone().into_bytes();
                let write_signal = write_signal.clone();
                write_signal.set(line.into());
            }
        })
        .await
    });

    async_run_all([serial.into(), fn_node.into()], run_options).await
}
