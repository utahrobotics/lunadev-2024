use std::io::{stdin, stdout, BufRead, Write};

use serial::SerialConnection;
use unros_core::{
    anyhow, async_run_all,
    bytes::Bytes,
    default_run_options,
    logging::init_logger,
    pubsub::{Publisher, Subscriber},
    tokio, tokio_rayon, FnNode, RuntimeContext,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = default_run_options!();
    init_logger(&run_options)?;
    // "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00"
    let mut serial = SerialConnection::new("/dev/ttyACM1".into(), 115200, true).await;
    let mut sub = Subscriber::<Bytes>::new(8);
    serial.accept_msg_received_sub(sub.create_subscription());
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
    let mut write_signal: Publisher<_> = Default::default();
    write_signal.accept_subscription(serial.create_message_to_send_sub());
    // let write_signal = Arc::new(write_signal);

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
                // let write_signal = write_signal.clone();
                write_signal.set(line.into());
            }
        })
        .await
    });

    async_run_all([serial.into(), fn_node.into()], run_options).await
}
