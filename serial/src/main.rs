use std::io::{stdin, stdout, BufRead, Write};

use serial::SerialConnection;
use unros_core::{
    anyhow,
    bytes::Bytes,
    default_run_options,
    pubsub::{Publisher, Subscriber},
    spawn_persistent_thread, start_unros_runtime, tokio,
};

fn main() -> anyhow::Result<()> {
    start_unros_runtime(
        |mut app| async {
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

            spawn_persistent_thread(move || {
                let stdin = stdin();
                let mut stdin = stdin.lock();

                let mut line = String::new();
                loop {
                    line.clear();
                    stdin.read_line(&mut line).expect("Failed to read a line");
                    line += "\r";
                    let line = line.clone().into_bytes();
                    write_signal.set(line.into());
                }
            });

            app.add_node(serial);

            Ok(app)
        },
        default_run_options!(),
    )
}
