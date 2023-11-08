use std::{
    io::{stdin, stdout, BufRead, Write},
    sync::Arc,
};

use serial::SerialConnection;
use unros_core::{
    anyhow, async_run_all, tokio::{runtime::Handle, self}, tokio_rayon, FnNode, OwnedSignal, Signal, init_logger,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let run_options = Default::default();
    init_logger(&run_options)?;
    // "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00"
    let mut serial = SerialConnection::new("/dev/ttyUSB0".into(), 115200, true).await?;
    serial.get_msg_received_signal().connect_to(|x| {
        stdout()
            .lock()
            .write_all(&x)
            .expect("Stdout should have been writable");
    });
    let mut write_signal: OwnedSignal<_> = Default::default();
    serial.connect_from(&mut write_signal);
    let write_signal = Arc::new(write_signal);

    let fn_node = FnNode::new(move || async {
        let handle = Handle::current();

        tokio_rayon::spawn(move || {
            let stdin = stdin();
            let mut stdin = stdin.lock();

            let mut line = String::new();
            loop {
                line.clear();
                stdin.read_line(&mut line)?;
                let line = line.clone().into_bytes().into_boxed_slice();
                let write_signal = write_signal.clone();
                handle.spawn(async move {
                    write_signal.emit(line.into()).await;
                });
            }
        }).await
    });

    async_run_all([serial.into(), fn_node.into()], run_options).await
}
