use std::io::{stdout, Write};

use serial::{Bytes, SerialConnection};
use unros::{
    pubsub::{Publisher, Subscriber}, runtime::MainRuntimeContext, tokio, node::AsyncNode
};

#[unros::main]
async fn main(ctx: MainRuntimeContext) {
    // "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00"
    let serial = SerialConnection::new(
        "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6616407e3496e28-if00",
        115200,
        true,
    );
    let sub = Subscriber::<Bytes>::new(8);
    serial
        .msg_received_pub()
        .accept_subscription(sub.create_subscription());
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
    let write_signal: Publisher<_> = Default::default();
    write_signal.accept_subscription(serial.message_to_send_sub());
    serial.spawn(ctx.make_context("serial"));

    let carriage_return_bytes = Bytes::from_static(b"\r");

    ctx.wait_for_exit_with_repl(move |line| {
        write_signal.set(line.to_string().into_bytes().into());
        write_signal.set(carriage_return_bytes.clone());
    }).await;
}
