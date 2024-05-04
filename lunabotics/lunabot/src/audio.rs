use rodio::source::SineWave;
use rodio::{OutputStream, Sink};
use std::sync::OnceLock;
use unros::anyhow::{self, Context};

static SINK: OnceLock<Sink> = OnceLock::new();

pub fn init_buzz() -> anyhow::Result<()> {
    let (_stream, stream_handle) =
        OutputStream::try_default().context("Failed to open audio stream")?;
    let sink = Sink::try_new(&stream_handle).unwrap();
    let Ok(()) = SINK.set(sink) else {
        unreachable!();
    };
    let sink = SINK.get().unwrap();
    let source = SineWave::new(4000.0);
    sink.append(source);
    sink.pause();

    std::thread::spawn(|| {
        sink.sleep_until_end();
    });

    Ok(())
}

pub fn play_buzz() {
    SINK.get().unwrap().play();
}

pub fn pause_buzz() {
    SINK.get().unwrap().pause();
}
