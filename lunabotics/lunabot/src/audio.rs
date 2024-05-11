use rodio::source::SineWave;
use rodio::{OutputStream, Sink};
use std::sync::OnceLock;
use unros::log::error;

static SINK: OnceLock<Sink> = OnceLock::new();

pub fn init_buzz() {
    match OutputStream::try_default() {
        Ok((stream, stream_handle)) => {
            std::mem::forget(stream);
            let sink = Sink::try_new(&stream_handle).unwrap();
            let source = SineWave::new(1000.0);
            sink.append(source);
            sink.pause();
            let Ok(()) = SINK.set(sink) else {
                unreachable!();
            };
        }
        Err(e) => {
            error!("Failed to open audio stream: {e}");
        }
    }
}

pub fn play_buzz() {
    if let Some(sink) = SINK.get() {
        sink.play();
    }
}

pub fn pause_buzz() {
    if let Some(sink) = SINK.get() {
        sink.pause();
    }
}
