use rodio::source::SineWave;
use rodio::{OutputStream, Sink};
use std::sync::OnceLock;

static SINK: OnceLock<Sink> = OnceLock::new();

pub fn init_buzz() {
    std::thread::spawn(|| {
        let (_stream, stream_handle) =
            OutputStream::try_default().expect("Failed to open audio stream");
        let sink = Sink::try_new(&stream_handle).unwrap();
        let Ok(()) = SINK.set(sink) else {
            unreachable!();
        };
        let sink = SINK.get().unwrap();
        let source = SineWave::new(300.0);
        sink.append(source);
        sink.pause();
        sink.sleep_until_end();
    });
}

pub fn play_buzz() {
    SINK.get().unwrap().play();
}

pub fn pause_buzz() {
    SINK.get().unwrap().pause();
}
