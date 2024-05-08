use rodio::cpal::traits::HostTrait;
use rodio::source::SineWave;
use rodio::{cpal, DeviceTrait, OutputStream, Sink};
use unros::log;
use std::sync::OnceLock;

static SINK: OnceLock<Box<[Sink]>> = OnceLock::new();

pub fn init_buzz() {
    let mut sinks = vec![];
    let (stream, stream_handle) =
        OutputStream::try_default().expect("Failed to open audio stream");
    std::mem::forget(stream);
    let sink = Sink::try_new(&stream_handle).unwrap();
    let source = SineWave::new(300.0);
    sink.append(source);
    sink.pause();
    sinks.push(sink);
    let Ok(()) = SINK.set(sinks.into_boxed_slice()) else {
        unreachable!();
    };
}

pub fn play_buzz() {
    SINK.get().unwrap().iter().for_each(Sink::play);
}

pub fn pause_buzz() {
    SINK.get().unwrap().iter().for_each(Sink::pause);
}
