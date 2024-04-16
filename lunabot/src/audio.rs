use rodio::source::{SineWave, Source};
use rodio::{OutputStream, Sink};
use std::time::Duration;
use unros::{log::error, rayon};

pub fn play_buzz(freq: f32, duration: f32) {
    rayon::spawn(move || {
        let (_stream, stream_handle) = match OutputStream::try_default() {
            Ok(x) => x,
            Err(e) => {
                error!("Failed to play_buzz: {e}");
                return;
            }
        };
        let sink = Sink::try_new(&stream_handle).unwrap();

        // Add a dummy source of the sake of the example.
        let source = SineWave::new(freq).take_duration(Duration::from_secs_f32(duration));
        sink.append(source);

        // The sound plays in a separate thread. This call will block the current thread until the sink
        // has finished playing all its queued sounds.
        sink.sleep_until_end();
    });
}
