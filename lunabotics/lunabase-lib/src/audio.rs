use std::time::Duration;

use crossbeam::atomic::AtomicCell;
use godot::log::godot_error;
use rodio::{OutputStream, Sink, Source};
use unros::log::error;

pub static MIC_PLAYBACK: MicPlayback = MicPlayback { cell: AtomicCell::new(0.0) };

pub struct MicPlayback {
    pub cell: AtomicCell<f32>
}


impl<'a> Iterator for &'a MicPlayback {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.cell.load())
    }
}


impl<'a> Source for &'a MicPlayback {
    #[inline]
    fn current_frame_len(&self) -> Option<usize> {
        None
    }

    #[inline]
    fn channels(&self) -> u16 {
        1
    }

    #[inline]
    fn sample_rate(&self) -> u32 {
        24000
    }

    #[inline]
    fn total_duration(&self) -> Option<Duration> {
        None
    }
}


pub fn init_audio() {
    match OutputStream::try_default() {
        Ok((stream, stream_handle)) => {
            std::mem::forget(stream);
            let sink = Sink::try_new(&stream_handle).unwrap();
            sink.append(&MIC_PLAYBACK);
        }
        Err(e) => {
            godot_error!("Failed to initialize audio: {:?}", e);
            error!("Failed to initialize audio: {:?}", e);
        }
    }
}