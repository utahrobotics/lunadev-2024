use cpal::traits::{HostTrait, StreamTrait};
use cpal::{SampleFormat, SampleRate};
use rodio::{Decoder, DeviceTrait, OutputStream, Sink, Source};
use unros::pubsub::{Publisher, PublisherRef};
use std::{fs::File, io::BufReader};
use std::sync::OnceLock;
use std::time::Duration;
use unros::log::{error, info, warn};

static BUZZ_SINK: OnceLock<Sink> = OnceLock::new();
static MUSIC_SINK: OnceLock<Sink> = OnceLock::new();
pub static MIC_PUB: OnceLock<PublisherRef<f32>> = OnceLock::new();

pub fn init_audio() {
    let host = cpal::default_host();

    'play: {
        let devices = match host.output_devices() {
            Ok(devices) => devices,
            Err(e) => {
                error!("Unable to get default output devices: {e}");
                break 'play;
            }
        };

        for device in devices {
            if let Ok(name) = device.name() {
                if &name != "pulse" {
                    continue;
                }
                match OutputStream::try_from_device(&device) {
                    Ok((stream, stream_handle)) => {
                        std::mem::forget(stream);
            
                        // Buzz
                        let sink = Sink::try_new(&stream_handle).unwrap();
                        let source = SquareWave::new(100.0);
                        sink.append(source);
                        sink.pause();
                        let Ok(()) = BUZZ_SINK.set(sink) else {
                            unreachable!();
                        };
                        info!("Buzzer initialized");
                        
                        // Music
                        match File::open("music.mp3") {
                            Ok(file) => {
                                match Decoder::new_looped(BufReader::new(file)) {
                                    Ok(source) => {
                                        let sink = Sink::try_new(&stream_handle).unwrap();
                                        sink.append(source);
                                        sink.pause();
                                        let Ok(()) = MUSIC_SINK.set(sink) else {
                                            unreachable!();
                                        };
                                        info!("Music initialized");
                                    }
                                    Err(e) => {
                                        warn!("Failed to decode music.mp3: {e}");
                                    }
                                };
                            }
                            Err(e) => {
                                warn!("Failed to load music.mp3: {e}");
                            }
                        }
                    }
                    Err(e) => {
                        error!("Failed to open audio stream: {e}");
                    }
                }
                break 'play;
            }
        }
    }

    // 'mic: {
    //     let devices = match host.input_devices() {
    //         Ok(devices) => devices,
    //         Err(e) => {
    //             error!("Unable to get default input devices: {e}");
    //             break 'mic;
    //         }
    //     };
    
    //     for device in devices {
    //         if let Ok(name) = device.name() {
    //             println!("a {name}");
    //             if &name != "pulse" {
    //                 continue;
    //             }
    //             let configs = match device.supported_input_configs() {
    //                 Ok(x) => x,
    //                 Err(e) => {
    //                     warn!("Unable to get default input device configs: {e}");
    //                     break 'mic;
    //                 }
    //             };
    //             let Some(config_range) = configs.filter(|config_range| config_range.sample_format() == SampleFormat::F32).next() else {
    //                 warn!("Input device does not support f32");
    //                 break 'mic;
    //             };
    //             let Some(config) = config_range.try_with_sample_rate(SampleRate(24000)) else {
    //                 warn!("Input device does not support the given sample rate");
    //                 break 'mic;
    //             };
    //             let publisher = Publisher::default();
    //             let Ok(()) = MIC_PUB.set(publisher.get_ref()) else {
    //                 unreachable!()
    //             };
    //             let stream = match device.build_input_stream(
    //                 &config.into(),
    //                 move |data: &[f32], _: &_| {
    //                     for d in data {
    //                         publisher.set(*d);
    //                     }
    //                 },
    //                 move |err| {
    //                     error!("an error occurred on stream: {}", err);
    //                 },
    //                 None,
    //             ) {
    //                 Ok(x) => x,
    //                 Err(e) => {
    //                     warn!("Failed to initialize audio input stream: {e}");
    //                     break 'mic;
    //                 }
    //             };
        
    //             match stream.play() {
    //                 Ok(()) => {}
    //                 Err(e) => {
    //                     warn!("Failed to start up audio input stream: {e}");
    //                     break 'mic;
    //                 }
    //             }
    //             std::mem::forget(stream);
    //             info!("Mic initialized");
    //             break 'mic;
    //         }
    //     }

    //     // let Some(device) = host.default_input_device() else {
    //     //     warn!("Unable to get default input device");
    //     //     break 'mic;
    //     // };
        
    // }
}

pub fn play_buzz() {
    if let Some(sink) = BUZZ_SINK.get() {
        sink.play();
    }
}

pub fn pause_buzz() {
    if let Some(sink) = BUZZ_SINK.get() {
        sink.pause();
    }
}

pub fn play_music() {
    if let Some(sink) = MUSIC_SINK.get() {
        sink.play();
    }
}

pub fn pause_music() {
    if let Some(sink) = MUSIC_SINK.get() {
        sink.pause();
    }
}

#[derive(Clone, Debug)]
pub struct SquareWave {
    freq: f32,
    num_sample: usize,
}

impl SquareWave {
    /// The frequency of the sine.
    #[inline]
    pub fn new(freq: f32) -> SquareWave {
        SquareWave {
            freq,
            num_sample: 0,
        }
    }
}

impl Iterator for SquareWave {
    type Item = f32;

    #[inline]
    fn next(&mut self) -> Option<f32> {
        self.num_sample = self.num_sample.wrapping_add(1);

        let value = 2.0 * std::f32::consts::PI * self.freq * self.num_sample as f32 / 48000.0;
        if value.sin() > 0.0 {
            Some(1.0)
        } else {
            Some(-1.0)
        }
    }
}

impl Source for SquareWave {
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
        48000
    }

    #[inline]
    fn total_duration(&self) -> Option<Duration> {
        None
    }
}
