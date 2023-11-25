use std::time::Instant;

use log::info;

pub struct RateLogger {
    start: Instant,
    epoch: usize,
    count: usize,
    pub window_size: f32,
    pub target: String,
}

impl Default for RateLogger {
    fn default() -> Self {
        Self {
            start: Instant::now(),
            epoch: 0,
            count: 0,
            window_size: 1.0,
            target: "rate-logger".into(),
        }
    }
}

impl RateLogger {
    pub fn increment(&mut self) {
        let current_epoch = (self.start.elapsed().as_secs_f32() / self.window_size) as usize;
        if self.epoch != current_epoch {
            info!(target: &self.target, "{:.4} Hz", self.count as f32 / self.window_size);
            self.count = 0;
            self.epoch = current_epoch;
        }
        self.count += 1;
    }
}
