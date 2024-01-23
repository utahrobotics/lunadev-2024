use crossbeam::queue::SegQueue;
use rand::{rngs::SmallRng, Rng, RngCore, SeedableRng};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QuickRng(SmallRng);

static RNGS: SegQueue<SmallRng> = SegQueue::new();

impl Default for QuickRng {
    fn default() -> Self {
        Self(RNGS.pop().unwrap_or_else(SmallRng::from_entropy))
    }
}


impl Drop for QuickRng {
    fn drop(&mut self) {
        // 2% of not returning
        if self.0.gen_bool(0.98) {
            RNGS.push(self.0.clone())
        }
    }
}


impl RngCore for QuickRng {
    fn next_u32(&mut self) -> u32 {
        self.0.next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.0.next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.0.fill_bytes(dest)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand::Error> {
        self.0.try_fill_bytes(dest)
    }
}
