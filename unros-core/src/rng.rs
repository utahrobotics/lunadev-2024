use rand::{rngs::SmallRng, SeedableRng};

use crate::utils::{ResourceGuard, ResourceQueue};

static RNGS: ResourceQueue<SmallRng> = ResourceQueue::new(16, SmallRng::from_entropy);

pub fn quick_rng() -> ResourceGuard<'static, SmallRng> {
    RNGS.get()
}