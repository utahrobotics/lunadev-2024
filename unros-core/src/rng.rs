//! A simple pool of pseudorandom number generators to avoid seeding new ones frequently.

use rand::{rngs::SmallRng, SeedableRng};

use crate::utils::{ResourceGuard, ResourceQueue};

static RNGS: ResourceQueue<SmallRng> = ResourceQueue::new(16, SmallRng::from_entropy);

/// Quickly retrieves a pseudorandom number generator that was seeded securely from a pool of prngs.
///
/// This is quite efficient as seeding a prng can be costly. The returned prng will be returned back
/// to the pool when it is dropped.
pub fn quick_rng() -> ResourceGuard<'static, SmallRng> {
    RNGS.get()
}
