//! A simple thread local pseudorandom number generator to avoid seeding new ones frequently.

use rand::{rngs::SmallRng, SeedableRng};

use crate::utils::{ThreadLocalResource, ThreadLocalResourceExt, ThreadLocalResourceGuard};


thread_local! {
    static RNG: ThreadLocalResource<SmallRng> = ThreadLocalResource::new(|| {
        SmallRng::from_entropy()
    });
}

/// Quickly retrieves a pseudorandom number generator that was seeded securely.
/// 
/// This rng is stored thread-locally and is not reseeded.
pub fn quick_rng() -> ThreadLocalResourceGuard<SmallRng> {
    RNG.take()
}
