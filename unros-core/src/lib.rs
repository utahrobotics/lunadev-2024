//! Unros is an experimental alternative to the ROS 1 & 2 frameworks.
//!
//! It is written from the ground up in Rust and seeks to replicate most
//! of the common functionality in ROS while adding some extra features
//! that exploit Rust's abilities.
//!
//! This crate contains the core functionality which defines what this
//! framework offers:
//!
//! 1. The Node trait
//! 2. A complete logging system
//! 3. An asynchronous Node runtime
//! 4. Publisher and Subscribers (analagous to ROS publisher and subscribers)
//! 5. The Service framework (analagous to ROS actions and services)

#![allow(clippy::type_complexity)]
#![feature(once_cell_try)]
#![feature(ptr_metadata)]
#![feature(alloc_layout_extra)]

use std::{
    borrow::Cow, path::Path, sync::{
        atomic::{AtomicBool, Ordering},
        Arc, OnceLock,
    }
};

pub mod logging;
pub mod node;
pub mod pubsub;
pub mod rng;
pub mod runtime;
pub mod service;
pub mod utils;

pub use anyhow;
use config::Config;
pub use log;
pub use rand;
pub use rayon;
use serde::Deserialize;
pub use tokio;


/// A simple primitive for tracking when clones of itself have been dropped.
///
/// Clones of this are all connected such that if any clone is dropped, all other
/// clones will be aware of that. For an object that only tracks if its clones were
/// dropped without updating them when itself is dropped, refer to `ObservingDropCheck`.
#[derive(Clone)]
pub struct DropCheck {
    dropped: Arc<AtomicBool>,
    update_on_drop: bool,
}

impl Default for DropCheck {
    fn default() -> Self {
        Self {
            dropped: Arc::default(),
            update_on_drop: true,
        }
    }
}

impl Drop for DropCheck {
    fn drop(&mut self) {
        if self.update_on_drop {
            self.dropped.store(true, Ordering::SeqCst);
        }
    }
}

impl DropCheck {
    /// Returns true iff a clone has been dropped.
    #[must_use]
    pub fn has_dropped(&self) -> bool {
        self.dropped.load(Ordering::SeqCst)
    }

    /// Forget if a clone has been dropped.
    pub fn reset(&self) {
        self.dropped.store(true, Ordering::SeqCst);
    }

    /// Ensures that this `DropCheck` will update its clones when dropped.
    pub fn update_on_drop(&mut self) {
        self.update_on_drop = true;
    }

    /// Ensures that this `DropCheck` will *not* update its clones when dropped.
    pub fn dont_update_on_drop(&mut self) {
        self.update_on_drop = true;
    }

    /// Get an observer to this `DropCheck` and its clones.
    pub fn get_observing(&self) -> ObservingDropCheck {
        ObservingDropCheck {
            dropped: self.dropped.clone(),
        }
    }
}

/// A similar object to `DropCheck`, however, none of its clones
/// will be updated when this is dropped.
///
/// This is equivalent to calling `dont_update_on_drop` on `DropCheck`,
/// except that this is enforced statically.
#[derive(Clone)]
pub struct ObservingDropCheck {
    dropped: Arc<AtomicBool>,
}

impl ObservingDropCheck {
    /// Returns true iff a clone has been dropped.
    #[must_use]
    pub fn has_dropped(&self) -> bool {
        self.dropped.load(Ordering::SeqCst)
    }
}

static CONFIG: OnceLock<Config> = OnceLock::new();

/// Deserialize environment variables and the default config file into the given generic type.
pub fn get_env<'de, T: Deserialize<'de>>() -> anyhow::Result<T> {
    let mut config = Config::builder().add_source(config::Environment::with_prefix(""));
    if Path::new("settings.toml").exists() {
        config = config.add_source(config::File::with_name("settings.toml"));
    }
    if Path::new(".env").exists() {
        config = config.add_source(config::File::with_name(".env"));
    }
    CONFIG
        .get_or_try_init(|| config.build())?
        .clone()
        .try_deserialize()
        .map_err(Into::into)
}

#[derive(Clone, Debug)]
pub struct DontDrop {
    pub name: Cow<'static, str>,
    pub ignore_drop: bool
}


impl DontDrop {
    pub fn new(name: impl Into<Cow<'static, str>>) -> Self {
        Self {
            name: name.into(),
            ignore_drop: false,
        }
    }
}


impl Drop for DontDrop {
    fn drop(&mut self) {
        if !self.ignore_drop {
            log::warn!("{} was dropped", self.name);
        }
    }
}
