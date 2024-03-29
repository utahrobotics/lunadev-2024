//! A good logging solution is instrumental to big projects with rapid
//! prototyping cycles.
//!
//! Having plenty sources of data while remaining highly configurable is
//! the goal of Unros, and this module provides that.

use std::{
    panic::catch_unwind,
    path::{Path, PathBuf},
    sync::{Arc, Mutex, OnceLock},
    time::Instant,
};

use anyhow::Context;
use chrono::{Datelike, Timelike};
use fern::colors::{Color, ColoredLevelConfig};
use log::Level;

use crate::{
    logging::eyre::UnrosEyreMessage,
    pubsub::{Publisher, PublisherRef},
    RunOptions,
};

pub mod dump;
mod eyre;
pub mod rate;

/// Sets up a locally available set of logging macros.
///
/// The `log` crate allows users to configure the `target`
/// parameter of a log, allowing developers to better filter
/// messages by file. Unros takes this one step further by
/// automatically setting this `target` parameter to be the name
/// of the current node (as passed by the context). This allows
/// two nodes of the same class to have different log targets
/// if their names differ, which should help you to identify
/// issues faster.
#[macro_export]
macro_rules! setup_logging {
    ($context: ident) => {
        setup_logging!($context $)
    };
    ($context: ident $dol:tt) => {
        let _context = &$context;
        #[allow(unused_macros)]
        macro_rules! info {
            ($dol($dol arg:tt)+) => {
                $crate::log::info!(target: $context.get_name(), $dol ($dol arg)+)
            };
        }
        #[allow(unused_macros)]
        macro_rules! warn {
            ($dol ($dol arg:tt)+) => {
                $crate::log::warn!(target: $context.get_name(), $dol ($dol arg)+)
            };
        }
        #[allow(unused_macros)]
        macro_rules! error {
            ($dol ($dol arg:tt)+) => {
                $crate::log::error!(target: $context.get_name(), $dol ($dol arg)+)
            };
        }
        #[allow(unused_macros)]
        macro_rules! debug {
            ($dol ($dol arg:tt)+) => {
                $crate::log::debug!(target: $context.get_name(), $dol ($dol arg)+)
            };
        }
        #[allow(unused_macros)]
        macro_rules! trace {
            ($dol ($dol arg:tt)+) => {
                $crate::log::trace!(target: $context.get_name(), $dol ($dol arg)+)
            };
        }
    };
}

static SUB_LOGGING_DIR: OnceLock<PathBuf> = OnceLock::new();
pub(crate) static START_TIME: OnceLock<Instant> = OnceLock::new();
static LOG_PUB: OnceLock<PublisherRef<Arc<str>>> = OnceLock::new();

/// Gets a reference to the `Publisher` for logs.
///
/// # Panics
/// Panics if the logger has not been initialized. If this method
/// is called inside of or after `start_unros_runtime`, the logger is
/// always initialized.
pub fn get_log_pub() -> PublisherRef<Arc<str>> {
    LOG_PUB.get().unwrap().clone()
}

#[derive(Default)]
struct LogPub {
    publisher: Mutex<Publisher<Arc<str>>>,
}

impl log::Log for LogPub {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        !(metadata.target() == "unros_core::logging::dump" && metadata.level() == Level::Info)
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        self.publisher
            .lock()
            .unwrap()
            .set(format!("{}", record.args()).into_boxed_str().into());
    }

    fn flush(&self) {}
}

/// Initializes the default logging implementation.
///
/// This is called automatically in `run_all` and `async_run_all`, but
/// there may be additional logs produced before these methods that would
/// be ignored if the logger was not set up yet. As such, you may call this
/// method manually, when needed. Calling this multiple times is safe and
/// will not return errors.
pub(super) fn init_logger(run_options: &RunOptions) -> anyhow::Result<()> {
    const LOGS_DIR: &str = "logs";

    SUB_LOGGING_DIR.get_or_try_init::<_, anyhow::Error>(|| {
        color_eyre::config::HookBuilder::default()
            .panic_message(UnrosEyreMessage)
            .install()
            .map_err(|e| anyhow::anyhow!(e))?;

        if !AsRef::<Path>::as_ref(LOGS_DIR)
            .try_exists()
            .context("Failed to check if logging directory exists. Do we have permissions?")?
        {
            std::fs::DirBuilder::new()
                .create(LOGS_DIR)
                .context("Failed to create logging directory. Do we have permissions?")?;
        }
        let mut runtime_name = run_options.runtime_name.to_string();
        if !runtime_name.is_empty() {
            runtime_name = "=".to_string() + &runtime_name;
        }

        let datetime = chrono::Local::now();
        let log_folder_name = format!(
            "{}-{:0>2}-{:0>2}={:0>2}-{:0>2}-{:0>2}{}",
            datetime.year(),
            datetime.month(),
            datetime.day(),
            datetime.hour(),
            datetime.minute(),
            datetime.second(),
            runtime_name,
        );

        let log_folder_name = PathBuf::from(LOGS_DIR).join(log_folder_name);

        std::fs::DirBuilder::new()
            .create(&log_folder_name)
            .context("Failed to create sub-logging directory. Do we have permissions?")?;

        let colors = ColoredLevelConfig::new()
            .warn(Color::Yellow)
            .error(Color::Red)
            .trace(Color::BrightBlack);

        let _ = START_TIME.set(Instant::now());

        let mut log_pub = LogPub::default();
        let _ = LOG_PUB.set(log_pub.publisher.get_mut().unwrap().get_ref());
        let log_pub: Box<dyn log::Log> = Box::new(log_pub);

        fern::Dispatch::new()
            // Add blanket level filter -
            .level(log::LevelFilter::Debug)
            // Output to stdout, files, and other Dispatch configurations
            .chain(
                fern::Dispatch::new()
                    .format(move |out, message, record| {
                        let secs = START_TIME.get().unwrap().elapsed().as_secs_f32();
                        out.finish(format_args!(
                            "[{:0>1}:{:.2} {} {}] {}",
                            (secs / 60.0).floor(),
                            secs % 60.0,
                            record.level(),
                            record.target(),
                            message
                        ));
                    })
                    .chain(
                        fern::log_file(log_folder_name.join(".log"))
                            .context("Failed to create log file. Do we have permissions?")?,
                    )
                    .chain(log_pub),
            )
            .chain(
                fern::Dispatch::new()
                    .level(log::LevelFilter::Info)
                    // This filter is to avoid logging panics to the console, since rust already does that.
                    // Note that the 'panic' target is set by us in eyre.rs.
                    .filter(|x| x.target() != "panic")
                    .filter(|x| {
                        !(x.target() == "unros_core::logging::dump" && x.level() == Level::Info)
                    })
                    .format(move |out, message, record| {
                        let secs = START_TIME.get().unwrap().elapsed().as_secs_f32();
                        out.finish(format_args!(
                            "\x1B[{}m[{:0>1}:{:.2} {}] {}\x1B[0m",
                            colors.get_color(&record.level()).to_fg_str(),
                            (secs / 60.0).floor(),
                            secs % 60.0,
                            record.target(),
                            message
                        ));
                    })
                    .chain(std::io::stdout()),
            )
            // Apply globally
            .apply()
            .context("Logger should have initialized correctly")?;

        if run_options.enable_console_subscriber {
            if let Err(e) = catch_unwind(console_subscriber::init) {
                log::error!("Failed to initialize console subscriber: {e:?}");
            }
        }
        Ok(log_folder_name)
    })?;

    Ok(())
}
