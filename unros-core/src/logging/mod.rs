use std::{
    path::{Path, PathBuf},
    sync::OnceLock,
    time::Instant,
};

use anyhow::Context;
use chrono::{Datelike, Timelike};

use crate::{logging::eyre::UnrosEyreMessage, RunOptions};

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
    };
}

static SUB_LOGGING_DIR: OnceLock<PathBuf> = OnceLock::new();
pub(crate) static START_TIME: OnceLock<Instant> = OnceLock::new();

/// Initializes the default logging implementation.
///
/// This is called automatically in `run_all` and `async_run_all`, but
/// there may be additional logs produced before these methods that would
/// be ignored if the logger was not set up yet. As such, you may call this
/// method manually, when needed. Calling this multiple times is safe and
/// will not return errors.
pub fn init_logger(run_options: &RunOptions) -> anyhow::Result<()> {
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
        let mut runtime_name = run_options.runtime_name.clone();
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

        let log_folder_name = PathBuf::from(LOGS_DIR).join(&log_folder_name);

        std::fs::DirBuilder::new()
            .create(&log_folder_name)
            .context("Failed to create sub-logging directory. Do we have permissions?")?;

        let _ = START_TIME.set(Instant::now());

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
                ))
            })
            // Add blanket level filter -
            .level(log::LevelFilter::Debug)
            // Output to stdout, files, and other Dispatch configurations
            .chain(
                fern::Dispatch::new().chain(
                    fern::log_file(log_folder_name.join(".log"))
                        .context("Failed to create log file. Do we have permissions?")?,
                ),
            )
            .chain(
                fern::Dispatch::new()
                    .level(log::LevelFilter::Info)
                    // This filter is to avoid logging panics to the console, since rust already does that.
                    // Note that the 'panic' target is set by us in eyre.rs.
                    .filter(|x| x.target() != "panic")
                    .chain(std::io::stdout()),
            )
            // Apply globally
            .apply()
            .expect("Logger should have initialized correctly");

        if run_options.enable_console_subscriber {
            console_subscriber::init();
        }
        Ok(log_folder_name)
    })?;

    Ok(())
}
