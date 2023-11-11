use std::{
    future::Future,
    path::{Path, PathBuf},
    pin::Pin,
    sync::{Arc, Once},
    time::Instant,
};

pub mod signal;

pub use anyhow;
use anyhow::Context;
pub use async_trait::async_trait;
pub use bytes;
pub use log;
use log::{error, info, warn};
use serde::Deserialize;
pub use tokio;
use tokio::{
    sync::broadcast,
    task::JoinSet,
};
pub use tokio_rayon::{self, rayon};

#[async_trait]
pub trait Node: Send + 'static {
    const DEFAULT_NAME: &'static str;

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()>;
}

pub struct FnNode<Fut, F>
where
    Fut: Future<Output = anyhow::Result<()>> + Send + 'static,
    F: FnOnce(RuntimeContext) -> Fut + Send + 'static,
{
    f: F,
}

impl<Fut, F> FnNode<Fut, F>
where
    Fut: Future<Output = anyhow::Result<()>> + Send + 'static,
    F: FnOnce(RuntimeContext) -> Fut + Send + 'static,
{
    pub fn new(f: F) -> Self {
        Self { f }
    }
}

#[async_trait]
impl<Fut, F> Node for FnNode<Fut, F>
where
    Fut: Future<Output = anyhow::Result<()>> + Send + 'static,
    F: FnOnce(RuntimeContext) -> Fut + Send + 'static,
{
    const DEFAULT_NAME: &'static str = "fn_node";

    async fn run(self, context: RuntimeContext) -> anyhow::Result<()> {
        (self.f)(context).await
    }
}

#[derive(Clone)]
pub struct RuntimeContext {
    name: Arc<str>,
}

impl RuntimeContext {
    pub fn get_name(&self) -> &Arc<str> {
        &self.name
    }
}

struct RunError {
    critical: bool,
}

pub struct FinalizedNode {
    critical: bool,
    name: String,
    run: Box<
        dyn FnOnce(Arc<str>) -> Pin<Box<dyn Future<Output = anyhow::Result<()>> + Send>> + Send,
    >,
}

impl<N: Node> From<N> for FinalizedNode {
    fn from(value: N) -> Self {
        Self::new(value)
    }
}

impl FinalizedNode {
    pub fn new<N: Node>(node: N) -> Self {
        Self {
            critical: false,
            name: N::DEFAULT_NAME.into(),
            run: Box::new(|name| {
                Box::pin(async move {
                    let context = RuntimeContext { name };
                    node.run(context).await
                })
            }),
        }
    }

    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }

    pub fn set_critical(&mut self, value: bool) {
        self.critical = value;
    }

    pub fn get_critical(&mut self) -> bool {
        self.critical
    }

    async fn run(self, mut abort: broadcast::Receiver<()>) -> Result<(), RunError> {
        let name: Arc<str> = Arc::from(self.name.into_boxed_str());
        let handle = tokio::spawn((self.run)(name.clone()));
        let abort_handle = handle.abort_handle();

        tokio::spawn(async move {
            let _ = abort.recv().await;
            abort_handle.abort();
        });

        let result = match handle.await {
            Ok(x) => x,
            Err(e) => {
                if e.is_cancelled() {
                    return Ok(());
                } else {
                    error!("{} has panicked", name);
                    return Err(RunError {
                        critical: self.critical,
                    });
                }
            }
        };

        result.map_err(|err| {
            error!("{} has faced the following error: {err}", name);
            RunError {
                critical: self.critical,
            }
        })
    }
}



#[derive(Deserialize, Default)]
pub struct RunOptions {
    #[serde(default)]
    pub runtime_name: String,
}

#[macro_export]
macro_rules! setup_logging {
    ($context: ident) => {
        setup_logging!($context $)
    };
    ($context: ident $dol:tt) => {
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
    };
}

const LOGS_DIR: &str = "logs";
static LOGGER_INITED: Once = Once::new();

pub fn init_logger(run_options: &RunOptions) -> anyhow::Result<()> {
    if LOGGER_INITED.is_completed() {
        return Ok(());
    }
    LOGGER_INITED.call_once(|| {});

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
        runtime_name += "_";
    }
    let log_file_name = format!(
        "{}{}.log",
        runtime_name,
        humantime::format_rfc3339(std::time::SystemTime::now())
    );
    let start_time = Instant::now();

    let _ = fern::Dispatch::new()
        .format(move |out, message, record| {
            let secs = start_time.elapsed().as_secs_f32();
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
            fern::Dispatch::new()
                .chain(fern::log_file(PathBuf::from(LOGS_DIR).join(log_file_name))?),
        )
        .chain(
            fern::Dispatch::new()
                .level(log::LevelFilter::Info)
                .chain(std::io::stdout()),
        )
        // Apply globally
        .apply();
    Ok(())
}

#[tokio::main]
pub async fn run_all(
    runnables: impl IntoIterator<Item = FinalizedNode>,
    run_options: RunOptions,
) -> anyhow::Result<()> {
    async_run_all(runnables, run_options).await
}

#[derive(Default)]
struct LastDrop {
    force_exit: bool
}


impl Drop for LastDrop {
    fn drop(&mut self) {
        if self.force_exit {
            std::process::exit(1);
        }
    }
}

pub async fn async_run_all(
    runnables: impl IntoIterator<Item = FinalizedNode>,
    run_options: RunOptions,
) -> anyhow::Result<()> {
    let mut last_drop = LastDrop::default();

    init_logger(&run_options)?;

    let abort_sender = broadcast::Sender::new(1);
    let mut tasks = JoinSet::new();
    for runnable in runnables {
        tasks.spawn(runnable.run(abort_sender.subscribe()));
    }
    if tasks.is_empty() {
        warn!("No nodes to run. Exiting...");
        return Ok(());
    }

    let mut ctrl_c_failed = false;

    loop {
        let ctrl_c_fut: Pin<Box<dyn Future<Output = _>>> = if ctrl_c_failed {
            Box::pin(std::future::pending())
        } else {
            Box::pin(tokio::signal::ctrl_c())
        };
        tokio::select! {
            option = tasks.join_next() => {
                let Some(result) = option else {
                    info!("All Nodes terminated. Exiting...");
                    break;
                };
                if let Err(RunError { critical }) = result.unwrap() {
                    if critical {
                        error!("Critical node has terminated! Exiting...");
                        break;
                    }
                }
            }
            result = ctrl_c_fut => {
                if let Err(e) = result {
                    error!("Ctrl C handler has failed: {e}");
                    ctrl_c_failed = true;
                } else {
                    info!("Ctrl-C received. Exiting...");
                    break;
                }
            }
        }
    }

    drop(abort_sender);
    tokio::select! {
        () = async {
            while let Some(_) = tasks.join_next().await {}
        } => {}
        () = async {
            if tokio::signal::ctrl_c().await.is_err() {
                std::future::pending().await
            }
            warn!("Force exiting...");
            last_drop.force_exit = true;
        } => {}
    }
    
    Ok(())
}
