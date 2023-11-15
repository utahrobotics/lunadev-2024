#![feature(associated_type_defaults)]

use std::{
    future::Future,
    ops::{Add, AddAssign},
    path::{Path, PathBuf},
    pin::Pin,
    sync::{Arc, Once},
    time::Instant,
};

pub mod signal;
pub mod task;

pub use anyhow;
use anyhow::Context;
pub use async_trait::async_trait;
pub use bytes;
use chrono::{Datelike, Timelike};
use fxhash::FxHashMap;
pub use log;
use log::{error, info, warn};
use serde::Deserialize;
pub use tokio;
use tokio::{
    sync::{broadcast, mpsc},
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
    node_sender: mpsc::UnboundedSender<FinalizedNode>,
}

impl RuntimeContext {
    pub fn get_name(&self) -> &Arc<str> {
        &self.name
    }

    pub fn spawn_node(&self, node: impl Into<FinalizedNode>) {
        let _ = self.node_sender.send(node.into());
    }
}

struct RunError {
    critical: bool,
}

pub struct FinalizedNode {
    critical: bool,
    runs: Vec<(
        Arc<str>,
        Box<
            dyn FnOnce(
                    mpsc::UnboundedSender<FinalizedNode>,
                ) -> Pin<
                    Box<dyn Future<Output = Result<(), (anyhow::Error, Arc<str>)>> + Send>,
                > + Send,
        >,
    )>,
}

impl<N: Node> From<N> for FinalizedNode {
    fn from(value: N) -> Self {
        Self::new(value, None)
    }
}

impl FinalizedNode {
    pub fn new<N: Node>(node: N, name: Option<String>) -> Self {
        let name = name.unwrap_or_else(|| N::DEFAULT_NAME.into());
        let name: Arc<str> = Arc::from(name.into_boxed_str());

        Self {
            critical: false,
            runs: vec![(
                name.clone(),
                Box::new(move |node_sender| {
                    Box::pin(async move {
                        info!("Running {name}");
                        let context = RuntimeContext {
                            name: name.clone(),
                            node_sender,
                        };
                        node.run(context).await.map_err(|e| (e, name))
                    })
                }),
            )],
        }
    }

    pub fn set_critical(&mut self, value: bool) {
        self.critical = value;
    }

    pub fn get_critical(&mut self) -> bool {
        self.critical
    }

    async fn run(
        self,
        mut abort: broadcast::Receiver<()>,
        node_sender: mpsc::UnboundedSender<FinalizedNode>,
    ) -> Result<(), RunError> {
        let mut tasks = JoinSet::new();
        let mut task_names = FxHashMap::default();

        for (name, run) in self.runs {
            let id = tasks.spawn(run(node_sender.clone())).id();
            task_names.insert(id, name);
        }

        let result = tokio::select! {
            _ = abort.recv() => {
                tasks.abort_all();
                while let Some(result) = tasks.join_next().await {
                    match result {
                        Ok(Ok(())) => continue,
                        Ok(Err((err, name))) => error!("{} has faced the following error: {err}", name),
                        Err(e) => if !e.is_cancelled() {
                            error!("{} has panicked", task_names.get(&e.id()).unwrap());
                        }
                    }
                }
                return Ok(());
            }
            option = tasks.join_next() => {
                match option.unwrap() {
                    Ok(x) => x,
                    Err(e) => {
                        debug_assert!(!e.is_cancelled());
                        error!("{} has panicked", task_names.get(&e.id()).unwrap());
                        return Err(RunError {
                            critical: self.critical,
                        });
                    }
                }
            }
        };

        tasks.abort_all();
        while let Some(result) = tasks.join_next().await {
            match result {
                Ok(Ok(())) => continue,
                Ok(Err((err, name))) => error!("{} has faced the following error: {err}", name),
                Err(e) => error!("{} has panicked", task_names.get(&e.id()).unwrap()),
            }
        }

        result.map_err(|(err, name)| {
            error!("{} has faced the following error: {err}", name);
            RunError {
                critical: self.critical,
            }
        })
    }
}

impl AddAssign for FinalizedNode {
    fn add_assign(&mut self, mut rhs: Self) {
        self.critical = self.critical || rhs.critical;
        self.runs.append(&mut rhs.runs);
    }
}

impl Add for FinalizedNode {
    type Output = Self;

    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;
        self
    }
}

#[derive(Deserialize)]
pub struct RunOptions {
    #[serde(default)]
    pub runtime_name: String,
}

#[macro_export]
macro_rules! default_run_options {
    () => {
        $crate::RunOptions {
            runtime_name: env!("CARGO_PKG_NAME").into(),
        }
    };
}

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
    };
}

pub fn init_logger(run_options: &RunOptions) -> anyhow::Result<()> {
    const LOGS_DIR: &str = "logs";
    static LOGGER_INITED: Once = Once::new();

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

    std::fs::DirBuilder::new()
        .create(PathBuf::from(LOGS_DIR).join(&log_folder_name))
        .context("Failed to create sub-logging directory. Do we have permissions?")?;

    let start_time = Instant::now();

    fern::Dispatch::new()
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
            fern::Dispatch::new().chain(
                fern::log_file(PathBuf::from(LOGS_DIR).join(&log_folder_name).join(".log"))
                    .context("Failed to create log file. Do we have permissions?")?,
            ),
        )
        .chain(
            fern::Dispatch::new()
                .level(log::LevelFilter::Info)
                .chain(std::io::stdout()),
        )
        // Apply globally
        .apply()
        .expect("Logger should have initialized correctly");

    console_subscriber::init();
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
    force_exit: bool,
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
    init_logger(&run_options)?;

    let mut last_drop = LastDrop::default();

    let abort_sender = broadcast::Sender::new(1);
    let (node_sender, mut node_receiver) = mpsc::unbounded_channel();
    let mut tasks = JoinSet::new();
    for runnable in runnables {
        tasks.spawn(runnable.run(abort_sender.subscribe(), node_sender.clone()));
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
            node = async {
                let Some(node) = node_receiver.recv().await else {
                    std::future::pending::<()>().await;
                    unreachable!();
                };
                node
            } => {
                tasks.spawn(node.run(abort_sender.subscribe(), node_sender.clone()));
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
                std::future::pending::<()>().await;
            }
            warn!("Force exiting...");
            last_drop.force_exit = true;
        } => {}
    }

    Ok(())
}
