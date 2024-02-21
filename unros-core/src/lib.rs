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
//! 4. Signals, with 3 subscription variants (analagous to ROS publisher and subscribers)
//! 5. The Task trait (analagous to ROS actions)

#![feature(associated_type_defaults, once_cell_try, iter_collect_into)]

use std::{
    future::Future,
    ops::{Add, AddAssign},
    pin::Pin,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, OnceLock,
    },
    thread::JoinHandle,
    time::Instant,
};

pub mod logging;
pub mod pubsub;
pub mod rng;
pub mod task;

pub use anyhow;
pub use async_trait::async_trait;
pub use bytes;
use config::Config;
use crossbeam::queue::SegQueue;
use fxhash::FxHashMap;
pub use log;
use log::{debug, error, info, warn};
use serde::Deserialize;
pub use tokio;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpListener,
    sync::{broadcast, mpsc},
    task::JoinSet,
};
pub use tokio_rayon::{self, rayon};

use crate::logging::init_logger;

/// A Node just represents a long running task.
///
/// Nodes are only required to run once, and may terminate at any point in time.
/// Nodes in ROS also serve as forms of isolation. If a thread faces an exception
/// while running code in ROS, other code in the same thread will stop executing.
/// Developers would then segment their code into nodes such that each node could
/// operate in a different thread.
///
/// Rust allows developers to make stronger guarantees regarding when and where
/// their code will panic. As such, Nodes are expected to never panic. Instead,
/// they must return an `anyhow::Error` when facing an unrecoverable error, or
/// log using the `error!` macro if normal functionality can be continued.
///
/// In the event that a Node panics, the thread running the node will not be taken
/// down, nor will any other node. An error message including the name of the node
/// that panicked will be logged. Even so, panics should be avoided.
#[async_trait]
pub trait Node: Send + 'static {
    const DEFAULT_NAME: &'static str;

    /// The entry point of the node.
    ///
    /// Nodes are always expected to be asynchronous, as asynchronous code is much
    /// easier to manage.
    ///
    /// If a node needs to run blocking code, it is recommended to use `tokio_rayon::spawn`
    /// instead of `rayon::spawn` or `std::thread::spawn`, as `tokio_rayon` allows you
    /// to await the spawned thread in a non-blocking way. If you spawn a thread and do not
    /// wait on it, you may accidentally exit this method while threads are still running.
    /// While this is not unsafe or incorrect, it can lead to misleading logs. Unros automatically
    /// logs all nodes whose `run` methods have returned as terminated, even if they have spawned
    /// threads that are still running.
    ///
    /// Do keep in mind that `tokio_rayon` threads do not terminate if their handles are dropped,
    /// which relates back to the issue previously mentioned.
    async fn run(self, context: RuntimeContext) -> anyhow::Result<()>;
}

/// A Node that is just an async function that runs once.
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

/// A reference to the runtime that is currently running.
///
/// The typical way of receiving this is through the `run` method
/// of `Node`. As such, the runtime in question is the runtime that
/// is currently running the node.
#[derive(Clone)]
pub struct RuntimeContext {
    name: Arc<str>,
    node_sender: mpsc::UnboundedSender<FinalizedNode>,
}

impl RuntimeContext {
    /// Get the name of the node that received this `RuntimeContext`.
    pub fn get_name(&self) -> &Arc<str> {
        &self.name
    }

    /// Spawn a new node into the runtime that the runtime will keep track of.
    pub fn spawn_node(&self, node: impl Into<FinalizedNode>) {
        let _ = self.node_sender.send(node.into());
    }
}

struct RunError {
    critical: bool,
}

/// A node that has been boxed up and is ready to run.
///
/// Finalized nodes may be added together such that they run
/// as a group. When one node in a group terminates, all other
/// nodes terminate. If one of these other nodes are critical,
/// the whole runtime will terminate even if the original node
/// that terminated was not critical.
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
    /// Box up the given node.
    ///
    /// The given name will be used as the name of the node.
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

    /// Sets the `critical` flag of the node.
    ///
    /// Critical nodes terminate the whole runtime when they terminate.
    pub fn set_critical(&mut self, value: bool) {
        self.critical = value;
    }

    /// Gets the `critical` flag of the node.
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

#[derive(Clone)]
pub struct DropCheck {
    dropped: Arc<AtomicBool>,
    update_on_drop: bool,
}

impl Default for DropCheck {
    fn default() -> Self {
        Self {
            dropped: Default::default(),
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
    pub fn has_dropped(&self) -> bool {
        self.dropped.load(Ordering::SeqCst)
    }

    pub fn reset(&self) {
        self.dropped.store(true, Ordering::SeqCst);
    }

    pub fn update_on_drop(&mut self) {
        self.update_on_drop = true;
    }

    pub fn dont_update_on_drop(&mut self) {
        self.update_on_drop = true;
    }
}

/// Configurations for the runtime
#[derive(Deserialize)]
pub struct RunOptions {
    /// The name of this runtime.
    ///
    /// This changes what the sub-logging directory name is.
    #[serde(default)]
    pub runtime_name: String,

    /// Whether or not auxilliary control should be enabled.
    ///
    /// Auxilliary control is a way for the current runtime
    /// to be controlled externally, such as from another program.
    /// This is typically used to terminate the runtime remotely
    /// when the interface to the program running the runtime has been
    /// lost.
    #[serde(default = "default_auxilliary_control")]
    pub auxilliary_control: bool,

    /// Enablinng console subscriber allows you to view the state of
    /// each `tokio` task as the program is running. However, under
    /// certain circumstances (such as running in `examples`) may lead
    /// to an irrecoverable panic from `console-subscriber`. To avoid this,
    /// simply set this field to `false`.
    #[serde(default = "default_enable_console_subscriber")]
    pub enable_console_subscriber: bool,
}

fn default_auxilliary_control() -> bool {
    true
}

fn default_enable_console_subscriber() -> bool {
    true
}

/// Creates a default `RunOptions`.
///
/// This macro was created instead of implementing `Default`
/// so that the crate calling this macro can have its name
/// used as the `runtime_name`.
///
/// By default, `auxilliary_control` is `true`.
#[macro_export]
macro_rules! default_run_options {
    () => {
        $crate::RunOptions {
            runtime_name: env!("CARGO_PKG_NAME").into(),
            auxilliary_control: true,
            enable_console_subscriber: true,
        }
    };
}

static THREADS: SegQueue<JoinHandle<()>> = SegQueue::new();

/// Spawns a thread that is guaranteed to run the given closure to completion.
///
/// There is a caveat, and that is if the program is forcefully exited, this
/// function cannot do anything.
///
/// Functionally, this just spawns a thread that will always be joined before the
/// main thread exits, *assuming* that you call `async_run_all` or `run_all`.
pub fn spawn_persistent_thread<F>(f: F)
where
    F: FnOnce(),
    F: Send + 'static,
{
    THREADS.push(std::thread::spawn(f));
}

/// The entry point of the runtime itself.
///
/// This function automatically starts up a `tokio` runtime.
/// As such, this method should not be called within a `tokio`
/// runtime, or any `async` code in general.
///
/// Refer to `async_run_all` for more info.
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

/// The entry point of the runtime itself.
///
/// This function runs all of the provided `FinalizedNode`s
/// in parallel, according to the given `RunOptions`. A
/// logging implementation will be initialized if not present.
///
/// This function only returns `Ok(())` if `Ctrl-C` was received,
/// or an auxilliary stop signal was received. In either case,
/// this method always attempts to exit gracefully. It will signal
/// to each `Node` to stop running, but the actual outcome of this
/// varies based on the actual implementations of these `Node`s. Since
/// this method waits on all `Node`s to exit, if one `Node` refuses to
/// terminate, this method will never return gracefully.
///
/// Receiving a second stop signal from `Ctrl-C` or auxilliary will force
/// exit the program. All threads other than the main thread will not exit
/// gracefully (all objects in their stacks will not be dropped), but objects
/// in the main thread will still be able to be dropped safely. This was chosen
/// to be the best case scenario.
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

    let (auxilliary_interrupt_sender, mut auxilliary_interrupt_sender_recv) = mpsc::channel(1);

    if run_options.auxilliary_control {
        tokio::spawn(async move {
            let tcp_listener = match TcpListener::bind("0.0.0.0:0").await {
                Ok(x) => x,
                Err(e) => {
                    debug!(target: "auxilliary-control", "Failed to initialize auxilliary control port: {e}");
                    std::mem::forget(auxilliary_interrupt_sender);
                    return;
                }
            };

            match tcp_listener.local_addr() {
                Ok(addr) => debug!(target: "auxilliary-control", "Successfully binded to: {addr}"),
                Err(e) => {
                    debug!(target: "auxilliary-control", "Failed to get local address of auxilliary control port: {e}");
                    std::mem::forget(auxilliary_interrupt_sender);
                    return;
                }
            }

            loop {
                let mut stream = match tcp_listener.accept().await {
                    Ok(x) => x.0,
                    Err(e) => {
                        debug!(target: "auxilliary-control", "Failed to accept auxilliary control stream: {e}");
                        continue;
                    }
                };
                let auxilliary_interrupt_sender = auxilliary_interrupt_sender.clone();
                tokio::spawn(async move {
                    let mut string_buf = Vec::with_capacity(1024);
                    let mut buf = [0u8; 1024];
                    loop {
                        macro_rules! write_all {
                            ($data: expr) => {
                                if let Err(e) = stream.write_all($data).await {
                                    debug!(target: "auxilliary-control", "Failed to write to auxilliary control stream: {e}");
                                    break;
                                }
                            }
                        }
                        match stream.read(&mut buf).await {
                            Ok(n) => {
                                string_buf.extend_from_slice(buf.split_at(n).0);
                            }
                            Err(e) => {
                                debug!(target: "auxilliary-control", "Failed to read from auxilliary control stream: {e}");
                                break;
                            }
                        }

                        let Ok(string) = std::str::from_utf8(&buf) else {
                            continue;
                        };
                        let Some(newline_idx) = string.find('\n') else {
                            continue;
                        };

                        let command = string.split_at(newline_idx).0;

                        match command {
                            "stop" => {
                                let _ = auxilliary_interrupt_sender.send(()).await;
                                write_all!(b"Stopping...\n");
                            }
                            _ => write_all!(b"Unrecognized command"),
                        }

                        string_buf.drain(0..newline_idx);
                    }
                });
            }
        });
    } else {
        std::mem::forget(auxilliary_interrupt_sender);
    }

    let mut ctrl_c_failed = false;
    let mut sys = sysinfo::System::new();
    let mut last_cpu_check = Instant::now();

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
            _ = tokio::time::sleep(sysinfo::MINIMUM_CPU_UPDATE_INTERVAL) => {
                sys.refresh_cpu(); // Refreshing CPU information.
                if last_cpu_check.elapsed().as_secs() < 3 {
                    continue;
                }
                let cpus = sys.cpus();
                let usage = cpus.into_iter().map(|cpu| cpu.cpu_usage()).sum::<f32>() / cpus.len() as f32;
                if usage >= 80.0 {
                    warn!("CPU Usage at {usage}%");
                    last_cpu_check = Instant::now();
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
            _ = auxilliary_interrupt_sender_recv.recv() => {
                info!("Auxilliary stop received. Exiting...");
                break;
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
            tokio::join!(
                async { if let Err(e) = tokio::task::spawn_blocking(|| {
                    while let Some(x) = THREADS.pop() {
                        if let Err(e) = x.join() {
                            error!("Failed to join thread: {e:?}");
                        }
                    }
                }).await { error!("Failed to wait on persistent threads: {e}"); }},
                async {while let Some(_) = tasks.join_next().await {}}
            );
        } => {}
        () = async {
            if tokio::signal::ctrl_c().await.is_err() {
                std::future::pending::<()>().await;
            }
            warn!("Force exiting...");
            last_drop.force_exit = true;
        } => {}
        _ = auxilliary_interrupt_sender_recv.recv() => {
            warn!("Force exiting...");
            last_drop.force_exit = true;
        }
    }

    Ok(())
}

static CONFIG: OnceLock<Config> = OnceLock::new();

pub fn get_env<'de, T: Deserialize<'de>>() -> anyhow::Result<T> {
    CONFIG
        .get_or_try_init(|| {
            Config::builder()
                // Add in `./Settings.toml`
                .add_source(config::File::with_name(".env"))
                .add_source(config::Environment::with_prefix(""))
                .build()
        })?
        .clone()
        .try_deserialize()
        .map_err(Into::into)
}
