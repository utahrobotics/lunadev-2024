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
//! 5. The Service trait (analagous to ROS actions and services)

#![feature(
    associated_type_defaults,
    once_cell_try,
    iter_collect_into,
    result_flattening,
    div_duration
)]

use std::{
    future::Future,
    marker::PhantomData,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, OnceLock,
    },
    thread::{panicking, JoinHandle},
    time::Instant,
};

pub mod logging;
pub mod pubsub;
pub mod rng;
pub mod service;
pub mod time;

pub use anyhow;
use anyhow::Context;
pub use async_trait::async_trait;
pub use bytes;
use config::Config;
use crossbeam::queue::SegQueue;
pub use log;
use log::{debug, error, info, warn};
use serde::Deserialize;
pub use tokio;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpListener,
    runtime::Runtime,
    sync::mpsc,
    task::JoinSet,
};
pub use tokio_rayon::{self, rayon};

use crate::logging::init_logger;

#[derive(Clone, PartialEq, Eq)]
enum Running {
    No,
    Yes(Arc<str>),
    Ignored,
}

pub struct NodeIntrinsics<N: Node + ?Sized> {
    running: Running,
    _phantom: PhantomData<N>,
}

impl<N: Node + ?Sized> NodeIntrinsics<N> {
    pub fn ignore_drop(&mut self) {
        self.running = Running::Ignored;
    }
}

impl<N: Node + ?Sized> Default for NodeIntrinsics<N> {
    fn default() -> Self {
        Self {
            running: Running::No,
            _phantom: PhantomData,
        }
    }
}

impl<N: Node + ?Sized> Drop for NodeIntrinsics<N> {
    fn drop(&mut self) {
        match &self.running {
            Running::No => warn!("{} was dropped without being ran!", N::DEFAULT_NAME),
            Running::Yes(name) => {
                if panicking() {
                    error!("{name} has panicked!");
                }
            }
            Running::Ignored => {}
        }
    }
}

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

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self>;
}

pub struct Application {
    pending: Vec<
        Box<
            dyn FnOnce(
                    &mut JoinSet<anyhow::Result<()>>,
                    mpsc::UnboundedSender<Box<dyn FnOnce(&mut JoinSet<anyhow::Result<()>>) + Send>>,
                ) + Send,
        >,
    >,
}

impl Application {
    pub fn add_node<N: Node>(&mut self, mut node: N) {
        let name: Arc<str> = Arc::from(N::DEFAULT_NAME.to_string().into_boxed_str());
        let name2 = name.clone();
        self.add_task_inner(
            |x| {
                node.get_intrinsics().running = Running::Yes(name2);
                node.run(x)
            },
            name,
        );
    }

    pub fn add_node_with_name<N: Node>(&mut self, mut node: N, name: impl Into<String>) {
        let name: Arc<str> = Arc::from(name.into().into_boxed_str());
        let name2 = name.clone();
        self.add_task_inner(
            |x| {
                node.get_intrinsics().running = Running::Yes(name2);
                node.run(x)
            },
            name,
        );
    }

    pub fn add_future(
        &mut self,
        fut: impl Future<Output = anyhow::Result<()>> + Send + 'static,
        name: impl Into<String>,
    ) {
        let name: Arc<str> = Arc::from(name.into().into_boxed_str());
        self.pending.push(Box::new(move |join_set, _| {
            join_set.spawn(async move {
                fut.await
                    .with_context(|| format!("{name} has faced an error"))
            });
        }));
    }

    pub fn add_task<F: Future<Output = anyhow::Result<()>> + Send + 'static>(
        &mut self,
        f: impl FnOnce(RuntimeContext) -> F + Send + 'static,
        name: impl Into<String>,
    ) {
        self.add_task_inner(f, Arc::from(name.into().into_boxed_str()));
    }

    fn add_task_inner<F: Future<Output = anyhow::Result<()>> + Send + 'static>(
        &mut self,
        f: impl FnOnce(RuntimeContext) -> F + Send + 'static,
        name: Arc<str>,
    ) {
        self.pending.push(Box::new(move |join_set, node_sender| {
            join_set.spawn(async move {
                f(RuntimeContext {
                    name: name.clone(),
                    node_sender,
                })
                .await
                .with_context(|| format!("{name} has faced an error"))
            });
        }));
    }

    async fn run(self) -> anyhow::Result<()> {
        let mut join_set = JoinSet::new();
        let (node_sender, mut node_recv) = mpsc::unbounded_channel();

        for pending in self.pending {
            pending(&mut join_set, node_sender.clone());
        }

        loop {
            tokio::select! {
                pending = node_recv.recv() => (pending.unwrap())(&mut join_set),
                result = join_set.join_next() => {
                    let Some(result) = result else {
                        info!("All nodes have terminated");
                        break Ok(());
                    };
                    match result {
                        Ok(result) => break result,
                        Err(e) => {
                            error!("Faced the following error while trying to join with node task: {e}");
                        }
                    }
                }
            }
        }
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
    node_sender: mpsc::UnboundedSender<Box<dyn FnOnce(&mut JoinSet<anyhow::Result<()>>) + Send>>,
}

impl RuntimeContext {
    /// Get the name of the node that received this `RuntimeContext`.
    pub fn get_name(&self) -> &Arc<str> {
        &self.name
    }

    /// Spawn a new node into the runtime that the runtime will keep track of.
    pub fn spawn_node<N: Node>(&self, mut node: N) {
        let mut new_context = self.clone();
        let name: Arc<str> = Arc::from(N::DEFAULT_NAME.to_string().into_boxed_str());
        new_context.name = name.clone();
        let _ = self.node_sender.send(Box::new(|join_set| {
            join_set.spawn(async {
                node.get_intrinsics().running = Running::Yes(name.clone());
                node.run(new_context)
                    .await
                    .with_context(move || format!("{name} has faced an error"))
            });
        }));
    }

    /// Spawn a new node into the runtime that the runtime will keep track of.
    pub fn spawn_node_with_name<N: Node>(&self, mut node: N, name: impl Into<String>) {
        let mut new_context = self.clone();
        let name: Arc<str> = Arc::from(name.into().into_boxed_str());
        new_context.name = name.clone();
        let _ = self.node_sender.send(Box::new(|join_set| {
            join_set.spawn(async {
                node.get_intrinsics().running = Running::Yes(name.clone());
                node.run(new_context)
                    .await
                    .with_context(move || format!("{name} has faced an error"))
            });
        }));
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
#[derive(Deserialize, Clone, Copy)]
pub struct RunOptions {
    /// The name of this runtime.
    ///
    /// This changes what the sub-logging directory name is.
    #[serde(default)]
    pub runtime_name: &'static str,

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

/// A safe way to terminate the program with ample logged information.
///
/// Panics do not get logged, and panics intentionally are not able to terminate
/// the runtime. The only way to terminate the program forcefully, other than pressing Ctrl-C
/// twice, is to use `std::process::exit`, which this macro uses, on top of some helpful logging.
///
/// Usage is discouraged if clean exits are required, but this macro is one of the fastest ways
/// to terminate *all* threads of the program to ensure that all computations stop immediately.
#[macro_export]
macro_rules! super_panic {
    () => {{
        $crate::log::error!("super_panic was invoked from {}:{}", file!(), line!());
        std::process::exit(1);
    }};
    ($($arg: tt)*) => {{
        $crate::log::error!("super_panic was invoked from {}:{} due to {}", file!(), line!(), format!($($arg)*));
        std::process::exit(1);
    }}
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
            runtime_name: env!("CARGO_PKG_NAME"),
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

enum EndCondition {
    CtrlC,
    Dropped,
}

pub fn start_unros_runtime<F: Future<Output = anyhow::Result<Application>> + Send + 'static>(
    main: impl FnOnce(Application) -> F,
    run_options: RunOptions,
) -> anyhow::Result<()> {
    init_logger(&run_options)?;
    std::thread::spawn(|| {
        let mut sys = sysinfo::System::new();
        let mut last_cpu_check = Instant::now();
        loop {
            std::thread::sleep(sysinfo::MINIMUM_CPU_UPDATE_INTERVAL);
            sys.refresh_cpu();
            if last_cpu_check.elapsed().as_secs() < 3 {
                continue;
            }
            let cpus = sys.cpus();
            let usage = cpus.iter().map(|cpu| cpu.cpu_usage()).sum::<f32>() / cpus.len() as f32;
            if usage >= 80.0 {
                warn!("CPU Usage at {usage}%");
                last_cpu_check = Instant::now();
            }
        }
    });

    let (end_sender, mut end_recv) = tokio::sync::mpsc::channel(1);
    let end_sender2 = end_sender.clone();

    ctrlc::set_handler(move || {
        let _ = end_sender2.blocking_send(EndCondition::CtrlC);
    })?;

    let runtime = Runtime::new()?;
    let ctrl_c_sender2 = end_sender.clone();
    if run_options.auxilliary_control {
        runtime.spawn(async move {
            let tcp_listener = match TcpListener::bind("0.0.0.0:0").await {
                Ok(x) => x,
                Err(e) => {
                    debug!(target: "auxilliary-control", "Failed to initialize auxilliary control port: {e}");
                    return;
                }
            };

            match tcp_listener.local_addr() {
                Ok(addr) => debug!(target: "auxilliary-control", "Successfully binded to: {addr}"),
                Err(e) => {
                    debug!(target: "auxilliary-control", "Failed to get local address of auxilliary control port: {e}");
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
                let end_sender = ctrl_c_sender2.clone();
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
                                let _ = end_sender.send(EndCondition::CtrlC).await;
                                write_all!(b"Stopping...\n");
                            }
                            _ => write_all!(b"Unrecognized command"),
                        }

                        string_buf.drain(0..newline_idx);
                    }
                });
            }
        });
    }

    runtime.block_on(async {
        let fut = async {
            let mut grp = Application { pending: vec![] };
            grp = tokio::spawn(main(grp)).await??;
            grp.run().await
        };
        info!("Runtime started with pid: {}", std::process::id());
        tokio::select! {
            res = fut => res,
            _ = end_recv.recv() => Ok(()),

        }
    })?;

    info!("Ctrl-C received. Exiting...");

    let dropper = std::thread::spawn(move || {
        drop(runtime);
        while let Some(x) = THREADS.pop() {
            if let Err(e) = x.join() {
                error!("Failed to join thread: {e:?}");
            }
        }
        let _ = end_sender.blocking_send(EndCondition::Dropped);
    });

    match end_recv.blocking_recv().unwrap() {
        EndCondition::CtrlC => warn!("Ctrl-C received. Force exiting..."),
        EndCondition::Dropped => dropper.join().unwrap(),
    }

    Ok(())
}
