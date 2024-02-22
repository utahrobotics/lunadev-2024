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

#![feature(associated_type_defaults, once_cell_try, iter_collect_into, result_flattening)]

use std::{
    future::Future,
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
pub use log;
use log::{debug, error, info, warn};
use serde::Deserialize;
pub use tokio;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt}, net::TcpListener, runtime::Runtime, sync::mpsc, task::JoinSet
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


#[derive(Default)]
pub struct NodeGroup {
    pending: Vec<Box<dyn FnOnce(&mut JoinSet<anyhow::Result<()>>, mpsc::UnboundedSender<Box<dyn FnOnce(&mut JoinSet<anyhow::Result<()>>) + Send>>) + Send>>
}


impl NodeGroup {
    pub fn add_node<N: Node>(&mut self, node: N) {
        let name = Arc::from(N::DEFAULT_NAME.to_string().into_boxed_str());
        self.pending.push(Box::new(|join_set, node_sender| {
            join_set.spawn(node.run(RuntimeContext {
                name,
                node_sender
            }));
        }));
    }

    pub fn add_node_with_name<N: Node>(&mut self, node: N, name: impl Into<String>) {
        let name = Arc::from(name.into().into_boxed_str());
        self.pending.push(Box::new(|join_set, node_sender| {
            join_set.spawn(node.run(RuntimeContext {
                name,
                node_sender
            }));
        }));
    }

    pub async fn run(self) -> anyhow::Result<()> {
        let mut join_set = JoinSet::new();
        let (node_sender, mut node_recv) = mpsc::unbounded_channel();

        for pending in self.pending {
            pending(&mut join_set, node_sender.clone());
        }

        loop {
            tokio::select! {
                pending = node_recv.recv() => (pending.unwrap())(&mut join_set),
                result = join_set.join_next() => {
                    let Some(result) = result else { break Ok(()); };
                    match result {
                        Ok(x) => break x,
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
    pub fn spawn_node<N: Node>(&self, node: N) {
        let mut new_context = self.clone();
        new_context.name = Arc::from(N::DEFAULT_NAME.to_string().into_boxed_str());
        let _ = self.node_sender.send(Box::new(|join_set| {
            join_set.spawn(node.run(new_context));
        }));
    }

    /// Spawn a new node into the runtime that the runtime will keep track of.
    pub fn spawn_node_with_name<N: Node>(&self, node: N, name: impl Into<String>) {
        let mut new_context = self.clone();
        new_context.name = Arc::from(name.into().into_boxed_str());
        let _ = self.node_sender.send(Box::new(|join_set| {
            join_set.spawn(node.run(new_context));
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


pub fn start_unros_runtime<F: Future<Output = anyhow::Result<()>> + Send + 'static>(main: impl FnOnce() -> F, run_options: RunOptions) -> anyhow::Result<()> {
    init_logger(&run_options)?;
    std::thread::spawn(|| {
        let mut sys = sysinfo::System::new();
        let mut last_cpu_check = Instant::now();
        loop {
            std::thread::sleep(sysinfo::MINIMUM_CPU_UPDATE_INTERVAL);
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
    });

    let (ctrl_c_sender, mut ctrl_c_recv) = tokio::sync::mpsc::channel(1);
    let ctrl_c_sender2 = ctrl_c_sender.clone();

    ctrlc::set_handler(move || {
        info!("Ctrl-C received. Exiting...");
        let _ = ctrl_c_sender2.blocking_send(());
    })?;

    let runtime = Runtime::new()?;
    let ctrl_c_sender2 = ctrl_c_sender.clone();
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
                let ctrl_c_sender = ctrl_c_sender2.clone();
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
                                let _ = ctrl_c_sender.send(()).await;
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
        tokio::select! {
            res = tokio::spawn(main()) => res.map_err(Into::into).flatten(),
            _ = ctrl_c_recv.recv() => Ok(()),
            
        }
    })?;

    std::thread::spawn(move || {
        drop(runtime);
        while let Some(x) = THREADS.pop() {
            if let Err(e) = x.join() {
                error!("Failed to join thread: {e:?}");
            }
        }
        let _ = ctrl_c_sender.blocking_send(());
    });

    ctrl_c_recv.blocking_recv().unwrap();
    Ok(())
}
