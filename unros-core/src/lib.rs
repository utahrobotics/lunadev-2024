use std::{
    future::Future,
    path::{Path, PathBuf},
    pin::Pin,
    sync::{
        atomic::{self, AtomicBool},
        Arc, Once,
    },
    time::Instant, num::NonZeroU32, collections::hash_map::Entry, ops::Deref,
};

pub use anyhow;
use anyhow::Context;
pub use async_trait::async_trait;
pub use bytes;
use fxhash::FxHashMap;
pub use log;
use log::{error, info, warn};
use serde::Deserialize;
pub use tokio;
use tokio::{
    sync::{oneshot, watch, broadcast, mpsc},
    task::{JoinError, JoinSet},
};
pub use tokio_rayon::{self, rayon};

// pub trait Variadic {
//     fn contains<T: 'static>() -> bool;
//     fn is_unique<T>() -> bool;
//     fn len() -> usize;
// }

// impl Variadic for () {
//     fn contains<T>() -> bool {
//         false
//     }
//     fn is_unique<T>() -> bool {
//         true
//     }
//     fn len() -> usize {
//         0
//     }
// }

// impl<A: 'static, X: Variadic> Variadic for (A, X) {
//     fn contains<T: 'static>() -> bool {
//         if TypeId::of::<A>() == TypeId::of::<T>() {
//             true
//         } else {
//             X::contains::<T>()
//         }
//     }
//     fn is_unique<T>() -> bool {
//         X::contains::<A>()
//     }
//     fn len() -> usize {
//         1 + X::len()
//     }
// }

#[macro_export]
macro_rules! node_info {
    ($node: expr, $($arg:tt)+) => {
        $crate::log::info!(target: $node.get_name(), $($arg)+)
    };
}

#[macro_export]
macro_rules! node_warn {
    ($node: expr, $($arg:tt)+) => {
        $crate::log::warn!(target: $node.get_name(), $($arg)+)
    };
}

#[macro_export]
macro_rules! node_error {
    ($node: expr, $($arg:tt)+) => {
        $crate::log::error!(target: $node.get_name(), $($arg)+)
    };
}

#[async_trait]
pub trait Node: Send + 'static {
    fn set_name(&mut self, name: String);
    fn get_name(&self) -> &str;
    async fn run(self) -> anyhow::Result<()>;
}

pub struct FnNode<Fut, F>
where
    Fut: Future<Output = anyhow::Result<()>> + Send + 'static,
    F: FnOnce() -> Fut + Send + 'static,
{
    name: String,
    f: F,
}

impl<Fut, F> FnNode<Fut, F>
where
    Fut: Future<Output = anyhow::Result<()>> + Send + 'static,
    F: FnOnce() -> Fut + Send + 'static,
{
    pub fn new(f: F) -> Self {
        Self {
            name: "fn_node".into(),
            f,
        }
    }
}

#[async_trait]
impl<Fut, F> Node for FnNode<Fut, F>
where
    Fut: Future<Output = anyhow::Result<()>> + Send + 'static,
    F: FnOnce() -> Fut + Send + 'static,
{
    fn set_name(&mut self, name: String) {
        self.name = name;
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    async fn run(self) -> anyhow::Result<()> {
        (self.f)().await
    }
}

struct RunError {
    err: anyhow::Error,
    name: String,
    critical: bool,
}

pub struct FinalizedNode {
    critical: Arc<AtomicBool>,
    run: Box<
        dyn FnOnce(
            &mut JoinSet<Result<Result<(), RunError>, (String, JoinError)>>,
            oneshot::Receiver<()>,
        ),
    >,
}

impl<N: Node> From<N> for FinalizedNode {
    fn from(value: N) -> Self {
        Self::new(value)
    }
}

impl FinalizedNode {
    pub fn new<N: Node>(node: N) -> Self {
        let critical = Arc::new(AtomicBool::new(false));
        Self {
            critical: critical.clone(),
            run: Box::new(move |tasks, recv| {
                let name = node.get_name().to_owned();
                let name2 = name.clone();

                tasks.spawn(async move {
                    let handle = tokio::spawn(async move {
                        log::info!("Initializing {}", name);
                        node.run().await.map_err(|err| RunError {
                            err,
                            name,
                            critical: critical.load(atomic::Ordering::SeqCst),
                        })
                    });

                    let abort = handle.abort_handle();

                    tokio::spawn(async move {
                        let _ = recv.await;
                        abort.abort();
                    });

                    handle.await.map_err(|x| (name2, x))
                });
            }),
        }
    }

    pub fn make_critical(&mut self) {
        self.critical.store(true, atomic::Ordering::SeqCst);
    }

    pub fn make_not_critical(&mut self) {
        self.critical.store(false, atomic::Ordering::SeqCst);
    }
}


pub struct UnboundedSubscription<T> {
    recv: Box<dyn FnMut() -> Pin<Box<dyn Future<Output=T>>> + Send + Sync>
}


static_assertions::assert_impl_all!(UnboundedSubscription<()>: Send, Sync);


impl<T: 'static> UnboundedSubscription<T> {
    pub async fn recv(&mut self) -> T {
        (self.recv)().await
    }

    /// Changes the generic type of the signal that this subscription is for.
    /// 
    /// This is done by applying a mapping function after a message is received.
    /// This mapping function is ran in an asynchronous context, so it should be
    /// non-blocking. Do note that the mapping function itself is not asynchronous
    /// and is multi-thread safe.
    /// 
    /// There is also a non-zero cost to mapping on top of the mapping functions,
    /// so avoid having deeply mapped subscriptions. This is due to the lack of
    /// `AsyncFn` traits and/or lending functions.
    pub fn map<V>(self, mapper: impl FnMut(T) -> V + 'static + Send + Sync) -> UnboundedSubscription<V> {
        let mapper = Arc::new(std::sync::Mutex::new(mapper));
        let recv = Arc::new(std::sync::Mutex::new(self.recv));
        
        UnboundedSubscription {
            recv: Box::new(move || {
                let mapper = mapper.clone();
                let recv = recv.clone();

                Box::pin(async move {
                    let value = (recv.lock().unwrap())().await;
                    (mapper.lock().unwrap())(value)
                })
            })
        }
    }
}


pub struct BoundedSubscription<T, S=T> {
    recv: broadcast::Receiver<S>,
    mapper: Box<dyn FnMut(S) -> T>
}


impl<T: 'static, S: Clone + 'static> BoundedSubscription<T, S> {
    pub async fn recv(&mut self) -> Result<T, u64> {
        let value = match self.recv.recv().await {
            Ok(x) => x,
            Err(broadcast::error::RecvError::Closed) => {
                std::future::pending::<()>().await;
                unreachable!();
            }
            Err(broadcast::error::RecvError::Lagged(n)) => return Err(n)
        };
        Ok((self.mapper)(value))
    }

    pub fn map<V>(mut self, mut mapper: impl FnMut(T) -> V + 'static) -> BoundedSubscription<V, S> {
        BoundedSubscription {
            recv: self.recv,
            mapper: Box::new(move |x| mapper((self.mapper)(x)))
        }
    }
}


pub struct WatchedSubscription<T, S=T> {
    recv: watch::Receiver<Option<S>>,
    mapper: Box<dyn FnMut(S) -> T>
}


impl<T: 'static, S: Clone + 'static> WatchedSubscription<T, S> {
    pub async fn get(&mut self) -> T {
        let value = loop {
            if let Some(value) = self.recv.borrow_and_update().deref() {
                break value.clone();
            }
            if self.recv.changed().await.is_err() {
                std::future::pending::<()>().await;
            }
        };
        (self.mapper)(value)
    }

    pub fn map<V>(mut self, mut mapper: impl FnMut(T) -> V + 'static) -> WatchedSubscription<V, S> {
        WatchedSubscription {
            recv: self.recv,
            mapper: Box::new(move |x| mapper((self.mapper)(x)))
        }
    }
}


/// An essential component that promotes separation of concerns, and is 
/// an intrinsic element of the ROS framework.
/// 
/// Signals provide a simple way to send a message to receivers, much
/// like Rust's channels. However, signals provide 3 different forms
/// of subscriptions:
/// 
/// 1. **Unbounded**<br>
///    The subscription will contain all sent messages forever and ever
///    if it is never read from. If the receiving code can handle the same
///    throughput that this signal produces, but the buffer size needed is
///    unknown or highly variable, use this subscription.
/// 
/// 2. **Bounded**<br>
///    The subscription will hold a limited number of messages before
///    ignoring future messages until the current ones are read. If the
///    receiving code may not be able to handle the same throughput that
///    this signal produces, and can tolerate lost messages, use this
///    subscription.
/// 
/// 3. **Watched**<br>
///    The subscription will only keep track of the latest message. If you
///    only need the latest value, use this subscription.
/// 
/// Signals make numerous clones of the values it will send, so you should
/// use a type `T` that is cheap to clone with this signal. A good default is
/// `Arc`. Since Nodes will often be used from different threads, the type `T`
/// should also be `Send + Sync`, but this is not a requirement.
pub struct Signal<T> {
    unbounded_senders: Vec<mpsc::UnboundedSender<T>>,
    bounded_senders: FxHashMap<NonZeroU32, broadcast::Sender<T>>,
    watch_sender: watch::Sender<Option<T>>
}


impl<T: Clone + Send + 'static> Signal<T> {
    pub fn subscribe_unbounded(&mut self) -> UnboundedSubscription<T> {
        let (sender, recv) = mpsc::unbounded_channel();
        let recv = Arc::new(tokio::sync::Mutex::new(recv));
        self.unbounded_senders.push(sender);

        UnboundedSubscription {
            recv: Box::new(move || {
                let recv = recv.clone();

                Box::pin(async move {
                    let Some(value) = recv.lock().await.recv().await else {
                        std::future::pending::<()>().await;
                        unreachable!();
                    };
                    value
                })
            })
        }
    }

    pub fn subscribe_bounded(&mut self, bound: NonZeroU32) -> BoundedSubscription<T> {
        let recv = match self.bounded_senders.entry(bound) {
            Entry::Occupied(x) => x.get().subscribe(),
            Entry::Vacant(x) => {
                let (sender, recv) = broadcast::channel(bound.get() as usize);
                x.insert(sender);
                recv
            }
        };
        BoundedSubscription { recv, mapper: Box::new(|x| x) }
    }

    pub fn watch(&self) -> WatchedSubscription<T> {
        WatchedSubscription { recv: self.watch_sender.subscribe(), mapper: Box::new(|x| x) }
    }

    /// Sets a value into this signal.
    /// 
    /// Unbounded and Bounded Subscriptions will receive this value, and
    /// Watched Subscriptions will replace their current values with this.
    /// 
    /// This method takes an immutable reference as a convenience, but this
    /// can be abused by nodes that do not own this signal. As a user of this
    /// signal, ie. you are accessing this signal just to subscribe to it,
    /// do not call this method ever. This will lead to spaghetti code. Only
    /// the node that owns this signal should call this method.
    pub fn set(&self, value: T) {
        for sender in &self.unbounded_senders {
            let _ = sender.send(value.clone());
        }
        for sender in self.bounded_senders.values() {
            let _ = sender.send(value.clone());
        }
        self.watch_sender.send_replace(Some(value));
    }
}


pub trait SignalProvider<T: Clone> {
    fn subscribe_unbounded(&mut self) -> UnboundedSubscription<T>;
    fn subscribe_bounded(&mut self, bound: NonZeroU32) -> BoundedSubscription<T>;
    fn watch(&self) -> WatchedSubscription<T>;
}


#[derive(Deserialize, Default)]
pub struct RunOptions {
    #[serde(default)]
    pub runtime_name: String,
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

pub async fn async_run_all(
    runnables: impl IntoIterator<Item = FinalizedNode>,
    run_options: RunOptions,
) -> anyhow::Result<()> {
    init_logger(&run_options)?;

    let mut senders = Vec::new();
    let mut tasks = JoinSet::new();
    for runnable in runnables {
        let (sender, recv) = oneshot::channel();
        senders.push(sender);
        (runnable.run)(&mut tasks, recv);
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
                let result = match result.unwrap() {
                    Ok(x) => x,
                    Err((name, _)) => {
                        error!("{name} has panicked");
                        continue;
                    }
                };
                if let Err(RunError { err, name, critical }) = result {
                    let mut err_string = format!("{err:?}");
                    err_string = err_string.replace('\n', "\n\t");
                    error!("{name} has faced the following error:\n\t{err_string}");
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

    drop(senders);
    while let Some(result) = tasks.join_next().await {
        let result = match result.unwrap() {
            Ok(x) => x,
            Err((name, e)) => {
                if !e.is_cancelled() {
                    error!("{name} has panicked");
                }
                continue;
            }
        };
        if let Err(RunError {
            err,
            name,
            critical,
        }) = result
        {
            let mut err_string = format!("{err:?}");
            err_string = err_string.replace('\n', "\n\t");
            error!("{name} has faced the following error:\n\t{err_string}");
            if critical {
                error!("Critical node has terminated! Exiting...");
                break;
            }
        }
    }
    Ok(())
}
