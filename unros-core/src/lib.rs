use std::{
    future::Future,
    path::{Path, PathBuf},
    pin::Pin,
    sync::{
        Arc, Once,
    },
    time::Instant, num::NonZeroU32, collections::hash_map::Entry, ops::{Deref, Add},
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
    sync::{watch, broadcast, mpsc},
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
        Self {
            f,
        }
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
    name: Arc<str>
}


impl RuntimeContext {
    pub fn get_name(&self) -> &Arc<str> {
        &self.name
    }
}


struct RunError {
    critical: bool
}

pub struct FinalizedNode {
    critical: bool,
    name: String,
    run: Box<dyn FnOnce(Arc<str>) -> Pin<Box<dyn Future<Output=anyhow::Result<()>> + Send>> + Send>,
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
            run: Box::new(|name| Box::pin(async move {
                let context = RuntimeContext {
                    name
                };
                node.run(context).await
            }))
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
            Err(e) => if e.is_cancelled() {
                return Ok(())
            } else {
                error!("{} has panicked", name);
                return Err(RunError { critical: self.critical })
            }
        };

        result.map_err(|err| {
            error!("{} has faced the following error: {err}", name);
            RunError { critical: self.critical }
        })
    }
}


pub struct UnboundedSubscription<T> {
    recv: Box<dyn FnMut() -> Pin<Box<dyn Future<Output=T>>> + Send + Sync>
}


static_assertions::assert_impl_all!(UnboundedSubscription<()>: Send, Sync);


impl<T: 'static> UnboundedSubscription<T> {
    pub fn none() -> Self {
        Self {
            recv: Box::new(|| Box::pin(std::future::pending()))
        }
    }

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


impl<T: 'static> Add for UnboundedSubscription<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let recv1 = Arc::new(tokio::sync::Mutex::new(self.recv));
        let recv2 = Arc::new(tokio::sync::Mutex::new(rhs.recv));

        Self {
            recv: Box::new(move || {
                let recv1 = recv1.clone();
                let recv2 = recv2.clone();

                Box::pin(async move {
                    tokio::select! {
                        value = async { (recv1.lock().await)().await } => { value }
                        value = async { (recv2.lock().await)().await } => { value }
                    }
                })
            })
        }
    }
}


pub struct BoundedSubscription<T> {
    recv: Box<dyn FnMut() -> Pin<Box<dyn Future<Output=Result<T, u64>>>> + Send + Sync>
}


static_assertions::assert_impl_all!(BoundedSubscription<()>: Send, Sync);


impl<T: 'static> BoundedSubscription<T> {
    pub fn none() -> Self {
        Self {
            recv: Box::new(|| Box::pin(std::future::pending()))
        }
    }

    pub async fn recv(&mut self) -> Result<T, u64> {
        (self.recv)().await
    }

    pub fn map<V>(self, mapper: impl FnMut(T) -> V + 'static + Send) -> BoundedSubscription<V> {
        let mapper = Arc::new(std::sync::Mutex::new(mapper));
        let recv = Arc::new(std::sync::Mutex::new(self.recv));

        BoundedSubscription {
            recv: Box::new(move || {
                let mapper = mapper.clone();
                let recv = recv.clone();

                Box::pin(async move {
                    let value = (recv.lock().unwrap())().await;
                    value.map(|x| (mapper.lock().unwrap())(x))
                })
            })
        }
    }
}


impl<T: 'static> Add for BoundedSubscription<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let recv1 = Arc::new(tokio::sync::Mutex::new(self.recv));
        let recv2 = Arc::new(tokio::sync::Mutex::new(rhs.recv));

        Self {
            recv: Box::new(move || {
                let recv1 = recv1.clone();
                let recv2 = recv2.clone();

                Box::pin(async move {
                    tokio::select! {
                        value = async { (recv1.lock().await)().await } => { value }
                        value = async { (recv2.lock().await)().await } => { value }
                    }
                })
            })
        }
    }
}


pub struct WatchedSubscription<T> {
    recv: Box<dyn FnMut() -> Pin<Box<dyn Future<Output=T>>> + Send + Sync>
}


static_assertions::assert_impl_all!(WatchedSubscription<()>: Send, Sync);


impl<T: 'static> WatchedSubscription<T> {
    pub fn none() -> Self {
        Self {
            recv: Box::new(|| Box::pin(std::future::pending()))
        }
    }

    pub async fn get(&mut self) -> T {
        (self.recv)().await
    }

    pub fn map<V>(self, mapper: impl FnMut(T) -> V + 'static + Send) -> WatchedSubscription<V> {
        let mapper = Arc::new(std::sync::Mutex::new(mapper));
        let recv = Arc::new(std::sync::Mutex::new(self.recv));
        
        WatchedSubscription {
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


impl<T: 'static> Add for WatchedSubscription<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let recv1 = Arc::new(tokio::sync::Mutex::new(self.recv));
        let recv2 = Arc::new(tokio::sync::Mutex::new(rhs.recv));

        Self {
            recv: Box::new(move || {
                let recv1 = recv1.clone();
                let recv2 = recv2.clone();

                Box::pin(async move {
                    tokio::select! {
                        value = async { (recv1.lock().await)().await } => { value }
                        value = async { (recv2.lock().await)().await } => { value }
                    }
                })
            })
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


impl<T> Default for Signal<T> {
    fn default() -> Self {
        Self { unbounded_senders: Default::default(), bounded_senders: Default::default(), watch_sender: watch::channel(None).0 }
    }
}


impl<T: Clone> Signal<T> {
    pub fn get_ref(&mut self) -> SignalRef<T> {
        SignalRef(self)
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


pub struct SignalRef<'a, T>(&'a mut Signal<T>);


impl<'a, T: Clone + Send + Sync + 'static> SignalRef<'a, T> {
    pub fn subscribe_unbounded(&mut self) -> UnboundedSubscription<T> {
        let (sender, recv) = mpsc::unbounded_channel();
        let recv = Arc::new(tokio::sync::Mutex::new(recv));
        self.0.unbounded_senders.push(sender);

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
        let recv = match self.0.bounded_senders.entry(bound) {
            Entry::Occupied(x) => x.get().subscribe(),
            Entry::Vacant(x) => {
                let (sender, recv) = broadcast::channel(bound.get() as usize);
                x.insert(sender);
                recv
            }
        };
        let recv = Arc::new(tokio::sync::Mutex::new(recv));

        BoundedSubscription {
            recv: Box::new(move || {
                let recv = recv.clone();

                Box::pin(async move {
                    let value = match recv.lock().await.recv().await {
                        Ok(x) => x,
                        Err(broadcast::error::RecvError::Closed) => {
                            std::future::pending::<()>().await;
                            unreachable!();
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => return Err(n)
                    };
                    Ok(value)
                })
            })
        }
    }

    pub fn watch(&self) -> WatchedSubscription<T> {
        let recv = Arc::new(tokio::sync::Mutex::new(self.0.watch_sender.subscribe()));

        WatchedSubscription {
            recv: Box::new(move || {
                let recv = recv.clone();

                Box::pin(async move {
                    let mut recv =  recv.lock().await;
                    loop {
                        if let Some(value) = recv.borrow_and_update().deref() {
                            break value.clone();
                        }
                        if recv.changed().await.is_err() {
                            std::future::pending::<()>().await;
                        }
                    }
                })
            })
        }
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

pub async fn async_run_all(
    runnables: impl IntoIterator<Item = FinalizedNode>,
    run_options: RunOptions,
) -> anyhow::Result<()> {
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
    while let Some(_) = tasks.join_next().await {}
    Ok(())
}
