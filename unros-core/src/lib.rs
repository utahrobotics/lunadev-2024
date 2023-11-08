use std::{
    future::Future,
    path::{Path, PathBuf},
    pin::Pin,
    sync::{
        atomic::{self, AtomicBool},
        Arc,
    },
    time::Instant,
};

pub use anyhow;
use anyhow::Context;
pub use async_trait::async_trait;
pub use log;
use log::{error, info, warn};
use serde::Deserialize;
use static_assertions::assert_impl_all;
pub use tokio;
use tokio::{sync::watch, task::JoinSet};
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

// pub struct RuntimeContext {
//     alive_receiver: broadcast::Receiver<()>,
// }

// impl Clone for RuntimeContext {
//     fn clone(&self) -> Self {
//         Self { alive_receiver: self.alive_receiver.resubscribe() }
//     }
// }

// impl RuntimeContext {
//     pub fn is_alive(&self) -> bool {
//         matches!( self.alive_receiver.try_recv(), Err(TryRecvError::Empty) | Err(TryRecvError::Lagged(_)) | Ok(()))
//     }

//     pub async fn wait_until_death(&mut self) {
//         loop {
//             if let Err(RecvError::Closed) = self.alive_receiver.recv().await {
//                 break
//             }
//         }
//     }
// }

#[async_trait]
pub trait Node: Send + 'static {
    fn set_name(&mut self, name: String);
    fn get_name(&self) -> &str;
    async fn run(self) -> anyhow::Result<()>;
}

struct RunError {
    err: anyhow::Error,
    name: String,
    critical: bool,
}

pub struct Runnable {
    critical: Arc<AtomicBool>,
    run: Box<dyn FnOnce(&mut JoinSet<Result<(), RunError>>)>,
}

impl<N: Node> From<N> for Runnable {
    fn from(value: N) -> Self {
        Self::new(value)
    }
}

impl Runnable {
    pub fn new<N: Node>(node: N) -> Self {
        let critical = Arc::new(AtomicBool::new(false));
        Self {
            critical: critical.clone(),
            run: Box::new(move |tasks| {
                tasks.spawn(async move {
                    let name = node.get_name().to_owned();
                    log::info!("Initializing {}", name);
                    node.run().await.map_err(|err| RunError {
                        err,
                        name,
                        critical: critical.load(atomic::Ordering::SeqCst),
                    })
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

// #[async_trait]
pub trait Signal<T> {
    // async fn emit(&self, msg: T);

    fn connect_to(&mut self, receiver: impl Fn(T) + Send + Sync + 'static);

    fn connect_to_async<F>(&mut self, receiver: impl Fn(T) -> F + Send + Sync + 'static)
    where
        F: Future<Output = ()> + Send + Unpin + 'static;

    fn connect_to_non_blocking(&mut self, receiver: impl Fn(T) + Send + Sync + 'static)
    where
        T: Send + 'static;

    fn connect_to_async_non_blocking<F>(
        &mut self,
        receiver: impl Fn(T) -> F + Send + Sync + 'static,
    ) where
        F: Future<Output = ()> + Send + Unpin + 'static,
        T: Send + 'static;

    // fn map<V: Clone + Send + Sync + 'static>(&mut self, f: impl Fn(T) -> V + Send + Sync) -> MappedSignal<V, T>
    // where T: Clone + Send + Sync + 'static;
}

pub struct OwnedSignal<T: Clone + Send + Sync> {
    async_fns: Vec<Box<dyn Fn(T) -> Box<dyn Future<Output = ()> + Send + Unpin> + Send + Sync>>,
    fns: Vec<Box<dyn Fn(T) + Send + Sync>>,
}

impl<T: Clone + Send + Sync> Default for OwnedSignal<T> {
    fn default() -> Self {
        Self {
            async_fns: Default::default(),
            fns: Default::default(),
        }
    }
}

impl<T: Clone + Send + Sync> OwnedSignal<T> {
    pub async fn emit(&self, msg: T) {
        for async_fn in &self.async_fns {
            async_fn(msg.clone()).await;
        }
        self.fns.iter().for_each(|x| x(msg.clone()));
    }
}

// #[async_trait]
impl<T: Clone + Send + Sync> Signal<T> for OwnedSignal<T> {
    fn connect_to(&mut self, receiver: impl Fn(T) + Send + Sync + 'static) {
        self.fns.push(Box::new(receiver));
    }

    fn connect_to_async<F>(&mut self, receiver: impl Fn(T) -> F + Send + Sync + 'static)
    where
        F: Future<Output = ()> + Send + Unpin + 'static,
    {
        self.async_fns
            .push(Box::new(move |x| Box::new(receiver(x))));
    }

    fn connect_to_non_blocking(&mut self, receiver: impl Fn(T) + Send + Sync + 'static)
    where
        T: Send + 'static,
    {
        let receiver = Arc::new(receiver);
        self.fns.push(Box::new(move |x| {
            let receiver = receiver.clone();
            rayon::spawn(move || receiver(x))
        }));
    }

    fn connect_to_async_non_blocking<F>(
        &mut self,
        receiver: impl Fn(T) -> F + Send + Sync + 'static,
    ) where
        F: Future<Output = ()> + Send + Unpin + 'static,
        T: Send + 'static,
    {
        let receiver = Arc::new(receiver);
        self.fns.push(Box::new(move |x| {
            let receiver = receiver.clone();
            tokio::spawn(async move {
                receiver(x).await;
            });
        }));
    }

    // fn map<V: Clone + Send + Sync + 'static>(&mut self, f: impl Fn(T) -> V + Send + Sync) -> MappedSignal<V, T>
    // where T: Clone + Send + Sync + 'static {
    //     let mapper: Box<dyn Fn(T) -> V + Send + Sync> = Box::new(f);
    //     MappedSignal { signal: SignalVariant::Owned(self), mapper: mapper.into() }
    // }
}

assert_impl_all!(OwnedSignal<()>: Send, Sync);

// enum SignalVariant<'a, T: Clone + Send + Sync + 'static, V: Clone + Send + Sync + 'static=()> {
//     Owned(&'a mut OwnedSignal<T>),
//     Mapped(&'a mut MappedSignal<'a, T, V>)
// }

// impl<'a, T: Clone + Send + Sync + 'static, V: Clone + Send + Sync + 'static> Signal<T> for SignalVariant<'a, T, V> {
//     fn connect_to(&mut self, receiver: impl Fn(T) + Send + Sync + 'static) {
//         match self {
//             Self::Owned(x) => x.connect_to(receiver),
//             Self::Mapped(x) => x.connect_to(receiver),
//         }
//     }

//     fn connect_to_async<F>(&mut self, receiver: impl Fn(T) -> F + Send + Sync + 'static)
//     where
//         F: Future<Output = ()> + Send + Unpin + 'static {
//             match self {
//                 Self::Owned(x) => x.connect_to_async(receiver),
//                 Self::Mapped(x) => x.connect_to_async(receiver),
//             }
//     }

//     fn connect_to_non_blocking(&mut self, receiver: impl Fn(T) + Send + Sync + 'static)
//     where
//         T: Send + 'static {
//             match self {
//                 Self::Owned(x) => x.connect_to_non_blocking(receiver),
//                 Self::Mapped(x) => x.connect_to_non_blocking(receiver),
//             }
//     }

//     fn connect_to_async_non_blocking<F>(
//         &mut self,
//         receiver: impl Fn(T) -> F + Send + Sync + 'static,
//     ) where
//         F: Future<Output = ()> + Send + Unpin + 'static,
//         T: Send + 'static {
//             match self {
//                 Self::Owned(x) => x.connect_to_async_non_blocking(receiver),
//                 Self::Mapped(x) => x.connect_to_async_non_blocking(receiver),
//             }
//     }
// }

pub struct MappedSignal<'a, V: Clone + Send + Sync + 'static, T: Clone + Send + Sync + 'static> {
    signal: &'a mut OwnedSignal<T>,
    mapper: Arc<dyn Fn(T) -> V + Send + Sync>,
}

impl<'a, V: Clone + Send + Sync + 'static, T: Clone + Send + Sync + 'static> Signal<V>
    for MappedSignal<'a, V, T>
{
    fn connect_to(&mut self, receiver: impl Fn(V) + Send + Sync + 'static) {
        let mapper = self.mapper.clone();
        self.signal.connect_to(move |x| receiver(mapper(x)));
    }

    fn connect_to_async<F>(&mut self, receiver: impl Fn(V) -> F + Send + Sync + 'static)
    where
        F: Future<Output = ()> + Send + Unpin + 'static,
    {
        let mapper = self.mapper.clone();
        self.signal.connect_to_async(move |x| receiver(mapper(x)));
    }

    fn connect_to_non_blocking(&mut self, receiver: impl Fn(V) + Send + Sync + 'static)
    where
        V: Send + 'static,
    {
        let mapper = self.mapper.clone();
        self.signal
            .connect_to_non_blocking(move |x| receiver(mapper(x)));
    }

    fn connect_to_async_non_blocking<F>(
        &mut self,
        receiver: impl Fn(V) -> F + Send + Sync + 'static,
    ) where
        F: Future<Output = ()> + Send + Unpin + 'static,
        V: Send + 'static,
    {
        let mapper = self.mapper.clone();
        self.signal
            .connect_to_async_non_blocking(move |x| receiver(mapper(x)));
    }
}

// impl<'a, V: Clone + Send + Sync + 'static, T: Clone + Send + Sync + 'static> MappedSignal<'a, V, T> {
//     pub fn map<X: Clone + Send + Sync + 'static>(&mut self, f: impl Fn(V) -> X + Send + Sync) -> MappedSignal<X, V>
//     where V: Clone + Send + Sync + 'static {
//         let mapper: Box<dyn Fn(V) -> X + Send + Sync> = Box::new(f);
//         MappedSignal { signal: SignalVariant::Mapped(self), mapper: mapper.into() }
//     }
// }

pub struct PublicValue<T: Clone + Send + Sync>(Arc<watch::Sender<T>>);


impl<T: Clone + Send + Sync + Default> Default for PublicValue<T> {
    fn default() -> Self {
        Self::new(Default::default())
    }
}


impl<T: Clone + Send + Sync> PublicValue<T> {
    pub fn new(value: T) -> Self {
        Self(Arc::new(watch::channel(value).0))
    }

    pub fn watch(&self) -> OwnedWatchedPublicValue<T> {
        OwnedWatchedPublicValue {
            _sender: self.0.clone(),
            recv: self.0.subscribe(),
        }
    }

    pub fn replace(&self, value: T) -> T {
        self.0.send_replace(value)
    }

    pub fn get(&self) -> T {
        self.0.borrow().clone()
    }
}

#[async_trait]
pub trait WatchedPublicValue<T: Clone + Send + Sync> {
    async fn wait_for_change(&mut self) -> T;

    fn get(&mut self) -> T;
}

#[derive(Clone)]
pub struct OwnedWatchedPublicValue<T: Clone + Send + Sync> {
    _sender: Arc<watch::Sender<T>>,
    recv: watch::Receiver<T>,
}

#[async_trait]
impl<T: Clone + Send + Sync> WatchedPublicValue<T> for OwnedWatchedPublicValue<T> {
    async fn wait_for_change(&mut self) -> T {
        self.recv.wait_for(|_| true).await.unwrap().clone()
    }

    fn get(&mut self) -> T {
        self.recv.borrow_and_update().clone()
    }
}

#[derive(Deserialize, Default)]
pub struct RunOptions {
    #[serde(default)]
    pub runtime_name: String,
}

const LOGS_DIR: &str = "logs";

#[tokio::main]
pub async fn run_all(
    runnables: impl IntoIterator<Item = Runnable>,
    mut run_options: RunOptions,
) -> anyhow::Result<()> {
    if !AsRef::<Path>::as_ref(LOGS_DIR)
        .try_exists()
        .context("Failed to check if logging directory exists. Do we have permissions?")?
    {
        std::fs::DirBuilder::new()
            .create(LOGS_DIR)
            .context("Failed to create logging directory. Do we have permissions?")?;
    }
    if !run_options.runtime_name.is_empty() {
        run_options.runtime_name += "_";
    }
    let log_file_name = format!(
        "{}{}.log",
        run_options.runtime_name,
        humantime::format_rfc3339(std::time::SystemTime::now())
    );
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
            fern::Dispatch::new()
                .chain(fern::log_file(PathBuf::from(LOGS_DIR).join(log_file_name))?),
        )
        .chain(
            fern::Dispatch::new()
                .level(log::LevelFilter::Info)
                .chain(std::io::stdout()),
        )
        // Apply globally
        .apply()?;

    let mut tasks = JoinSet::new();
    for runnable in runnables {
        (runnable.run)(&mut tasks);
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
                if let Err(RunError { err, name, critical }) = result? {
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

    tasks.abort_all();
    while let Some(_) = tasks.join_next().await {}
    Ok(())
}
