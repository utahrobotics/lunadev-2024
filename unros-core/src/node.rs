use std::future::Future;

use crate::{runtime::{AbortHandle, RuntimeContext, RuntimeContextExt}, setup_logging};

pub trait NodeResult: Send + 'static {
    fn finish(self, context: RuntimeContext);
}

impl NodeResult for () {
    fn finish(self, context: RuntimeContext) {
        setup_logging!(context);
        info!("Node finished successfully.");
    }
}

impl<T: NodeResult, E: std::fmt::Debug + Send + 'static> NodeResult for Result<T, E> {
    fn finish(self, context: RuntimeContext) {
        setup_logging!(context);
        match self {
            Ok(result) => result.finish(context),
            Err(err) => error!("Node finished with error: {:?}", err),
        }
    }
}

pub trait SyncNode {
    type Result: NodeResult;
    /// Whether the node should be run in a persistent thread.
    /// 
    /// A persistent thread will be awaited on when the runtime is exiting.
    /// If the thread does not have a mechanism for exiting, the runtime
    /// will wait indefinitely.
    /// 
    /// A non-persistent thread will be detached when the runtime exits. This
    /// means that it will continue to run. If the main thread exits, the resources
    /// in the thread are not guaranteed to be dropped.
    const PERSISTENT: bool = false;

    /// Runs the node in the current thread, blocking until the node is finished.
    fn run(self, context: RuntimeContext) -> Self::Result;

    /// Spawns the node in a new thread.
    /// 
    /// Refer to the documentation of `PERSISTENT` for more information.
    /// 
    /// There is no generic way to implement cancellations as threads cannot
    /// be externally stopped. If you need to cancel a node, you should implement
    /// that functionality on your own.
    fn spawn(self, context: RuntimeContext)
    where
        Self: Sized + Send + 'static,
    {
        if Self::PERSISTENT {
            context.clone().spawn_persistent_sync(move || {
                let result = self.run(context.clone());
                result.finish(context);
            });
        } else {
            let handle = context.inner.runtime_handle.clone();
            std::thread::spawn(move || {
                let _guard = handle.enter();
                let result = self.run(context.clone());
                result.finish(context);
            });
        }
    }
}

pub trait AsyncNode {
    type Result: NodeResult;
    /// Whether the node should be run in a persistent task.
    /// 
    /// A persistent task will be awaited on when the runtime is exiting.
    /// If the task does not have a mechanism for exiting, the runtime
    /// will wait indefinitely.
    /// 
    /// A non-persistent task will be aborted when the runtime exits.
    const PERSISTENT: bool = false;

    fn run(self, context: RuntimeContext) -> impl Future<Output = Self::Result> + Send + 'static;

    /// Spawns the node in a new task.
    /// 
    /// Refer to the documentation of `PERSISTENT` for more information.
    /// 
    /// The returned handle can be used to abort the task. Dropping the handle
    /// will not abort the task.
    fn spawn(self, context: RuntimeContext) -> AbortHandle
    where
        Self: Sized + Send + 'static,
    {
        if Self::PERSISTENT {
            context.clone().spawn_persistent_async(async move {
                let result = self.run(context.clone()).await;
                result.finish(context);
            })
        } else {
            let spawn_context = context.get_name().into();
            let abort = tokio::spawn(async move {
                let result = self.run(context.clone()).await;
                result.finish(context);
            }).abort_handle();
            AbortHandle {
                inner: abort,
                spawn_context,
            }
        }
    }
}
