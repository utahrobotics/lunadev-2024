use std::future::Future;

use crate::{runtime::{RuntimeContext, RuntimeContextExt}, setup_logging};

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

    fn run(self, context: RuntimeContext) -> Self::Result;

    fn spawn(self, context: RuntimeContext)
    where
        Self: Sized + Send + 'static,
    {
        context.clone().spawn_persistent_sync(move || {
            let result = self.run(context.clone());
            result.finish(context);
        });
    }
}

pub trait AsyncNode {
    type Result: NodeResult;

    fn run(self, context: RuntimeContext) -> impl Future<Output = Self::Result> + Send + 'static;

    fn spawn(self, context: RuntimeContext)
    where
        Self: Sized + Send + 'static,
    {
        tokio::spawn(async move {
            let result = self.run(context.clone()).await;
            result.finish(context);
        });
    }
}
