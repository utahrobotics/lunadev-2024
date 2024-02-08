//! A task is a long running action whose progress or result may be of importance.
//!
//! In robotics, a task could be as simple as moving an arm to some angle. If an
//! autonomous process wants to move the arm to some angle, it schedules the
//! appropriate task, and watches its progress (the angle of the arm). If there
//! was an error (such as the arm getting stuck), the task may returrn an error
//! to the autonomous process, allowing it to intelligently handle it.

use std::{borrow::Cow, fmt::Display};

use async_trait::async_trait;
use log::error;
use tokio::sync::{mpsc, oneshot};

/// A task is a long running action which can return a value and/or provide
/// data about itself to the code that scheduled it.
#[async_trait]
pub trait Task: Sync {
    /// The output of a task.
    ///
    /// Any type can be used as long as it is safe to send across threads.
    type Output: Send;
    /// The error that is returned if a task could not be scheduled at this moment.
    type ScheduleError: std::error::Error;
    /// The data needed to schedule a task.
    ///
    /// If the task moves an arm, the data could be the angle that the arm
    /// should move to.
    type ScheduleData;
    /// Arbitrary data that the scheduling code could use while the task is running.
    ///
    /// By default, this is void. However, this could be some shared float representing
    /// the current progress of the task, or some other valuable metric that ought to
    /// be shared.
    type TaskData = ();

    /// Attempts to schedule a task with the given schedule data.
    async fn try_schedule(
        &self,
        data: Self::ScheduleData,
    ) -> Result<TaskHandle<Self::Output, Self::TaskData>, Self::ScheduleError>;

    /// Consumes `self` and returns an `AnyTask`.
    ///
    /// This is mainly a convenience method that also helps to ensure that the `Task` trait is object safe.
    fn into_any(
        self,
    ) -> AnyTask<Self::Output, Self::ScheduleError, Self::ScheduleData, Self::TaskData>
    where
        Self: Sized + 'static,
    {
        Box::new(self)
    }
}

/// A scheduled task that is waiting to be given output.
pub struct PendingChannelTask<T> {
    sender: oneshot::Sender<T>,
}

impl<T> PendingChannelTask<T> {
    /// Provide the output of this task to the scheduler.
    pub fn finish(self, data: T) {
        let _ = self.sender.send(data);
    }
}

/// A request to schedule a task.
pub struct ChannelTaskInit<TD, T> {
    task_data_sender: oneshot::Sender<(TD, oneshot::Receiver<T>)>,
}

impl<TD, T> ChannelTaskInit<TD, T> {
    /// Sends the `TaskData` to the scheduler, thus completing the scheduling.
    ///
    /// Returns `None` if the scheduler was dropped before receiving the data.  
    /// Returns `Some(PendingChannelTask)` if the scheduler sucessfully received the `TaskData`.
    pub fn send_task_data(self, data: TD) -> Option<PendingChannelTask<T>> {
        let (sender, recv) = oneshot::channel();
        self.task_data_sender.send((data, recv)).ok()?;
        Some(PendingChannelTask { sender })
    }
}

/// A scheduling error that a `ChannelTask` may return.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ChannelTaskError {
    /// The queue of scheduled tasks is full.
    TaskQueueFull,
    /// The task itself has been dropped.
    TaskDropped,
}

impl Display for ChannelTaskError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TaskQueueFull => write!(f, "The queue of scheduled tasks is full"),
            Self::TaskDropped => write!(f, "The task itself has been dropped"),
        }
    }
}

impl std::error::Error for ChannelTaskError {}

/// A common variant of a `Task` where scheduling of the `Task` is done
/// over `tokio` channels.
///
/// `T` represents the `Output` type.  
/// `SD` represents the `ScheduleData` type.  
/// `TD` represents the `TaskData` type.
///
/// When using this `Task` to create your own `Task`s, you simply have
/// to create a type alias with the generic types substituted for the actual
/// concrete types your `Task` requires.
///
/// # Examples
/// ```
/// use unros_core::task::ChannelTask;
///
/// /// This task performs the factorial with the given integer.
/// ///
/// /// The `TaskData` type is left to its default value of `()`.
/// type FactorialTask = ChannelTask<usize, usize>;
/// ```
#[derive(Clone)]
pub struct ChannelTask<T, SD, TD = ()> {
    sender: mpsc::Sender<(SD, ChannelTaskInit<TD, T>)>,
    task_name: Cow<'static, str>,
}

impl<T, SD, TD> ChannelTask<T, SD, TD> {
    pub fn new(
        size: usize,
        task_name: impl Into<Cow<'static, str>>,
    ) -> (Self, mpsc::Receiver<(SD, ChannelTaskInit<TD, T>)>) {
        let (sender, receiver) = mpsc::channel(size);
        (
            Self {
                sender,
                task_name: task_name.into(),
            },
            receiver,
        )
    }
}

/// A concrete type that can be made from any `Task` with the same associated types.
pub type AnyTask<T, SE, SD, TD = ()> =
    Box<dyn Task<Output = T, ScheduleError = SE, ScheduleData = SD, TaskData = TD>>;

#[async_trait]
impl<T: Send + 'static, SD: Send, TD: Send> Task for ChannelTask<T, SD, TD> {
    type Output = T;
    type ScheduleData = SD;
    type TaskData = TD;
    type ScheduleError = ChannelTaskError;

    async fn try_schedule(
        &self,
        schedule_data: Self::ScheduleData,
    ) -> Result<TaskHandle<Self::Output, Self::TaskData>, Self::ScheduleError> {
        let (task_data_sender, task_data_receiver) = oneshot::channel();
        self.sender
            .try_send((schedule_data, ChannelTaskInit { task_data_sender }))
            .map_err(|e| match e {
                mpsc::error::TrySendError::Full(_) => ChannelTaskError::TaskQueueFull,
                mpsc::error::TrySendError::Closed(_) => ChannelTaskError::TaskDropped,
            })?;
        let (task_data, done_recv) = task_data_receiver
            .await
            .map_err(|_| ChannelTaskError::TaskDropped)?;

        Ok(
            TaskHandle::from_oneshot_receiver(done_recv, task_data, self.task_name.to_string())
                .await,
        )
    }
}

/// The task failed to complete, or it already completed and should not be waited for again.
pub enum TaskCompletionError {
    OutputAlreadyTaken,
    DidNotComplete,
}

/// A handle to a task.
///
/// This handle does not make any attempt at stopping the task when dropped.
/// To implement this functionality, one should utilize the `TaskData`'s `Drop`.
pub struct TaskHandle<T, TD> {
    recv: mpsc::Receiver<T>,
    done: bool,
    data: TD,
}

impl<T, TD> TaskHandle<T, TD> {
    /// Immutably gets a reference to the `TaskData`.
    pub fn get_data(&self) -> &TD {
        &self.data
    }

    /// Mutably gets a reference to the `TaskData`.
    pub fn get_mut_data(&mut self) -> &mut TD {
        &mut self.data
    }

    /// Waits for the task to complete.
    ///
    /// If the task has already completed, this will return `Err(TaskCompletionError::OutputAlreadyTaken)`.
    /// If the task failed to complete (due to a panic or cancellation), this will return `Err(TaskCompletionError::DidNotComplete)`.
    /// Otherwise, the `Output` of the task will be returned.
    pub async fn wait_for_completion(&mut self) -> Result<T, TaskCompletionError> {
        if self.done {
            Err(TaskCompletionError::OutputAlreadyTaken)
        } else {
            let result = self
                .recv
                .recv()
                .await
                .ok_or(TaskCompletionError::DidNotComplete);
            self.done = true;
            result
        }
    }

    /// Creates a `TaskHandle` from a `JoinHandle` from tokio.
    ///
    /// The `task_name` will be used when logging if the given task panics or was cancelled.
    pub async fn from_tokio_task(
        task: tokio::task::JoinHandle<T>,
        data: TD,
        task_name: String,
    ) -> Self
    where
        T: Send + 'static,
    {
        let (sender, recv) = mpsc::channel(1);

        tokio::spawn(async move {
            match task.await {
                Ok(x) => {
                    let _ = sender.send(x);
                }
                Err(e) => {
                    if e.is_cancelled() {
                        error!("{task_name} was cancelled");
                    } else {
                        error!("{task_name} panicked");
                    }
                }
            }
        });

        Self {
            recv,
            done: false,
            data,
        }
    }

    /// Creates a `TaskHandle` from a `oneshot::Receiver` from tokio.
    ///
    /// The `task_name` will be used when logging if the given receiver fails to produce a value.
    pub async fn from_oneshot_receiver(
        oneshot: oneshot::Receiver<T>,
        data: TD,
        task_name: String,
    ) -> Self
    where
        T: Send + 'static,
    {
        let (sender, recv) = mpsc::channel(1);

        tokio::spawn(async move {
            match oneshot.await {
                Ok(x) => {
                    let _ = sender.send(x);
                }
                Err(_) => {
                    error!("{task_name} has failed to complete");
                }
            }
        });

        Self {
            recv,
            done: false,
            data,
        }
    }

    /// Creates a `TaskHandle` from an `mpsc::Receiver` from tokio.
    ///
    /// You should generally avoid using this unless you know what you are doing.
    /// Using this method allows you to make a `TaskHandle` that produces multiple `Output`s,
    /// which goes against its intended usage.
    pub async fn from_mpsc_receiver(recv: mpsc::Receiver<T>, data: TD) -> Self {
        Self {
            recv,
            done: false,
            data,
        }
    }
}
