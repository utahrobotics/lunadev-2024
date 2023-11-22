//! A task is a long running action whose progress or result may be of importance.
//! 
//! In robotics, a task could be as simple as moving an arm to some angle. If an
//! autonomous process wants to move the arm to some angle, it schedules the
//! appropriate task, and watches its progress (the angle of the arm). If there
//! was an error (such as the arm getting stuck), the task may returrn an error
//! to the autonomous process, allowing it to intelligently handle it.

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
    ) -> Result<TaskHandle<Self>, Self::ScheduleError>;
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
pub struct TaskHandle<T: Task + ?Sized> {
    recv: mpsc::Receiver<T::Output>,
    done: bool,
    data: T::TaskData,
}

impl<T: Task + ?Sized> TaskHandle<T> {
    /// Immutably gets a reference to the `TaskData`.
    pub fn get_data(&self) -> &T::TaskData {
        &self.data
    }

    /// Mutably gets a reference to the `TaskData`.
    pub fn get_mut_data(&mut self) -> &mut T::TaskData {
        &mut self.data
    }

    /// Waits for the task to complete.
    /// 
    /// If the task has already completed, this will return `Err(TaskCompletionError::OutputAlreadyTaken)`.
    /// If the task failed to complete (due to a panic or cancellation), this will return `Err(TaskCompletionError::DidNotComplete)`.
    /// Otherwise, the `Output` of the task will be returned.
    pub async fn wait_for_completion(&mut self) -> Result<T::Output, TaskCompletionError> {
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
        task: tokio::task::JoinHandle<T::Output>,
        data: T::TaskData,
        task_name: String,
    ) -> Self
    where
        T::Output: 'static,
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
        oneshot: oneshot::Receiver<T::Output>,
        data: T::TaskData,
        task_name: String,
    ) -> Self
    where
        T::Output: 'static,
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
    pub async fn from_mpsc_receiver(recv: mpsc::Receiver<T::Output>, data: T::TaskData) -> Self {
        Self {
            recv,
            done: false,
            data,
        }
    }
}
