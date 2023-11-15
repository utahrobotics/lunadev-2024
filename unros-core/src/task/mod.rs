use async_trait::async_trait;
use log::error;
use tokio::sync::{mpsc, oneshot};

#[async_trait]
pub trait Task: Sync {
    type Output: Send;
    type ScheduleError: std::error::Error;
    type ScheduleData;
    type TaskData = ();

    async fn try_schedule(
        &self,
        data: Self::ScheduleData,
    ) -> Result<TaskHandle<Self>, Self::ScheduleError>;
}

pub enum TaskCompletionError {
    OutputAlreadyTaken,
    DidNotComplete,
}

pub struct TaskHandle<T: Task + ?Sized> {
    recv: mpsc::Receiver<T::Output>,
    done: bool,
    data: T::TaskData,
}

impl<T: Task + ?Sized> TaskHandle<T> {
    pub fn get_data(&self) -> &T::TaskData {
        &self.data
    }

    pub fn get_mut_data(&mut self) -> &mut T::TaskData {
        &mut self.data
    }

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

    pub async fn from_mpsc_receiver(recv: mpsc::Receiver<T::Output>, data: T::TaskData) -> Self {
        Self {
            recv,
            done: false,
            data,
        }
    }
}
