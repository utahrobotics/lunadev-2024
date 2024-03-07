use tokio::sync::{mpsc, oneshot};

pub struct ServiceHandle<ScheduleInput, ScheduleError, ScheduleData, TaskOutput> {
    schedule_sender: mpsc::UnboundedSender<
        ScheduleRequest<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>,
    >,
}

impl<ScheduleInput, ScheduleError, ScheduleData, TaskOutput> Clone
    for ServiceHandle<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
{
    fn clone(&self) -> Self {
        Self {
            schedule_sender: self.schedule_sender.clone(),
        }
    }
}

impl<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
    ServiceHandle<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
{
    pub async fn try_schedule_or_closed(
        &self,
        input: ScheduleInput,
    ) -> Option<Result<ScheduledService<ScheduleData, TaskOutput>, ScheduleError>> {
        let (sender, receiver) = oneshot::channel();
        self.schedule_sender
            .send(ScheduleRequest {
                input: Some(input),
                sender,
            })
            .ok()?;
        receiver.await.ok()
    }

    pub async fn try_schedule(
        &self,
        input: ScheduleInput,
    ) -> Result<ScheduledService<ScheduleData, TaskOutput>, ScheduleError> {
        let Some(out) = self.try_schedule_or_closed(input).await else {
            std::future::pending::<()>().await;
            unreachable!()
        };
        out
    }
}

pub struct ScheduledService<ScheduleData, TaskOutput> {
    data: Option<ScheduleData>,
    output_recv: oneshot::Receiver<TaskOutput>,
}

pub struct ScheduleRequest<ScheduleInput, ScheduleError, ScheduleData, TaskOutput> {
    input: Option<ScheduleInput>,
    sender: oneshot::Sender<Result<ScheduledService<ScheduleData, TaskOutput>, ScheduleError>>,
}

pub struct Pending<TaskOutput> {
    output_sender: oneshot::Sender<TaskOutput>,
}

pub struct Service<ScheduleInput, ScheduleError, ScheduleData, TaskOutput> {
    schedule_recv: mpsc::UnboundedReceiver<
        ScheduleRequest<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>,
    >,
}

#[must_use]
pub fn new_service<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>() -> (
    Service<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>,
    ServiceHandle<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>,
) {
    let (schedule_sender, schedule_recv) = mpsc::unbounded_channel();
    (Service { schedule_recv }, ServiceHandle { schedule_sender })
}

impl<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
    Service<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
{
    pub async fn wait_for_request(
        &mut self,
    ) -> Option<ScheduleRequest<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>> {
        self.schedule_recv.recv().await
    }

    pub fn blocking_wait_for_request(
        &mut self,
    ) -> Option<ScheduleRequest<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>> {
        self.schedule_recv.blocking_recv()
    }
}

impl<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
    ScheduleRequest<ScheduleInput, ScheduleError, ScheduleData, TaskOutput>
{
    pub fn accept(self, data: ScheduleData) -> Option<Pending<TaskOutput>> {
        let (output_sender, output_recv) = oneshot::channel();
        if self
            .sender
            .send(Ok(ScheduledService {
                data: Some(data),
                output_recv,
            }))
            .is_err()
        {
            None
        } else {
            Some(Pending { output_sender })
        }
    }

    pub fn reject(self, error: ScheduleError) {
        let _ = self.sender.send(Err(error));
    }

    pub fn get_input(&self) -> Option<&ScheduleInput> {
        self.input.as_ref()
    }

    pub fn get_mut_input(&mut self) -> Option<&mut ScheduleInput> {
        self.input.as_mut()
    }

    pub fn take_input(&mut self) -> Option<ScheduleInput> {
        self.input.take()
    }
}

impl<TaskOutput> Pending<TaskOutput> {
    pub fn finish(self, output: TaskOutput) {
        let _ = self.output_sender.send(output);
    }

    #[must_use]
    pub fn is_closed(&self) -> bool {
        self.output_sender.is_closed()
    }
}

impl<ScheduleData, TaskOutput> ScheduledService<ScheduleData, TaskOutput> {
    pub fn get_data(&self) -> Option<&ScheduleData> {
        self.data.as_ref()
    }

    pub fn get_mut_data(&mut self) -> Option<&mut ScheduleData> {
        self.data.as_mut()
    }

    pub fn take_data(&mut self) -> Option<ScheduleData> {
        self.data.take()
    }

    pub async fn wait_or_closed(self) -> Option<TaskOutput> {
        self.output_recv.await.ok()
    }

    pub async fn wait(self) -> TaskOutput {
        let Some(out) = self.wait_or_closed().await else {
            std::future::pending::<()>().await;
            unreachable!()
        };
        out
    }
}
