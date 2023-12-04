use std::{future::Future, pin::Pin};

use tokio::sync::{broadcast, mpsc};

pub struct TransitionInit<B> {
    sender: mpsc::Sender<B>,
    alive_recv: broadcast::Receiver<()>,
}

pub struct Transitioned {
    fut: Option<Pin<Box<dyn Future<Output = ()> + Send>>>,
}

impl<B> TransitionInit<B> {
    pub fn next_state<T, R, Fut>(mut self, blackboard: B, state: fn(B) -> Fut) -> Transitioned
    where
        B: Send + 'static,
        Fut: Future<Output = (B, R)> + Send + 'static,
        T: Transition<R, B>,
    {
        Transitioned {
            fut: Some(Box::pin(async move {
                let (blackboard, output) = state(blackboard).await;

                let trans = T::transition(
                    output,
                    blackboard,
                    TransitionInit {
                        sender: self.sender.clone(),
                        alive_recv: self.alive_recv.resubscribe(),
                    },
                );
                let Some(fut) = trans.fut else {
                    return;
                };
                tokio::spawn(async move {
                    tokio::select! {
                        _ = self.alive_recv.recv() => return,
                        () = fut => { }
                    }
                });
            })),
        }
    }

    pub fn exit_machine(self, blackboard: B) -> Transitioned {
        self.sender.try_send(blackboard).unwrap();
        Transitioned { fut: None }
    }
}

pub trait Transition<T, B> {
    fn transition(input: T, blackboard: B, init: TransitionInit<B>) -> Transitioned;
}

pub async fn start_machine<T, B, R, Fut>(blackboard: B, init_state: fn(B) -> Fut) -> B
where
    B: Send + 'static,
    Fut: Future<Output = (B, R)> + Send,
    T: Transition<R, B>,
{
    let (blackboard, output) = init_state(blackboard).await;
    let (sender, mut receiver) = mpsc::channel(1);
    let (_alive_sender, alive_recv) = broadcast::channel(1);

    let trans = T::transition(output, blackboard, TransitionInit { sender, alive_recv });
    let Some(fut) = trans.fut else {
        return receiver.recv().await.expect("State has panicked");
    };
    fut.await;
    receiver.recv().await.expect("State has panicked")
}

#[cfg(test)]
mod tests {
    use crate::{start_machine, Transition};

    struct SumBlackboard {
        sum: usize,
    }

    struct SumTransition;

    impl Transition<(), SumBlackboard> for SumTransition {
        fn transition(
            _: (),
            blackboard: SumBlackboard,
            init: crate::TransitionInit<SumBlackboard>,
        ) -> crate::Transitioned {
            init.next_state::<CheckTransition, _, _>(blackboard, check_state)
        }
    }

    struct CheckTransition;

    impl Transition<bool, SumBlackboard> for CheckTransition {
        fn transition(
            input: bool,
            blackboard: SumBlackboard,
            init: crate::TransitionInit<SumBlackboard>,
        ) -> crate::Transitioned {
            if input {
                init.exit_machine(blackboard)
            } else {
                init.next_state::<SumTransition, _, _>(blackboard, sum_state)
            }
        }
    }

    async fn sum_state(mut bb: SumBlackboard) -> (SumBlackboard, ()) {
        bb.sum += 1;
        (bb, ())
    }

    async fn check_state(bb: SumBlackboard) -> (SumBlackboard, bool) {
        let finished = bb.sum >= 10;
        (bb, finished)
    }

    #[tokio::test]
    async fn test_adder01() {
        let mut bb = SumBlackboard { sum: 0 };
        bb = start_machine::<SumTransition, _, _, _>(bb, sum_state).await;
        assert_eq!(bb.sum, 10);
    }
}
