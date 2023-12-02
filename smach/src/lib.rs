use async_trait::async_trait;
use std::{
    future::Future,
    marker::PhantomData,
    sync::{Arc, Weak},
};
use tokio::sync::{oneshot, Mutex};

// #[async_trait]
// trait StateFn<B, R>: Send {
//     async fn call(&mut self, blackboard: &mut B) -> StateEnded<B, R>;
// }

// struct SendSyncPhantom<T>(PhantomData<fn(T)>);

// // unsafe impl<T> Send for SendSyncPhantom<T> {}
// // unsafe impl<T> Sync for SendSyncPhantom<T> {}

// impl<T> Default for SendSyncPhantom<T> {
//     fn default() -> Self {
//         Self(PhantomData)
//     }
// }

// #[async_trait]
// impl<'a, B, SB, R, F, Fut> StateFn<B, R> for (F, SendSyncPhantom<SB>)
// where
//     SB: 'a,
//     B: AsSubBlackboard<'a, SB> + Send,
//     F: FnMut(SB) -> Fut + Send + 'static,
//     Fut: Future<Output = StateEnded<B, R>> + Send,
// {
//     async fn call(&mut self, blackboard: &mut B) -> StateEnded<B, R> {
//         (self.0)(blackboard.as_sub()).await
//     }
// }

enum StateEndedEnum<B, R> {
    NextState(Arc<Mutex<Box<dyn StateFn<B, R>>>>),
    End(R),
}

pub struct StateEnded<B, R>(StateEndedEnum<B, R>);

impl<B, R> StateEnded<B, R> {
    pub fn transition(next_state: State<B, R>) -> Self {
        Self(StateEndedEnum::NextState(next_state.func))
    }
    pub fn end(return_val: R) -> Self {
        Self(StateEndedEnum::End(return_val))
    }
}

pub trait AsSubBlackboard<'a, SB: 'a> {
    fn as_sub(&'a mut self) -> SB;
}

impl<'a, T> AsSubBlackboard<'a, ()> for T {
    fn as_sub(&mut self) -> () {
        ()
    }
}

pub struct State<B, R> {
    func: Arc<Mutex<Box<dyn StateFn<B, R>>>>,
}

impl<B, R> Clone for State<B, R> {
    fn clone(&self) -> Self {
        Self {
            func: self.func.clone(),
        }
    }
}

impl<B, R> State<B, R> {
    pub fn new() -> Self
    where
        B: Send,
    {
        Self {
            func: Arc::new(Mutex::new(Box::new((
                |_| async { unimplemented!() },
                SendSyncPhantom::default(),
            )))),
        }
    }

    pub fn provide_fn<SB, F, Fut>(&mut self, f: F)
    where
        SB: 'static,
        B: AsSubBlackboard<'static, SB> + Send,
        F: FnMut(SB) -> Fut + Send + 'static,
        Fut: Future<Output = StateEnded<B, R>> + Send,
    {
        self.func = Arc::new(Mutex::new(Box::new((f, SendSyncPhantom::default()))));
    }

    pub async fn run(&mut self, blackboard: &mut B) -> R {
        match self.func.lock().await.call(blackboard).await.0 {
            StateEndedEnum::NextState(mut current_state) => loop {
                let next_state;
                match current_state.lock().await.call(blackboard).await.0 {
                    StateEndedEnum::NextState(x) => next_state = x,
                    StateEndedEnum::End(x) => break x,
                };
                current_state = next_state;
            },

            StateEndedEnum::End(x) => x,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{AsSubBlackboard, State, StateEnded};

    struct SumBlackBoard {
        sum: usize,
    }

    struct SubSumBlackBoard<'a> {
        sum: &'a mut usize,
    }

    impl<'a> AsSubBlackboard<'a, SubSumBlackBoard<'a>> for SumBlackBoard {
        fn as_sub(&'a mut self) -> SubSumBlackBoard<'a> {
            SubSumBlackBoard { sum: &mut self.sum }
        }
    }

    #[tokio::test]
    async fn add_one() {
        let mut adder = State::<SumBlackBoard, usize>::new();
        let mut checker = State::<SumBlackBoard, usize>::new();
        let adder2 = adder.clone();

        checker.provide_fn(move |bb: SubSumBlackBoard| {
            let adder = adder2.clone();
            async move {
                if *bb.sum == 20 {
                    StateEnded::end(*bb.sum)
                } else {
                    StateEnded::transition(adder.clone())
                }
            }
        });

        let checker2 = checker.clone();
        adder.provide_fn(move |bb: SubSumBlackBoard| {
            let checker = checker2.clone();
            async {
                *bb.sum += 1;
                StateEnded::transition(checker)
            }
        });

        let mut blackboard = SumBlackBoard { sum: 0 };
        let returned = checker.run(&mut blackboard).await;
        assert_eq!(returned, blackboard.sum);
        assert_eq!(returned, 20);
    }
}
