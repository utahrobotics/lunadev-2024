use std::{ops::Deref, time::Duration};

use lunabot_lib::{ArmAction, ArmParameters, AutonomyAction, Steering};
use ordered_float::NotNan;
use smach::{State, StateResult};
use unros::{
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, Publisher, PublisherRef, Subscriber, WatchSubscriber},
    runtime::RuntimeContext,
    setup_logging, tokio,
};

use crate::actuators::ArmValues;

pub struct Autonomy {
    arm_pub: Publisher<ArmParameters<ArmAction>>,
    steering_pub: Publisher<Steering>,
    arm_values_sub: WatchSubscriber<ArmValues>,
    action_sub: Subscriber<AutonomyAction>,
}

impl Default for Autonomy {
    fn default() -> Self {
        Self {
            arm_pub: Default::default(),
            steering_pub: Default::default(),
            arm_values_sub: WatchSubscriber::new(ArmValues::default()),
            action_sub: Subscriber::new(8),
        }
    }
}

impl Autonomy {
    pub fn get_arm_pub(&self) -> PublisherRef<ArmParameters<ArmAction>> {
        self.arm_pub.get_ref()
    }

    pub fn get_steering_pub(&self) -> PublisherRef<Steering> {
        self.steering_pub.get_ref()
    }

    pub fn create_arm_values_sub(&self) -> DirectSubscription<ArmValues> {
        self.arm_values_sub.create_subscription()
    }

    pub fn create_autonomy_sub(&self) -> DirectSubscription<AutonomyAction> {
        self.action_sub.create_subscription()
    }
}

impl AsyncNode for Autonomy {
    type Result = ();

    async fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);

        loop {
            match self.action_sub.recv().await {
                AutonomyAction::Dig => {
                    let (prep_bucket, prep_bucket_trans) = State::new(
                        |mut state: Autonomy| async move {
                            state.arm_pub.set(ArmParameters {
                                lift: ArmAction::SetValue(200),
                                tilt: ArmAction::SetValue(190),
                            });
                            loop {
                                tokio::select! {
                                    _ = async {
                                        WatchSubscriber::try_update(&mut state.arm_values_sub);
                                        let last_value = *state.arm_values_sub.deref();
                                        loop {
                                            WatchSubscriber::update(&mut state.arm_values_sub).await;
                                            if last_value != *state.arm_values_sub.deref() {
                                                break;
                                            }
                                        }
                                    } => {}
                                    () = tokio::time::sleep(Duration::from_millis(1000)) => break,
                                    _ = state.action_sub.recv() => return StateResult::new(state, None),
                                }
                            }
                            StateResult::new(state, Some(()))
                        },
                    );
                    let (lower_bucket, lower_bucket_trans) = State::new(
                        |mut state: Autonomy| async move {
                            state.arm_pub.set(ArmParameters {
                                lift: ArmAction::SetValue(120),
                                tilt: ArmAction::SetValue(190),
                            });
                            loop {
                                tokio::select! {
                                    _ = async {
                                        WatchSubscriber::try_update(&mut state.arm_values_sub);
                                        let last_value = *state.arm_values_sub.deref();
                                        loop {
                                            WatchSubscriber::update(&mut state.arm_values_sub).await;
                                            if last_value != *state.arm_values_sub.deref() {
                                                break;
                                            }
                                        }
                                    } => {}
                                    () = tokio::time::sleep(Duration::from_millis(1000)) => break,
                                    _ = state.action_sub.recv() => return StateResult::new(state, None),
                                }
                            }
                            StateResult::new(state, Some(()))
                        },
                    );

                    let (drive_forward, drive_forward_trans) = State::new(
                        |state: Autonomy| async move {
                            state.arm_pub.set(ArmParameters {
                                lift: ArmAction::SetValue(130),
                                tilt: ArmAction::SetValue(160),
                            });
                            for _ in 0..10 {
                                state.steering_pub.set(Steering {
                                    left: NotNan::new(-1.0).unwrap(),
                                    right: NotNan::new(-1.0).unwrap(),
                                });
                                tokio::select! {
                                    () = tokio::time::sleep(Duration::from_millis(75)) => {}
                                    _ = state.action_sub.recv() => return StateResult::new(state, None),
                                }
                            }
                            StateResult::new(state, Some(()))
                        },
                    );

                    let (raise_bucket, raise_bucket_trans) = State::new(
                        |mut state: Autonomy| async move {
                            state.steering_pub.set(Steering {
                                left: NotNan::new(0.0).unwrap(),
                                right: NotNan::new(0.0).unwrap(),
                            });
                            state.arm_pub.set(ArmParameters {
                                lift: ArmAction::SetValue(240),
                                tilt: ArmAction::SetValue(100),
                            });
                            loop {
                                tokio::select! {
                                    _ = async {
                                        WatchSubscriber::try_update(&mut state.arm_values_sub);
                                        let last_value = *state.arm_values_sub.deref();
                                        loop {
                                            WatchSubscriber::update(&mut state.arm_values_sub).await;
                                            if last_value != *state.arm_values_sub.deref() {
                                                break;
                                            }
                                        }
                                    } => {}
                                    () = tokio::time::sleep(Duration::from_millis(1000)) => break,
                                    _ = state.action_sub.recv() => return StateResult::from(state),
                                }
                            }
                            StateResult::from(state)
                        },
                    );

                    prep_bucket_trans.set_transition(move |ret| ret.map(|_| lower_bucket.clone()));
                    lower_bucket_trans
                        .set_transition(move |ret| ret.map(|_| drive_forward.clone()));
                    drive_forward_trans
                        .set_transition(move |ret| ret.map(|_| raise_bucket.clone()));
                    raise_bucket_trans.set_transition(move |_| None);

                    self = prep_bucket.start(self).await;

                    self.arm_pub.set(ArmParameters {
                        lift: ArmAction::Stop,
                        tilt: ArmAction::Stop,
                    });
                    self.steering_pub.set(Steering {
                        left: NotNan::new(0.0).unwrap(),
                        right: NotNan::new(0.0).unwrap(),
                    });
                }
                AutonomyAction::Dump => {
                    let (drive_forward, drive_forward_trans) = State::new(
                        |state: Autonomy| async move {
                            state.arm_pub.set(ArmParameters {
                                lift: ArmAction::SetValue(220),
                                tilt: ArmAction::SetValue(150),
                            });
                            for _ in 0..10 {
                                state.steering_pub.set(Steering {
                                    left: NotNan::new(-0.5).unwrap(),
                                    right: NotNan::new(-0.5).unwrap(),
                                });
                                tokio::select! {
                                    () = tokio::time::sleep(Duration::from_millis(200)) => {}
                                    _ = state.action_sub.recv() => return StateResult::new(state, None),
                                }
                            }
                            StateResult::new(state, Some(()))
                        },
                    );

                    let (lower_bucket, lower_bucket_trans) = State::new(
                        |mut state: Autonomy| async move {
                            state.steering_pub.set(Steering {
                                left: NotNan::new(0.0).unwrap(),
                                right: NotNan::new(0.0).unwrap(),
                            });
                            state.arm_pub.set(ArmParameters {
                                lift: ArmAction::SetValue(220),
                                tilt: ArmAction::SetValue(u8::MAX),
                            });
                            loop {
                                tokio::select! {
                                    _ = async {
                                        WatchSubscriber::try_update(&mut state.arm_values_sub);
                                        let last_value = *state.arm_values_sub.deref();
                                        loop {
                                            WatchSubscriber::update(&mut state.arm_values_sub).await;
                                            if last_value != *state.arm_values_sub.deref() {
                                                break;
                                            }
                                        }
                                    } => {}
                                    () = tokio::time::sleep(Duration::from_millis(1000)) => break,
                                    _ = state.action_sub.recv() => return StateResult::new(state, None),
                                }
                            }
                            tokio::select! {
                                () = tokio::time::sleep(Duration::from_millis(2000)) => {}
                                _ = state.action_sub.recv() => return StateResult::new(state, None),
                            }
                            StateResult::new(state, Some(()))
                        },
                    );

                    let (reverse_dump, reverse_dump_trans) =
                        State::new(|state: Autonomy| async move {
                            for _ in 0..3 {
                                state.steering_pub.set(Steering {
                                    left: NotNan::new(1.0).unwrap(),
                                    right: NotNan::new(1.0).unwrap(),
                                });
                                tokio::select! {
                                    () = tokio::time::sleep(Duration::from_millis(250)) => {}
                                    _ = state.action_sub.recv() => break,
                                }
                            }
                            StateResult::from(state)
                        });

                    drive_forward_trans
                        .set_transition(move |ret| ret.map(|_| lower_bucket.clone()));
                    lower_bucket_trans.set_transition(move |ret| ret.map(|_| reverse_dump.clone()));
                    reverse_dump_trans.set_transition(move |_| None);

                    self = drive_forward.start(self).await;

                    self.arm_pub.set(ArmParameters {
                        lift: ArmAction::Stop,
                        tilt: ArmAction::Stop,
                    });
                    self.steering_pub.set(Steering {
                        left: NotNan::new(0.0).unwrap(),
                        right: NotNan::new(0.0).unwrap(),
                    });
                }
                AutonomyAction::Stop => {}
            }
        }
    }
}
