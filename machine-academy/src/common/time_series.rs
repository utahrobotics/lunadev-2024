use std::fmt::Debug;

use burn::{
    module::Module,
    nn::{
        gru::{Gru, GruConfig},
        loss::{MSELoss, Reduction},
        Dropout, LayerNorm, Linear, DropoutConfig, LinearConfig, LayerNormConfig,
    },
    tensor::{
        backend::{AutodiffBackend, Backend},
        Tensor,
    },
    train::RegressionOutput,
    config::Config,
};

use crate::{Model, RegressionBatch};

use super::{Activation, PhantomModule, ThreeTuple};


#[derive(Config)]
pub struct GruNetworkConfig {
    pub grus: Vec<(GruConfig, Option<LayerNormConfig>, Activation)>,
    pub linears: Vec<(LinearConfig, Option<LayerNormConfig>, Activation)>,
    pub dropout: DropoutConfig,
}


#[derive(Module)]
pub struct GruNetwork<B: Backend, T, O> {
    grus: Vec<ThreeTuple<Gru<B>, Option<LayerNorm<B>>, Activation>>,
    linears: Vec<ThreeTuple<Linear<B>, Option<LayerNorm<B>>, Activation>>,
    dropout: Dropout,
    _phantom: PhantomModule<(T, O)>,
}

unsafe impl<B: Backend, T, O> Send for GruNetwork<B, T, O> {}
unsafe impl<B: Backend, T, O> Sync for GruNetwork<B, T, O> {}

impl<B: Backend, T, O> Debug for GruNetwork<B, T, O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GruModule")
            .field("grus", &self.grus)
            .field("linear", &self.linears)
            .field("dropout", &self.dropout)
            .finish()
    }
}

impl<B: AutodiffBackend> Model<B> for GruNetwork<B, RegressionBatch<B, 3, 2>, RegressionOutput<B>> {
    type Input = Tensor<B, 3>;
    type Output = Tensor<B, 2>;
    type LossOutput = RegressionOutput<B>;
    type Batch = RegressionBatch<B, 3, 2>;
    type Config = GruNetworkConfig;

    fn forward(&self, input: Self::Input) -> Self::Output {
        let mut x = input;
        for layer in &self.grus {
            x = layer.0.forward(x, None);
            if let Some(norm) = &layer.1 {
                x = norm.forward(x);
            }
            x = layer.2.forward(x);
            x = self.dropout.forward(x);
        }
        for layer in &self.linears {
            x = layer.0.forward(x);
            if let Some(norm) = &layer.1 {
                x = norm.forward(x);
            }
            x = layer.2.forward(x);
            x = self.dropout.forward(x);
        }
        let [batch_size, a, b] = x.dims();
        x.reshape([batch_size, a * b])
    }

    fn forward_training(&self, batch: Self::Batch) -> Self::LossOutput {
        let output = self.forward(batch.inputs);
        let loss = MSELoss::new().forward(output.clone(), batch.targets.clone(), Reduction::Auto);
        RegressionOutput::new(loss, output, batch.targets)
    }

    fn from_config(config: Self::Config) -> Self {
        Self {
            grus: config.grus.into_iter().map(|(a, b, c)| ThreeTuple(a.init(), b.map(|x| x.init()), c)).collect(),
            linears: config.linears.into_iter().map(|(a, b, c)| ThreeTuple(a.init(), b.map(|x| x.init()), c)).collect(),
            dropout: config.dropout.init(),
            _phantom: Default::default()
        }
    }
}
