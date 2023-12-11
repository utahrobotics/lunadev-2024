use std::fmt::Debug;

use burn::{
    module::Module,
    nn::{
        gru::Gru,
        loss::{MSELoss, Reduction},
        Dropout, LayerNorm, Linear,
    },
    tensor::{
        backend::{AutodiffBackend, Backend},
        Tensor,
    },
    train::RegressionOutput,
};

use crate::{Model, RegressionBatch};

use super::{Activation, PhantomModule, ThreeTuple};

#[derive(Module)]
pub struct GruModule<B: Backend, T, O> {
    grus: Vec<ThreeTuple<Gru<B>, Option<LayerNorm<B>>, Activation>>,
    linear: Vec<ThreeTuple<Linear<B>, Option<LayerNorm<B>>, Activation>>,
    dropout: Dropout,
    _phantom: PhantomModule<(T, O)>,
}

unsafe impl<B: Backend, T, O> Send for GruModule<B, T, O> {}
unsafe impl<B: Backend, T, O> Sync for GruModule<B, T, O> {}

impl<B: Backend, T, O> Debug for GruModule<B, T, O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GruModule")
            .field("grus", &self.grus)
            .field("linear", &self.linear)
            .field("dropout", &self.dropout)
            .finish()
    }
}

impl<B: AutodiffBackend> Model<B> for GruModule<B, RegressionBatch<B, 3, 2>, RegressionOutput<B>> {
    type Input = Tensor<B, 3>;
    type Output = Tensor<B, 2>;
    type LossOutput = RegressionOutput<B>;
    type Batch = RegressionBatch<B, 3, 2>;

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
        for layer in &self.linear {
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
}
