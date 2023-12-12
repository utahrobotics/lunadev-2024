use std::fmt::Debug;

use burn::{
    config::Config,
    module::Module,
    nn::{
        gru::{Gru, GruConfig},
        loss::{MSELoss, Reduction},
        Dropout, DropoutConfig, LayerNorm, LayerNormConfig, Linear, LinearConfig,
    },
    tensor::{
        backend::Backend,
        Tensor,
    },
    train::RegressionOutput,
};

use crate::{Model, RegressionBatch, TrainableModel};

use super::{Activation, ThreeTuple};

#[derive(Config)]
pub struct GruNetworkConfig {
    pub grus: Vec<(GruConfig, Option<LayerNormConfig>, Activation)>,
    pub linears: Vec<(LinearConfig, Option<LayerNormConfig>, Activation)>,
    pub dropout: DropoutConfig,
}


impl GruNetworkConfig {
    pub fn new_basic(activation: Activation, d_input: usize, d_output: usize, hidden_size: usize, gru_count: usize, linear_count: usize, dropout_prob: f64, bias: bool, normalize: bool) -> Self {
        Self {
            grus: (0..gru_count).into_iter().map(|i| {
                let gru = if i == 0 {
                    GruConfig::new(d_input, hidden_size, bias)
                } else {
                    GruConfig::new(hidden_size, hidden_size, bias)
                };
                (gru, normalize.then(|| LayerNormConfig::new(hidden_size)), activation)
            }).collect(),
            linears: (0..linear_count).into_iter().map(|i| {
                let mut norm = None;
                let linear = if i == linear_count - 1 {
                    LinearConfig::new(hidden_size, d_output)
                } else {
                    norm = normalize.then(|| LayerNormConfig::new(hidden_size));
                    LinearConfig::new(hidden_size, hidden_size)
                };
                (linear, norm, activation)
            }).collect(),
            dropout: DropoutConfig::new(dropout_prob)
        }
    }
}


#[derive(Module)]
pub struct GruNetwork<B: Backend> {
    grus: Vec<ThreeTuple<Gru<B>, Option<LayerNorm<B>>, Activation>>,
    linears: Vec<ThreeTuple<Linear<B>, Option<LayerNorm<B>>, Activation>>,
    dropout: Dropout,
}

unsafe impl<B: Backend> Send for GruNetwork<B> {}
unsafe impl<B: Backend> Sync for GruNetwork<B> {}

impl<B: Backend> Debug for GruNetwork<B> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GruModule")
            .field("grus", &self.grus)
            .field("linear", &self.linears)
            .field("dropout", &self.dropout)
            .finish()
    }
}

impl<B: Backend> Model<B> for GruNetwork<B> {
    type Input = Tensor<B, 3>;
    type Output = Tensor<B, 2>;
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

    fn from_config(config: Self::Config) -> Self {
        Self {
            grus: config
                .grus
                .into_iter()
                .map(|(a, b, c)| ThreeTuple(a.init(), b.map(|x| x.init()), c))
                .collect(),
            linears: config
                .linears
                .into_iter()
                .map(|(a, b, c)| ThreeTuple(a.init(), b.map(|x| x.init()), c))
                .collect(),
            dropout: config.dropout.init(),
        }
    }
}

impl<B: Backend> TrainableModel<B, RegressionOutput<B>> for GruNetwork<B> {
    type Batch = RegressionBatch<B, 3, 2>;

    fn forward_training(&self, batch: Self::Batch) -> RegressionOutput<B> {
        let output = self.forward(batch.inputs);
        let loss = MSELoss::new().forward(output.clone(), batch.targets.clone(), Reduction::Auto);
        RegressionOutput::new(loss, output, batch.targets)
    }
}

#[derive(Config)]
pub struct GruNetworkSuperConfig {
    pub activations: Vec<Activation>,
    pub d_input: usize,
    pub d_output: usize,
    pub min_hidden_size: usize,
    pub max_hidden_size: usize,
    #[config(default = 0)]
    pub min_dropout_prob_step: usize,
    #[config(default = 7)]
    pub max_dropout_prob_step: usize,
    #[config(default = 1)]
    pub min_gru_count: usize,
    pub max_gru_count: usize,
    #[config(default = 1)]
    pub min_linear_count: usize,
    pub max_linear_count: usize,
}


impl IntoIterator for GruNetworkSuperConfig {
    type Item = GruNetworkConfig;

    type IntoIter = GruNetworkSuperConfigIter;

    fn into_iter(self) -> Self::IntoIter {
        let d_input = self.d_input;
        let d_output = self.d_output;
        let min_hidden_size = self.min_hidden_size;
        let max_hidden_size = self.max_hidden_size;
        let min_dropout_prob_step = self.min_dropout_prob_step;
        let max_dropout_prob_step = self.max_dropout_prob_step;
        let min_gru_count = self.min_gru_count;
        let max_gru_count = self.max_gru_count;
        let min_linear_count = self.min_linear_count;
        let max_linear_count = self.max_linear_count;
        GruNetworkSuperConfigIter {
            len: self.activations.len() * (max_hidden_size - min_hidden_size + 1) * (max_dropout_prob_step - min_dropout_prob_step + 1) * (max_gru_count - min_gru_count + 1) * (max_linear_count - min_linear_count + 1) * 4,
            iter: Box::new(
                self.activations.into_iter()
                    .flat_map(move |activation| {
                        (min_hidden_size..=max_hidden_size)
                            .flat_map(move |hidden_size| {
                                (min_dropout_prob_step..=max_dropout_prob_step)
                                    .flat_map(move |dropout_prob_step| {
                                        let dropout_prob = 0.1 * dropout_prob_step as f64;
                                        (min_gru_count..=max_gru_count)
                                            .into_iter()
                                            .flat_map(move |gru_count| {
                                                (min_linear_count..=max_linear_count)
                                                    .into_iter()
                                                    .flat_map(move |linear_count| {
                                                        [
                                                            GruNetworkConfig::new_basic(activation, d_input, d_output, hidden_size, gru_count, linear_count, dropout_prob, false, false),
                                                            GruNetworkConfig::new_basic(activation, d_input, d_output, hidden_size, gru_count, linear_count, dropout_prob, false, true),
                                                            GruNetworkConfig::new_basic(activation, d_input, d_output, hidden_size, gru_count, linear_count, dropout_prob, true, false),
                                                            GruNetworkConfig::new_basic(activation, d_input, d_output, hidden_size, gru_count, linear_count, dropout_prob, true, true),
                                                        ]
                                                    })
                                            })
                                    })
                            })
                    })
            ),
        }
    }
}


pub struct GruNetworkSuperConfigIter {
    iter: Box<dyn Iterator<Item = GruNetworkConfig>>,
    len: usize
}

impl Iterator for GruNetworkSuperConfigIter {
    type Item = GruNetworkConfig;

    fn next(&mut self) -> Option<Self::Item> {
        let out = self.iter.next()?;
        self.len -= 1;
        Some(out)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len, Some(self.len))
    }
}


impl ExactSizeIterator for GruNetworkSuperConfigIter {}