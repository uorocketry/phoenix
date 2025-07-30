#![allow(dead_code)]
use burn::{
    module::Module,
    nn::{
        Dropout, DropoutConfig, Initializer, LayerNorm, LayerNormConfig, Linear, LinearConfig,
        LstmState, Sigmoid, Tanh,
    },
    prelude::*,
};
use heapless::Vec;

// =================================================================================
// This file is a `no_std` compatible version of the model used for training.
// The `StackedLstm` has been removed to be compatible with `#[derive(Module)]`.
// =================================================================================

// --- LstmCell ---
#[derive(Module, Debug)]
pub struct LstmCell<B: Backend> {
    pub hidden_size: usize,
    pub weight_ih: Linear<B>,
    pub weight_hh: Linear<B>,
    pub norm_x: Option<LayerNorm<B>>,
    pub norm_h: Option<LayerNorm<B>>,
    pub norm_c: Option<LayerNorm<B>>,
    pub dropout: Dropout,
}

impl<B: Backend> LstmCell<B> {
    pub fn new(input_size: usize, hidden_size: usize, dropout: f64, layer_norm: bool, device: &B::Device) -> Self {
        let initializer = Initializer::XavierNormal { gain: 1.0 };
        let init_bias = Tensor::<B, 1>::ones([hidden_size], device);

        let mut weight_ih = LinearConfig::new(input_size, 4 * hidden_size).with_initializer(initializer.clone()).init(device);
        let bias = weight_ih.bias.clone().unwrap().val().slice_assign([hidden_size..2 * hidden_size], init_bias.clone());
        weight_ih.bias = weight_ih.bias.map(|p| p.map(|_t| bias));

        let mut weight_hh = LinearConfig::new(hidden_size, 4 * hidden_size).with_initializer(initializer).init(device);
        let bias = weight_hh.bias.clone().unwrap().val().slice_assign([hidden_size..2 * hidden_size], init_bias);
        weight_hh.bias = weight_hh.bias.map(|p| p.map(|_t| bias));

        Self {
            hidden_size,
            weight_ih,
            weight_hh,
            norm_x: if layer_norm { Some(LayerNormConfig::new(4 * hidden_size).init(device)) } else { None },
            norm_h: if layer_norm { Some(LayerNormConfig::new(hidden_size).init(device)) } else { None },
            norm_c: if layer_norm { Some(LayerNormConfig::new(hidden_size).init(device)) } else { None },
            dropout: DropoutConfig::new(dropout).init(),
        }
    }

    pub fn forward(&self, x: Tensor<B, 2>, state: LstmState<B, 2>) -> LstmState<B, 2> {
        let (h_prev, c_prev) = (state.hidden, state.cell);
        let gates_x = self.weight_ih.forward(x);
        let gates_h = self.weight_hh.forward(h_prev);
        let gates_x = self.norm_x.as_ref().map_or(gates_x.clone(), |norm| norm.forward(gates_x));
        let gates = gates_x + gates_h;
        let gates = gates.chunk(4, 1);
        let i_t = Sigmoid::new().forward(gates[0].clone());
        let f_t = Sigmoid::new().forward(gates[1].clone());
        let g_t = Tanh::new().forward(gates[2].clone());
        let o_t = Sigmoid::new().forward(gates[3].clone());
        let c_t = f_t * c_prev + i_t * g_t;
        let c_t = self.norm_c.as_ref().map_or(c_t.clone(), |norm| norm.forward(c_t));
        let h_t = o_t * Tanh::new().forward(c_t.clone());
        let h_t = self.norm_h.as_ref().map_or(h_t.clone(), |norm| norm.forward(h_t));
        let h_t = self.dropout.forward(h_t);
        LstmState::new(h_t, c_t)
    }

    pub fn init_state(&self, batch_size: usize, device: &B::Device) -> LstmState<B, 2> {
        let cell = Tensor::zeros([batch_size, self.hidden_size], device);
        let hidden = Tensor::zeros([batch_size, self.hidden_size], device);
        LstmState::new(cell, hidden)
    }
}

// --- LstmNetwork ---
#[derive(Module, Debug)]
pub struct LstmNetwork<B: Backend> {
    pub layer_0: LstmCell<B>,
    pub layer_1: LstmCell<B>,
    pub dropout: Dropout,
    pub fc: Linear<B>,
}

impl<B: Backend> LstmNetwork<B> {
    pub fn new(device: &B::Device) -> Self {
        const INPUT_SIZE: usize = 12;
        const HIDDEN_SIZE: usize = 32;
        const OUTPUT_SIZE: usize = 3;
        const DROPOUT: f64 = 0.2;
        const LAYER_NORM: bool = true;

        let layer_0 = LstmCell::new(INPUT_SIZE, HIDDEN_SIZE, DROPOUT, LAYER_NORM, device);
        let layer_1 = LstmCell::new(HIDDEN_SIZE, HIDDEN_SIZE, 0.0, LAYER_NORM, device);

        let fc = LinearConfig::new(HIDDEN_SIZE, OUTPUT_SIZE).init(device);
        let dropout = DropoutConfig::new(DROPOUT).init();

        Self { layer_0, layer_1, dropout, fc }
    }

    pub fn forward(&self, x: Tensor<B, 3>) -> Tensor<B, 2> {
        let [batch_size, seq_length, _] = x.dims();
        let device = x.device();

        // This avoids the need for cloning and resolves the compiler error.
        let state_0 = self.layer_0.init_state(batch_size, &device);
        let state_1 = self.layer_1.init_state(batch_size, &device);

        // Layer 0 forward pass
        let (output_0, _) = self.run_layer(x, state_0, &self.layer_0);

        // Layer 1 forward pass
        let (output_1, _) = self.run_layer(output_0, state_1, &self.layer_1);

        let output = self.dropout.forward(output_1);
        self.fc.forward(output.slice([0..batch_size, seq_length - 1..seq_length]).squeeze(1))
    }

    // Helper function to run a single LSTM layer over a sequence
    fn run_layer(&self, x: Tensor<B, 3>, mut state: LstmState<B, 2>, layer: &LstmCell<B>) -> (Tensor<B, 3>, LstmState<B, 2>) {
        let [batch_size, seq_length, _] = x.dims();
        let mut outputs: Vec<Tensor<B, 2>, 50> = Vec::new(); // 50 is SEQ_LENGTH

        for t in 0..seq_length {
            let input_t = x.clone().slice([0..batch_size, t..t+1]).squeeze(1);
            state = layer.forward(input_t, state);
            outputs.push(state.hidden.clone()).ok();
        }

        let stacked_output: Tensor<B, 3> = Tensor::stack(outputs.into_iter().collect(), 1);
        (stacked_output, state)
    }
}
