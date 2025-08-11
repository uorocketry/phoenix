use defmt::*;
use embassy_executor::task;
use embassy_time::{Duration, Timer};

// Inference entrypoint (moved out of main). Behavior intentionally minimal for now.
// The full reference code from main is preserved below as comments.
#[task]
pub async fn inference_task() {
    info!("Inference task started.");
    loop {
        // TODO: wire real model inputs/outputs once data sources are ready
        Timer::after(Duration::from_secs(1)).await;
    }
}

// --- Reference: Original inference setup from main.rs ---
// // Get a default device for the backend
// type Backend = burn::backend::NdArray<f32>;
// type BackendDevice = <Backend as burn::tensor::backend::Backend>::Device;
// let device = BackendDevice::default();
//
// // Create a new model and load the state
// let recorder = burn::record::BinBytesRecorder::<burn::record::FullPrecisionSettings>::new();
// let record_bytes = include_bytes!("model/tte.mpk");
// let record = recorder
//     .load(record_bytes.as_ref(), &device)
//     .expect("Failed to load recorder");
//
// let model: crate::model::LstmNetwork<Backend> = crate::model::LstmNetwork::new(&device).load_record(record);
// // Example inference call (define real input tensor as needed):
// // let input: burn::tensor::Tensor<Backend, 3> = ...;
// // let output = model.forward(input);

// --- Reference helper moved from main.rs ---
// fn run_model<'a>(model: &Model<NdArray>, device: &BackendDevice, input: f32) -> Tensor<Backend, 2> {
//     // Define the tensor
//     let input = Tensor::<Backend, 2>::from_floats([[input]], &device);
//
//     // Run the model on the input
//     let output = model.forward(input);
//
//     output
// }
