use crate::model;
use burn::backend::NdArray;
use burn::prelude::*;
use burn::record::Recorder;
use defmt::info;
use embassy_time::{Duration, Timer};
use heapless::{HistoryBuffer, Vec};

type AiBackend = NdArray<f32>;
type AiDevice = <AiBackend as burn::tensor::backend::Backend>::Device;

const SEQ_LENGTH: usize = 50;
const NUM_FEATURES: usize = 12;

// --- PASTE CONSTANTS FROM PYTHON SCRIPT HERE ---
// Replace these dummy values with the actual output from clean_rocket_data.py
const SCALE_MIN: [f32; NUM_FEATURES] = [0.0f32; 12];
const SCALE_MAX: [f32; NUM_FEATURES] = [1.0f32; 12];
// ------------------------------------------------

/// Normalizes a single feature value using the pre-calculated min/max.
fn normalize_value(value: f32, min: f32, max: f32) -> f32 {
    if (max - min) == 0.0 {
        return 0.0; // Avoid division by zero
    }
    (value - min) / (max - min)
}

#[embassy_executor::task]
pub async fn ai_task() {
    info!("AI Inference Task starting...");

    let device = AiDevice::default();

    // 1. Create the model structure
    info!("Initializing model structure...");
    let model: model::LstmNetwork<AiBackend> = model::LstmNetwork::new(&device);

    // 2. Load the trained weights from the embedded file
    info!("Loading trained weights...");
    let recorder = burn::record::NoStdInferenceRecorder::new();
    let record_bytes = include_bytes!("models/model.bin");
    info!("Loaded {} bytes of model weights.", record_bytes.len());
    let record = recorder
        .load(record_bytes.to_vec(), &device)
        .expect("Failed to load model weights");
    let model = model.load_record(record);
    info!("Model loaded successfully.");

    let mut sensor_history: HistoryBuffer<[f32; NUM_FEATURES], SEQ_LENGTH> = HistoryBuffer::new();

    loop {
        let latest_sensor_data: [f32; NUM_FEATURES] = [0.0; 12]; // Dummy data

        let mut normalized_data = [0.0f32; NUM_FEATURES];
        for i in 0..NUM_FEATURES {
            normalized_data[i] = normalize_value(latest_sensor_data[i], SCALE_MIN[i], SCALE_MAX[i]);
        }
        sensor_history.write(normalized_data);

        if sensor_history.len() == sensor_history.capacity() {
            let mut flat_history: Vec<f32, { SEQ_LENGTH * NUM_FEATURES }> = Vec::new();
            for frame in sensor_history.iter() {
                for value in frame.iter() {
                    flat_history.push(*value).ok();
                }
            }

            let input = Tensor::<AiBackend, 3>::from_floats(flat_history.as_slice(), &device)
                .reshape([1, SEQ_LENGTH, NUM_FEATURES]);

            let output_log = model.forward(input);
            let output_sec = (output_log.exp() - 1.0).into_data();
            let predictions = output_sec.as_slice::<f32>().unwrap();

            info!(
                "PREDICTIONS -> Burnout: {=f32}s, Apogee: {=f32}s, Impact: {=f32}s",
                predictions[0], predictions[1], predictions[2]
            );
        }

        Timer::after(Duration::from_millis(100)).await; // Run at 10Hz
    }
}
