use burn_import::burn::graph::RecordType;
use burn_import::onnx::ModelGen;
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    generate_model();
}

fn generate_model() {
    // Generate the model code from the ONNX file.
    ModelGen::new()
        .input("src/model/sine.onnx")
        .out_dir("model/")
        .record_type(RecordType::Bincode)
        .embed_states(true)
        .run_from_script();
}