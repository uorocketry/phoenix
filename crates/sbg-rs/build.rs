use std::path::{Path, PathBuf};

/// The path to the sbgECom C library relative to this script.
const SGB_C_LIB_PATH: &str = "sbgECom";
/// The name of the built sbgECom library to link against.
const SBG_C_LIB_NAME: &str = "sbgECom";
/// The header file for bindgen to create Rust bindings for.
const BINDGEN_HEADER: &str = "wrapper.h";

/// Speficy the CMake generator based on the target platform.
#[cfg(windows)]
const GENERATOR: &str = "MinGW Makefiles";
#[cfg(not(windows))]
const GENERATOR: &str = "Unix Makefiles";

fn main() {
    build_c_library();
    create_rust_bindings();
}

/// Builds the sbgECom C library using CMake.
fn build_c_library() {
    // Get the toolchain.cmake file for cross-compilation
    let toolchain_path = normalize_path(Path::new("toolchain.cmake").to_path_buf());

    // Build the SBG C library with CMake
    let dst = cmake::Config::new(SGB_C_LIB_PATH)
        .generator(GENERATOR)
        .define("CMAKE_TOOLCHAIN_FILE", toolchain_path)
        .build();

    // Special instructions for cargo:
    // - cargo:rustc-link-search={path} tells cargo where to find the library
    // - cargo:rustc-link-lib=static={name} tells cargo to link the library as static
    // - cargo:rerun-if-changed={path} tells cargo to rerun the build script if the library changes
    println!("cargo:rustc-link-search={}", dst.join("lib").display());
    println!("cargo:rustc-link-lib=static={}", SBG_C_LIB_NAME);
    println!("cargo:rerun-if-changed={}", SBG_C_LIB_NAME);
}

/// Generates Rust bindings for the SBG C library using bindgen.
fn create_rust_bindings() {
    // The sysroot path specifies where to find the cross-compilation headers
    let sysroot_path = {
        // Run `arm-none-eabi-gcc -print-sysroot` to get the sysroot path.
        // This is the binary used to cross-compile the C library thus
        // the headers provided should suffice to create the bindings.
        let output = std::process::Command::new("arm-none-eabi-gcc")
            .arg("-print-sysroot")
            .output()
            .expect("Failed to execute arm-none-eabi-gcc");
        if output.status.success() {
            String::from_utf8(output.stdout)
                .expect("Invalid UTF-8 in sysroot path")
                .trim()
                .to_string()
        } else {
            panic!("Failed to detect sysroot: {}", String::from_utf8_lossy(&output.stderr));
        }
    };
    let sysroot_path = normalize_path(Path::new(&sysroot_path).to_path_buf());

    // Include the relevant sbgECom paths so bindgen can find the headers
    let common_include_path = normalize_path(Path::new("sbgECom").join("common").to_path_buf());
    let src_include_path = normalize_path(Path::new("sbgECom").join("src").to_path_buf());

    bindgen::Builder::default()
        // Specify the header file to generate bindings for
        .header(BINDGEN_HEADER)
        .clang_args(&[
            &format!("--sysroot={}", sysroot_path),
            &format!("-I{}", common_include_path),
            &format!("-I{}", src_include_path),
        ])
        // Use core rather than std as we are compiling for a no_std environment
        .use_core()
        // Register cargo callbacks to rebuild bindings when the header changes
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Generate the bindings
        .generate()
            .expect("Unable to generate bindings")
        // Write the bindings to src/bindings.rs
        .write_to_file(Path::new("src").join("bindings.rs"))
            .expect("Couldn't write bindings!");
}

/// Normalizes a path and returns it as a String.
/// Equivalent to [`PathBuf::canonicalize`] in most instances.
/// On Windows, it will additionally remove any \\?\ prefixes.
fn normalize_path(p: PathBuf) -> String {
    const VERBATIM_PREFIX: &str = r#"\\?\"#;
    let p = p.canonicalize()
             .unwrap()
             .display()
             .to_string();

    if cfg!(windows) && p.starts_with(VERBATIM_PREFIX) {
        p[VERBATIM_PREFIX.len()..].to_string()
    } else {
        p
    }
}
