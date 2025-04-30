# SBG-RS

This library provides Rust bindings for the sbgECom C library.

## Build

The `build.rs` file deals with compiling and linking the sbgECom C library and generating a `src/bindings.rs` file. These bindings are generated for everything included in the `wrapper.h` file.

To build this library, you must have [CMake](https://cmake.org/download/) installed on your system. Additionally, you must have the [GCC ARM Cross-Compiler](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) `(arm-none-eabi)` installed and added to path. The `toolchain.cmake` file specifies that the `arm-none-eabi` binaries should be used to build the library.

For the binding generation stage, rust-bindgen requires that Clang be installed. See the [requirements file](https://github.com/rust-lang/rust-bindgen/blob/main/book/src/requirements.md) for bindgen.

Once everything is setup, `cargo build` will automatically compile the sbgECom library and generate the bindings as necessary. It will also ensure the sbgECom library is linked after compiling the Rust source code.
