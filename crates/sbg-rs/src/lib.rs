#![no_std]

// Disable specific format checks as C bindings may not conform.
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

#[allow(unused_imports)] // Auto-generated bindings import unused code.
mod bindings;
mod data_conversion;
#[allow(static_mut_refs)] // Supress warnings as these are safe in our context.
mod sbg;

// Expose the Rust API wrapper with `pub use` not `pub mod` to hide the
// implementation details and simply expose the API as part of sbg-rs.
pub use sbg::*;
