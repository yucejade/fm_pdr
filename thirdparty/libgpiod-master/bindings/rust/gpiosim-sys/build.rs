// SPDX-License-Identifier: Apache-2.0 OR BSD-3-Clause
// SPDX-FileCopyrightText: 2022 Linaro Ltd.
// SPDX-FileCopyrightText: 2022 Viresh Kumar <viresh.kumar@linaro.org>

extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn generate_bindings() {
    // Tell cargo to invalidate the built crate whenever following files change
    println!("cargo:rerun-if-changed=../../../tests/gpiosim/gpiosim.h");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header("../../../tests/gpiosim/gpiosim.h")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}

fn main() {
    generate_bindings();

    println!("cargo:rustc-link-lib=kmod");
    println!("cargo:rustc-link-lib=mount");
    println!("cargo:rustc-link-search=./../../tests/gpiosim/.libs/");
    println!("cargo:rustc-link-lib=static=gpiosim");
}
