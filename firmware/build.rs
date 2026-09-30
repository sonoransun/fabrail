//! Build script: puts `memory.x` on the linker search path for cortex-m-rt's `link.x`
//! and adds defmt's linker script only with the `dev` feature (defmt is optional, so an
//! unconditional `-Tdefmt.x` would fail to link a production image).

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    fs::copy("memory.x", out.join("memory.x")).expect("copy memory.x into OUT_DIR");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    if env::var_os("CARGO_FEATURE_DEV").is_some() {
        println!("cargo:rustc-link-arg=-Tdefmt.x");
    }

    // Exactly one rail class (src/main.rs repeats this as a compile_error! so the
    // message also appears in `cargo check` without the build script's output).
    let classes = ["DRAWER", "DOOR", "WALL", "HEAVY"]
        .iter()
        .filter(|c| env::var_os(format!("CARGO_FEATURE_CLASS_{c}")).is_some())
        .count();
    if classes != 1 {
        panic!(
            "fabrail-fw: enable exactly one of the class-drawer / class-door / class-wall / \
             class-heavy features (found {classes}); use --no-default-features for a non-door class"
        );
    }
}
