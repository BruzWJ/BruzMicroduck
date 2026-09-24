//! Compile the vendored VL53L5CX Ultra Lite Driver into the crate.
//!
//! No autotools, system library or Python: `cc` picks the target compiler up
//! from the environment, which is what makes `cargo board` cross-compile it.
//! The ULD is upstream code, so its warnings are deliberately not errors.
//!
//! `vendor/platform.c` uses Linux `i2c-dev`; off Linux no C is built and the
//! Rust wrapper refuses to open hardware. `tofd --fake` remains available on a
//! development machine.

fn main() {
    // A build script runs on the host, so ask Cargo about the target rather than
    // using `cfg!(target_os)` here.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("linux") {
        println!("cargo::rerun-if-changed=build.rs");
        return;
    }

    let vendor = std::path::Path::new("vendor");
    let driver = vendor.join("vl53l5cx");
    cc::Build::new()
        .include(&driver)
        .define("TOF_PLATFORM", "VL53L5CX_Platform")
        .file(driver.join("vl53l5cx_api.c"))
        .file(driver.join("shim.c"))
        .file(vendor.join("platform.c"))
        .warnings(false)
        .opt_level(2)
        .compile("vl53l5cx");

    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rerun-if-changed=vendor/platform.c");
    // The firmware blob is a large header no source file lists explicitly, so
    // name every input or a driver update would not trigger a rebuild.
    for file in ["api.c", "api.h", "buffers.h"] {
        println!("cargo::rerun-if-changed=vendor/vl53l5cx/vl53l5cx_{file}");
    }
    println!("cargo::rerun-if-changed=vendor/vl53l5cx/platform.h");
    println!("cargo::rerun-if-changed=vendor/vl53l5cx/shim.c");
}
