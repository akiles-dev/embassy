//! Copies `memory.x` into the linker search path and wires up the link scripts.
//!
//! Order matters: `-Tmemory.x` is passed *before* `-Tlink.x` (riscv-rt's), so
//! the plain `_stext = ...` assignment in `memory.x` takes effect before
//! riscv-rt's `PROVIDE(_stext = ORIGIN(REGION_TEXT))` (a `PROVIDE` is a no-op
//! once the symbol already has a definition). That shift is what reserves the
//! 32-byte soft-peripheral header at the load base.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    // `--nmagic` is required because our memory section addresses are not
    // aligned to 0x10000.
    println!("cargo:rustc-link-arg=--nmagic");

    // memory.x first (defines MEMORY, REGION_ALIAS, _stext and the .fw_header
    // section), then riscv-rt's link.x.
    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=-Tlink.x");
}
