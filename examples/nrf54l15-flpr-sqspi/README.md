# nRF54L15 sQSPI FLPR firmware (clean-room reimplementation)

This crate builds a **drop-in replacement** for Nordic's precompiled
`sqspi_firmware.bin` — the firmware that runs on the nRF54L15's FLPR RISC-V
coprocessor and emulates a QSPI/SPI flash controller. It is wire-compatible
with the `embassy_nrf::sqspi` host driver (same shared-RAM register protocol),
written without transcribing Nordic's licensed blob. See
`docs/sqspi-re/SPEC.md` for the reverse-engineered protocol.

It lives in its own crate (not as a bin in `nrf54l15-flpr`) because it needs a
different linker script: the first 32 bytes of the image are the
soft-peripheral metadata header, so `_stext` is shifted to `base + 0x20`. Doing
that in the shared `nrf54l15-flpr/memory.x` would also move `blinky`'s entry.

> **Status:** not yet validated on hardware. The protocol logic and the linker
> placement are complete; the bit-bang PHY timing and the completion-event
> mechanism are the things most likely to need a tweak during bring-up.

## Requirements

- Nightly Rust (the `riscv32emc-unknown-none-elf` target is built with
  `build-std`; configured in `.cargo/config.toml`).
- `rust-objcopy` (`cargo install cargo-binutils` + `rustup component add
  llvm-tools`), or any `llvm-objcopy`.
- A `probe-rs`-supported debugger and an nRF54L15-DK.

## 1. Build the firmware and produce the loadable binary

```sh
cd examples/nrf54l15-flpr-sqspi
cargo +nightly build --release
rust-objcopy -O binary \
  target/riscv32emc-unknown-none-elf/release/sqspi \
  sqspi_fw.bin
```

`sqspi_fw.bin` already starts with the 32-byte header (it is the `.fw_header`
section, linked at the base) — no prepend step. The app example `include_bytes!`s
this exact path.

Sanity-check the placement before flashing:

```sh
# .fw_header must be at 0x20020000 and _start (the .text entry) at 0x20020020.
rust-objdump -t target/riscv32emc-unknown-none-elf/release/sqspi | grep -E '_stext|fw_header'
# First bytes of the binary must be 05 a0 (little-endian 0xa005 = c.j .+0x20):
xxd -l 8 sqspi_fw.bin     # expect: 05a0 1200 b145 0000
```

## 2. Build & flash the app-core example

The app core program loads this firmware into the FLPR, boots it, and runs an
end-to-end flash test. It is in the **app** crate, not here:

```sh
cd ../nrf54l15-app
cargo build --release --bin sqspi_reimpl       # includes ../nrf54l15-flpr-sqspi/sqspi_fw.bin
cargo run   --release --bin sqspi_reimpl       # flash + RTT log via probe-rs
```

(`cargo run` uses the `probe-rs run --chip nrf54l15` runner from that crate's
`.cargo/config.toml`.) You do **not** flash this firmware separately — the app
core copies it into the FLPR's RAM at runtime, exactly as it does for Nordic's
blob.

## Expected RTT output (happy path)

```
sqspi (reimpl) example: boot
driver ready (firmware booted: ENABLE handshake completed)
JEDEC id: [0x.., 0x.., 0x..]
status: 0x..
enabling quad mode (QE)...
status now: 0x.. (QE=1)
page 0: erasing... programming...
...
done! reimplemented sQSPI firmware works end-to-end.
```

## Bring-up order (if it doesn't work first try)

Matches `docs/sqspi-re/SPEC.md` §11.4 — bisect from the top:

1. **`driver ready`** prints → the boot handshake works (firmware cleared
   `ENABLE`, so `Sqspi::new` returned). If it hangs *before* this, the firmware
   isn't running at `0x20020000` or the register block offset is wrong.
2. **JEDEC id** is sane → the bit-bang PHY clocks/samples correctly on a single
   line, and the completion event (`EVENTS_TRIGGERED[20]`) pends the app IRQ. A
   hang here with correct wiring points at the completion event (try the VEVIF
   CSR path, SPEC §9.2); garbage id points at the sample point / `sample_delay`.
3. **status / QE** → the WREN+WRSR+WIP-poll path.
4. **erase/program/read verify** → addressing + the quad data phases.
5. Diff the SCLK/IO lines against Nordic's blob on a logic analyzer.

## Memory map

| Region | Address | Notes |
|---|---|---|
| Metadata header | `0x2002_0000` | 32 bytes; word0 = `c.j .+0x20` |
| `_start` / `.text` … | `0x2002_0020` | code, rodata, data, bss, stack (grows down) |
| Shared register block | `0x2002_3b40` | `base + 0x3b40`; owned by the host |
| End of FW_RAM buffer | `0x2002_4000` | `fw_ram_total_size` = `0x4000` |

The app core's `FW_RAM` buffer **must** sit at `0x2002_0000` (it does, via the
`.flpr_ram` section in `nrf54l15-app/memory.x`) because this firmware is not
position-independent. If you ever make the firmware PIC (SPEC milestone M6),
both this constraint and the `_stext` shift can go away.
