#![no_std]
#![no_main]

//! Same as `sqspi.rs`, but driving the **clean-room reimplemented** FLPR
//! firmware (`examples/nrf54l15-flpr-sqspi`) instead of Nordic's precompiled
//! `sqspi_firmware.bin` blob.
//!
//! Two differences from `sqspi.rs`, both forced by the reimplemented firmware
//! being **non-PIC** (linked at a fixed `0x2002_0000`):
//!
//! 1. `FW_RAM` is pinned to `0x2002_0000` via the `.flpr_ram` linker section
//!    (see `memory.x`), so the address the host loads the firmware at matches
//!    the address it was linked for.
//! 2. `FW` is `include_bytes!`'d from the sibling firmware crate's build output.
//!    **You must build that firmware first** (see its README), otherwise this
//!    file fails to compile with "couldn't read ... sqspi_fw.bin".
//!
//! ## Wiring (unchanged)
//! | Flash pin | nRF54L15-DK |
//! |-----------|-------------|
//! | SCLK      | P2.01       |
//! | SI / IO0  | P2.02       |
//! | SO / IO1  | P2.04       |
//! | WP# / IO2 | P2.03       |
//! | HOLD#/IO3 | P2.00       |
//! | CS#       | P2.05       |

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::sqspi::{self, Config};
use embassy_nrf::{bind_interrupts, peripherals};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    VPR00 => sqspi::InterruptHandler<peripherals::VPR>;
});

/// Reimplemented FLPR firmware. Built by the `nrf54l15-flpr-sqspi` crate; see
/// its README for the `cargo build` + `rust-objcopy -O binary` recipe that
/// produces this file. The 32-byte header is part of the binary already.
static FW: &[u8] = include_bytes!("../../../nrf54l15-flpr-sqspi/sqspi_fw.bin");

/// Firmware RAM: code, working RAM and the shared register block. **Pinned** to
/// 0x2002_0000 through the `.flpr_ram` section so it matches the firmware's
/// fixed link address. 0x4000 bytes == the `fw_ram_total_size` in the header.
///
/// This is a bare array, not a `ConstStaticCell`: the cell carries a hidden
/// "taken" flag that would offset the actual buffer past the section base (and
/// the host would then 128-align *up*, missing 0x2002_0000 entirely). We need
/// the array itself at offset 0, so we hand out the `&mut` ourselves.
#[unsafe(link_section = ".flpr_ram")]
static mut FW_RAM: [MaybeUninit<u8>; 0x4000] = [MaybeUninit::uninit(); 0x4000];

const PAGE_SIZE: usize = 4096;

#[repr(C, align(4))]
struct AlignedBuf([u8; PAGE_SIZE]);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("sqspi (reimpl) example: boot");
    // `embassy_nrf::init` stops and resets the FLPR by default, so re-runs start
    // from a clean state.
    let p = embassy_nrf::init(Default::default());

    // SAFETY: taken exactly once; no other reference to FW_RAM exists. We go
    // through a raw pointer (addr_of_mut!) to avoid creating a reference to a
    // `static mut` (the edition-2024 `static_mut_refs` rule).
    let ram: &'static mut [MaybeUninit<u8>] = unsafe { &mut *addr_of_mut!(FW_RAM) };
    // Sanity: the firmware is non-PIC and assumes this exact base.
    defmt::assert_eq!(ram.as_ptr() as usize, 0x2002_0000, "FW_RAM must be pinned at 0x20020000");

    let mut config = Config::default();
    config.capacity = 8 * 1024 * 1024;

    let mut q = unwrap!(sqspi::Sqspi::new(
        p.VPR, Irqs, FW, ram, p.P2_01, p.P2_05, p.P2_02, p.P2_04, p.P2_03, p.P2_00, config,
    ));
    info!("driver ready (firmware booted: ENABLE handshake completed)");

    // 1. JEDEC id (single-line Rx) — the first thing to confirm the bit-bang
    //    PHY clocks and samples correctly.
    let mut id = [0; 3];
    unwrap!(q.custom_instruction(0x9F, &[], &mut id).await);
    info!("JEDEC id: {=[u8]:#04x}", id);

    // 2. Status register (single-line Rx).
    let mut status = [0; 1];
    unwrap!(q.custom_instruction(0x05, &[], &mut status).await);
    info!("status: {=u8:#04x}", status[0]);

    // 3. Quad-enable (status bit 6) so the quad data phases below work, and
    //    clear the block-protection bits.
    info!("enabling quad mode (QE)...");
    unwrap!(q.custom_instruction(0x01, &[0x40], &mut []).await);
    unwrap!(q.custom_instruction(0x05, &[], &mut status).await);
    info!("status now: {=u8:#04x} (QE={=u8})", status[0], (status[0] >> 6) & 1);

    // 4. Erase + program + read-back verify, one page at a time.
    let mut buf = AlignedBuf([0u8; PAGE_SIZE]);
    let pattern = |a: u32| (a ^ (a >> 8) ^ (a >> 16) ^ (a >> 24)) as u8;

    for i in 0..8 {
        info!("page {}: erasing...", i);
        unwrap!(q.erase(i * PAGE_SIZE as u32).await);

        for j in 0..PAGE_SIZE {
            buf.0[j] = pattern(j as u32 + i * PAGE_SIZE as u32);
        }
        info!("programming...");
        unwrap!(q.write(i * PAGE_SIZE as u32, &buf.0).await);
    }

    for i in 0..8 {
        info!("page {}: reading...", i);
        unwrap!(q.read(i * PAGE_SIZE as u32, &mut buf.0).await);

        info!("verifying...");
        for j in 0..PAGE_SIZE {
            defmt::assert_eq!(buf.0[j], pattern(j as u32 + i * PAGE_SIZE as u32));
        }
    }

    info!("done! reimplemented sQSPI firmware works end-to-end.");
}
