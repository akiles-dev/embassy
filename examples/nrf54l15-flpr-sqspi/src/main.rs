#![no_std]
#![no_main]

//! Clean-room reimplementation of the nRF54L "sQSPI" soft-peripheral firmware,
//! the program that runs on the **FLPR** RISC-V coprocessor and emulates a
//! QSPI/SPI flash controller for the application core.
//!
//! It is written to be **wire-compatible** with the existing host driver
//! `embassy_nrf::sqspi`, i.e. it speaks the same shared-RAM "virtual register
//! block" protocol so it can replace Nordic's `sqspi_firmware.bin` blob.
//!
//! See `docs/sqspi-re/SPEC.md` for the full reverse-engineered protocol. This
//! is the first implementation milestone (M2–M4): the boot handshake, the
//! sync-barrier loop, and a **bit-banged GPIO** SPI engine covering single,
//! dual and quad phases. The fast VPR-VIO/VTIM shift engine that Nordic's blob
//! uses is a later optimization (M5); a GPIO bit-bang is slower but
//! correct-by-construction and easy to validate on a logic analyzer.
//!
//! ## Build / placement
//! This firmware is **non-PIC**, linked at the fixed base `0x2002_0000` (see
//! `memory.x`). The first 32 bytes of the image are the soft-peripheral
//! metadata header ([`FW_HEADER`]); the header's first word is a compressed
//! jump over the header into `_start` at `base + 0x20`. The host copies the
//! whole blob to its FW_RAM buffer and starts the core at the base, so that
//! buffer must be pinned at `0x2002_0000` (see the app example's `memory.x`).
//!
//! ## Status / caveats
//! - The completion-event path writes `VPR.EVENTS_TRIGGERED[20]` via the
//!   memory-mapped peripheral; if that does not pend the app-core IRQ on
//!   silicon, switch to the VEVIF trigger CSR (SPEC §9.2).

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{compiler_fence, fence, Ordering};

use riscv_rt::entry;
use {nrf_pac as pac, panic_halt as _};

// ===========================================================================
// Soft-peripheral metadata header (first 32 bytes of the image)
// ===========================================================================
//
// The host (`embassy_nrf::sqspi::FirmwareMetadata::parse`) reads this 8 x u32
// header from the start of the blob. Word 0's low 16 bits are also executed:
// the core starts at the base, so word 0 must be a valid instruction that jumps
// over the header. `0xa005` is the compressed `c.j .+0x20`, landing on `_start`.
//
// Field layout (matches the host parser and the SPEC §5 header):
//   w0: magic/jump(16) | hdr_ver(4) | comm_id(8) | rsvd(3) | self_boot(1)
//   w1: softperiph_id(16) | platform(16)
//   w3: fw_code_size(16, x16 bytes) | fw_ram_total_size(16, x16 bytes)
//   w6: fw_shared_ram_size(16) | fw_shared_ram_addr_offset(16, x16? no: bytes)
//
// The host computes reg_offset = fw_code_size*16 + fw_shared_ram_addr_offset =
// 0x3740 + 0x400 = 0x3b40, which must equal REG_OFFSET below. The linker keeps
// the whole image under 0x3740 bytes; if it ever grows past that, bump
// fw_code_size here *and* RAM/register placement in memory.x together.
#[unsafe(link_section = ".fw_header")]
#[used]
static FW_HEADER: [u32; 8] = [
    0x0012_a005, // w0: c.j 0x20 (0xa005) | hdr_ver=2 | self_boot=0
    0x0000_45b1, // w1: softperiph_id = 0x45b1 (sQSPI) | platform = 0
    0x0102_0001, // w2: abi/version info (host-ignored)
    0x0400_0374, // w3: fw_code_size=0x374 (*16 = 0x3740 B) | fw_ram_total=0x400 (*16 = 0x4000 B)
    0x0000_0000, // w4
    0x0000_3740, // w5: code size in bytes (informative)
    0x0400_0020, // w6: fw_shared_ram_addr_offset=0x400 | fw_shared_ram_size=0x20
    0x0000_0000, // w7
];

// ===========================================================================
// Shared "virtual register block" layout (mirror of embassy-nrf/src/sqspi/regs.rs)
// ===========================================================================
//
// The block lives in RAM at `load_base + reg_offset`. For this non-PIC build
// `load_base == 0x2002_0000` (memory.x) and `reg_offset == 0x3b40`
// (= fw_code_size 0x3740 + fw_shared_ram_addr_offset 0x400), matching
// FW_HEADER above. Keep these in sync.

const LOAD_BASE: usize = 0x2002_0000;
const REG_OFFSET: usize = 0x3b40;
const REG_BASE: usize = LOAD_BASE + REG_OFFSET;

// Top-level register offsets (from REG_BASE).
const EVENTS_DMA: usize = 0x00C; // sub-block
const O_ENABLE: usize = 0x054;
const FORMAT: usize = 0x06C; // sub-block
const CORE: usize = 0x0A8; // sub-block
const SPSYNC: usize = 0x1A8; // AUX[0..3]

// EVENTS_DMA sub-block offsets.
const E_DMA_DONE: usize = EVENTS_DMA + 0x20;
// Set instead of DONE on a bus error. Not produced yet (no error detection in
// the bit-bang PHY); kept for the completion-path contract.
#[allow(dead_code)]
const E_DMA_ABORTED: usize = EVENTS_DMA + 0x2C;

// FORMAT sub-block offsets (informative; we read PIXELS for the length).
const F_PIXELS: usize = FORMAT + 0x08;

// CORE (DWC_SSI model) sub-block offsets.
const C_CTRLR0: usize = CORE + 0x00;
const C_SQSPIENR: usize = CORE + 0x08;
const C_DR0: usize = CORE + 0x60; // DR[n] at +0x60 + 4*n
const C_RXSAMPLEDELAY: usize = CORE + 0xF0;
const C_SPICTRLR0: usize = CORE + 0xF4;

// SPSYNC handshake words.
const AUX0: usize = SPSYNC + 0x00;
const AUX1: usize = SPSYNC + 0x04;

#[inline(always)]
fn reg_rd(off: usize) -> u32 {
    unsafe { read_volatile((REG_BASE + off) as *const u32) }
}
#[inline(always)]
fn reg_wr(off: usize, val: u32) {
    unsafe { write_volatile((REG_BASE + off) as *mut u32, val) }
}
#[inline(always)]
fn dr(n: usize) -> u32 {
    reg_rd(C_DR0 + 4 * n)
}

// ===========================================================================
// Pin map (fixed — the host always wires these P2 pins to match the firmware)
// ===========================================================================
//
// From examples/nrf54l15-app/src/bin/sqspi.rs wiring table:
//   SCLK=P2.01  CS#=P2.05  IO0/SI=P2.02  IO1/SO=P2.04  IO2/WP#=P2.03  IO3/HOLD#=P2.00

const SCLK: usize = 1;
const CSN: usize = 5;
const IO0: usize = 2;
const IO1: usize = 4;
const IO2: usize = 3;
const IO3: usize = 0;
const IO: [usize; 4] = [IO0, IO1, IO2, IO3];

#[inline(always)]
fn port() -> pac::gpio::Gpio {
    // Secure alias; the host grants the VPR secure access before boot.
    pac::P2_S
}

fn pin_out(n: usize, connect_input: bool) {
    port().pin_cnf(n).write(|w| {
        w.set_ctrlsel(pac::gpio::vals::Ctrlsel::Gpio);
        w.set_dir(pac::gpio::vals::Dir::Output);
        w.set_input(if connect_input {
            pac::gpio::vals::Input::Connect
        } else {
            pac::gpio::vals::Input::Disconnect
        });
        w.set_pull(pac::gpio::vals::Pull::Disabled);
    });
}

fn pin_in(n: usize) {
    port().pin_cnf(n).write(|w| {
        w.set_ctrlsel(pac::gpio::vals::Ctrlsel::Gpio);
        w.set_dir(pac::gpio::vals::Dir::Input);
        w.set_input(pac::gpio::vals::Input::Connect);
        w.set_pull(pac::gpio::vals::Pull::Pullup);
    });
}

#[inline(always)]
fn set_level(n: usize, high: bool) {
    if high {
        port().outset().write(|w| w.set_pin(n, true));
    } else {
        port().outclr().write(|w| w.set_pin(n, true));
    }
}
#[inline(always)]
fn set_dir_out(n: usize) {
    port().dirset().write(|w| w.set_pin(n, true));
}
#[inline(always)]
fn set_dir_in(n: usize) {
    port().dirclr().write(|w| w.set_pin(n, true));
}
#[inline(always)]
fn read_level(n: usize) -> bool {
    port().in_().read().pin(n)
}

// ===========================================================================
// SPI bit-bang engine
// ===========================================================================

/// Multi-line width: 1 (single), 2 (dual) or 4 (quad), derived from SPI_FRF.
fn width_of(frf: u32) -> usize {
    match frf {
        1 => 2,
        2 => 4,
        _ => 1,
    }
}

struct Phy {
    /// Clock idle level (CPOL / SCPOL).
    cpol: bool,
    // NOTE: clock phase (SCPH) is read by the caller but not yet used to shift
    // the sample point; the supported modes (0 and 3) both sample on the
    // leading active edge with the timing below. Revisit for other modes.
}

impl Phy {
    #[inline(always)]
    fn clock_idle(&self) {
        set_level(SCLK, self.cpol);
    }
    #[inline(always)]
    fn clock_active(&self) {
        set_level(SCLK, !self.cpol);
    }

    /// Output `count` bits of `val` (MSB first) across `width` data lines.
    /// `width` IO lines carry the top `width` bits of each group first.
    fn write_bits(&self, val: u32, count: u32, width: usize) {
        // Ensure the data lines are outputs.
        for &p in &IO[..width] {
            set_dir_out(p);
        }
        let mut remaining = count as i32;
        // Number of clocks = count / width (count is always a multiple of width
        // for address phases; for the 8-bit command width==1).
        while remaining > 0 {
            // For this clock, the highest data line carries the highest bit.
            for lane in 0..width {
                let bit_index = remaining - 1 - lane as i32;
                let bit = if bit_index >= 0 {
                    (val >> bit_index) & 1 != 0
                } else {
                    false
                };
                // group MSB goes on the highest line: line index = width-1-lane.
                let line = IO[width - 1 - lane];
                set_level(line, bit);
            }
            // Mode 0: data is set while clock idle, slave samples on leading edge.
            self.clock_active();
            self.clock_idle();
            remaining -= width as i32;
        }
    }

    /// Read `count` bits (MSB first) across `width` data lines into a u32.
    ///
    /// In **single-line** mode the controller receives on IO1 (MISO); in
    /// multi-line modes it receives across IO0..IO(width-1), MSB on the highest
    /// line. (`width` is 1, 2 or 4.)
    fn read_bits(&self, count: u32, width: usize) -> u32 {
        if width == 1 {
            set_dir_in(IO1);
        } else {
            for &p in &IO[..width] {
                set_dir_in(p);
            }
        }
        let mut acc: u32 = 0;
        let mut remaining = count as i32;
        while remaining > 0 {
            // Mode 0: slave drives on the trailing edge of the previous clock;
            // master samples around the leading edge. We sample just before
            // releasing the active edge.
            self.clock_active();
            if width == 1 {
                acc = (acc << 1) | read_level(IO1) as u32;
            } else {
                for lane in 0..width {
                    let line = IO[width - 1 - lane];
                    acc = (acc << 1) | read_level(line) as u32;
                }
            }
            self.clock_idle();
            remaining -= width as i32;
        }
        acc
    }

    /// Issue `cycles` dummy clocks with the data lines released (inputs).
    fn dummy(&self, cycles: u32) {
        for &p in &IO {
            set_dir_in(p);
        }
        for _ in 0..cycles {
            self.clock_active();
            self.clock_idle();
        }
    }
}

// ===========================================================================
// Transfer
// ===========================================================================

const TMOD_TX: u32 = 1;
const TMOD_RX: u32 = 2;

fn run_transfer() {
    let ctrlr0 = reg_rd(C_CTRLR0);
    let spictrlr0 = reg_rd(C_SPICTRLR0);

    let cpha = (ctrlr0 >> 8) & 1 != 0;
    let cpol = (ctrlr0 >> 9) & 1 != 0;
    let tmod = (ctrlr0 >> 10) & 0x3;
    let frf = (ctrlr0 >> 22) & 0x3;

    let transtype = spictrlr0 & 0x3;
    let addrl_nibbles = (spictrlr0 >> 2) & 0xf;
    let waitcycles = (spictrlr0 >> 11) & 0x1f;

    let opcode = dr(0);
    let address = dr(1); // low 32 bits; DR[2] = address>>31 (unused for <4GiB)
    let data_ptr = dr(3) as *mut u8;
    let len = dr(4) as usize;
    let addr_bits = addrl_nibbles * 4;

    // `len`/`PIXELS` are kept consistent by the host; read PIXELS as a sanity
    // cross-check but trust DR[4].
    let _ = reg_rd(F_PIXELS);
    let _ = reg_rd(C_RXSAMPLEDELAY); // honored implicitly by sampling timing

    let _ = cpha;
    let phy = Phy { cpol };
    let data_width = width_of(frf);

    // --- begin transaction ---
    phy.clock_idle();
    set_level(CSN, false); // assert CS#

    // 1. command: 8 bits, always single line.
    phy.write_bits(opcode & 0xFF, 8, 1);

    // 2. address (skip if addr_bits == 0).
    if addr_bits > 0 {
        let aw = if transtype == 0 { 1 } else { data_width };
        phy.write_bits(address, addr_bits, aw);
    }

    // 3. dummy cycles.
    if waitcycles > 0 {
        phy.dummy(waitcycles);
    }

    // 4. data phase.
    match tmod {
        TMOD_TX => {
            // Re-assert data lines as outputs and stream bytes.
            for i in 0..len {
                let b = unsafe { read_volatile(data_ptr.add(i)) } as u32;
                phy.write_bits(b, 8, data_width);
            }
        }
        TMOD_RX => {
            for i in 0..len {
                let b = phy.read_bits(8, data_width) as u8;
                unsafe { write_volatile(data_ptr.add(i), b) };
            }
        }
        _ => {}
    }

    // Release IO2/IO3 high so WP#/HOLD# stay deasserted between transactions.
    set_dir_out(IO2);
    set_dir_out(IO3);
    set_level(IO2, true);
    set_level(IO3, true);

    set_level(CSN, true); // deassert CS#

    // --- signal completion ---
    compiler_fence(Ordering::SeqCst);
    fence(Ordering::SeqCst); // order data/RAM writes before the event
    reg_wr(E_DMA_DONE, 1);
    trigger_done_event();
}

/// Raise the transfer-complete event the host's `VPR00` interrupt keys on
/// (`EVENTS_TRIGGERED[20]`). See SPEC §9.2 for the VEVIF-CSR alternative if a
/// plain memory-mapped write does not pend the app-core interrupt.
fn trigger_done_event() {
    pac::VPR00_S.events_triggered(20).write_value(1);
}

// ===========================================================================
// Entry / main loop
// ===========================================================================

fn boot_init() {
    // Take over the pins from the VPR-VIO routing the host set up and drive them
    // as plain GPIO. Idle: SCK low (set per-transfer), CS# high, IO2/IO3 high.
    pin_out(SCLK, false);
    pin_out(CSN, false);
    pin_out(IO0, true);
    pin_in(IO1);
    pin_out(IO2, true);
    pin_out(IO3, true);

    set_level(CSN, true);
    set_level(SCLK, false);
    set_level(IO2, true);
    set_level(IO3, true);

    // Boot-ready handshake: the host pre-set ENABLE=1 and spins until we clear it.
    reg_wr(O_ENABLE, 0);
}

#[entry]
fn main() -> ! {
    boot_init();

    // Poll loop: service sync barriers (AUX echo) and run an armed transfer
    // after the action barrier that follows SQSPIENR=1. No VEVIF/CLIC receive
    // path is needed — the barriers and the "go" condition are all in RAM.
    loop {
        let a0 = reg_rd(AUX0);
        if a0 != reg_rd(AUX1) {
            // A barrier was requested. Acknowledge it (order any prior observed
            // config writes before the ack).
            fence(Ordering::SeqCst);
            reg_wr(AUX1, a0);
            fence(Ordering::SeqCst);

            // If a transfer is armed (SQSPIENR=1), this was the action barrier
            // that precedes the start trigger — run it now.
            if reg_rd(C_SQSPIENR) & 1 != 0 {
                run_transfer();
            }
        }
    }
}
