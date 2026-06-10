#![no_std]
#![no_main]

//! Bring-up example for the Nordic **sQSPI** soft peripheral on the nRF54L15-DK.
//!
//! sQSPI is a QSPI/SPI controller implemented in firmware running on the FLPR
//! RISC-V coprocessor (VPR00). The application core and the FLPR talk through a
//! "virtual register interface" placed in shared RAM that the firmware emulates
//! as if it were a real QSPI peripheral. This example is a faithful, hand-rolled
//! port of the host side of Nordic's C driver
//! (`nrfxlib/softperipheral/sQSPI/src/nrf_sqspi.c`), specialised to exactly the
//! operations the nRF52840 `qspi.rs` example performs: read JEDEC id, read the
//! status register, erase, program and read back 8 pages, then verify.
//!
//! This first bring-up uses **single-line SPI at ~8 MHz** (read 0x03,
//! page-program 0x02, sector-erase 0x20). Once this is proven, the `xfer()`
//! helper already takes the line-mode/dummy parameters needed to move to quad
//! (0xEB / 0x38) and higher clocks.
//!
//! ## Wiring (external MX25R64, e.g. the chip on the nRF52840-DK)
//! The nRF54L15-DK has no onboard QSPI flash and the FLPR can only reach port
//! P2, so wire an MX25R64 to (per the Nordic sQSPI porting guide):
//!
//! | Flash pin        | nRF54L15 |
//! |------------------|----------|
//! | SCLK             | P2.01    |
//! | SI  / IO0        | P2.02    |
//! | SO  / IO1        | P2.04    |
//! | WP# / IO2        | P2.03    |
//! | HOLD#/ IO3       | P2.00    |
//! | CS#              | P2.05    |
//! | VCC = 3V3, GND = GND                |
//!
//! ## Firmware blob
//! `sqspi_firmware.bin` next to this file is the FLPR firmware (v1.2.1),
//! extracted verbatim from
//! `nrfxlib/softperipheral/sQSPI/include/nrf54l/sqspi_firmware_v1.2.1.h`
//! (the C `sqspi_firmware_bin[]` array, 12088 bytes).

use defmt::{assert_eq, info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pac;
use embassy_nrf::pac::gpio::vals::{Ctrlsel, Dir, Drive, Input, Pull};
use embassy_nrf::vpr::Vpr;
use {defmt_rtt as _, panic_probe as _};

// ---------------------------------------------------------------------------
// Memory map (nRF54L15, from the sQSPI porting guide). The top 128K of RAM is
// already reserved by this example's memory.x (RAM length = 256K - 128K), so
// none of this collides with the application.
// ---------------------------------------------------------------------------

/// Where the FLPR firmware is copied to and executed from (INITPC).
const SP_FIRMWARE_ADDR: u32 = 0x2003_C000;
/// Size of the firmware code region (`fw_code_size << 4` from the blob header).
/// The blob itself is shorter (12088 bytes); the tail is `.bss` and is zeroed.
const FW_CODE_REGION: usize = 0x3740;
/// Base of the virtual register interface (`p_reg`):
/// `SP_FIRMWARE_ADDR + 0x3740 (exec RAM) + 0x400 (shared) = +0x3B40`.
const P_REG: usize = 0x2003_FB40;

/// FLPR firmware (`self_boot == 0`, so the host copies it into RAM).
static FW: &[u8] = include_bytes!("sqspi_firmware.bin");

// ---------------------------------------------------------------------------
// VPR (FLPR) task/event indices for the nRF54L series (softperipheral_regif.h).
// ---------------------------------------------------------------------------
const TASK_START: usize = 16; // TASKS_TRIGGER[16] - start the prepared transfer
const TASK_CONFIG: usize = 17; // config sync barrier (__CSB)
const TASK_ACTION: usize = 18; // action sync barrier (__ASB)
#[allow(dead_code)]
const TASK_STOP: usize = 19; // stop sync barrier (__SSB)
const EVENT_DONE: usize = 20; // EVENTS_TRIGGERED[20] - peripheral -> host notify
const VPR_BASE_FREQ_HZ: u32 = 128_000_000;

// ---------------------------------------------------------------------------
// Offsets into the virtual register block (NRF_SP_QSPI_Type), obtained by
// compiling nrf_sp_qspi.h and printing offsetof() for each field used here.
// ---------------------------------------------------------------------------
const INTEN: usize = 0x44;
const EV_DMA_DONEJOB: usize = 0x1c;
const EV_DMA_DONE: usize = 0x2c;
const EV_DMA_ABORTED: usize = 0x38;
const ENABLE: usize = 0x54;
const FORMAT_DFS: usize = 0x6c;
const FORMAT_BPP: usize = 0x70;
const FORMAT_PIXELS: usize = 0x74;
const FORMAT_CILEN: usize = 0x78;
const FORMAT_BITORDER: usize = 0x7c;
const CORE_CTRLR1: usize = 0xac;
const CORE_SQSPIENR: usize = 0xb0;
const CORE_BAUDR: usize = 0xbc;
const CORE_RXSAMPLEDELAY: usize = 0x198;
const CORE_SPICTRLR0: usize = 0x19c;

#[inline]
const fn dr(n: usize) -> usize {
    0x108 + 4 * n // CORE.CORE.DR[n]
}
#[inline]
const fn aux(n: usize) -> usize {
    0x1a8 + 4 * n // SPSYNC.AUX[n]
}

#[inline]
unsafe fn reg_rd(off: usize) -> u32 {
    unsafe { core::ptr::read_volatile((P_REG + off) as *const u32) }
}
#[inline]
unsafe fn reg_wr(off: usize, val: u32) {
    unsafe { core::ptr::write_volatile((P_REG + off) as *mut u32, val) }
}

// Transfer direction (CTRLR0.TMOD).
const DIR_TX: u32 = 1;
const DIR_RX: u32 = 2;
// SPI frame format (CTRLR0.SPIFRF).
const FRF_STD: u32 = 0; // single line
#[allow(dead_code)]
const FRF_QUAD: u32 = 2;
// Address/data layout (SPICTRLR0.TRANSTYPE).
const TT0: u32 = 0; // cmd/addr on the command lines
#[allow(dead_code)]
const TT1: u32 = 1; // addr on the data lines (e.g. 1-4-4)

/// Aligned 4-byte scratch buffer for small DMA reads (id, status register).
#[repr(C, align(4))]
struct Scratch([u8; 4]);

/// Host-side handle to the sQSPI soft peripheral.
struct Sqspi {
    /// Monotonic counter used by the sync-barrier handshake (`m_task_count`).
    task_count: u32,
}

impl Sqspi {
    /// Sync barrier: write the counter to AUX[0], trigger `task`, and spin until
    /// the firmware echoes it into AUX[1]. Mirrors the `__XSBx` macro.
    fn xsb(&mut self, task: usize) {
        unsafe {
            reg_wr(aux(0), self.task_count);
            pac::VPR00.tasks_trigger(task).write_value(1);
            while reg_rd(aux(0)) != reg_rd(aux(1)) {
                cortex_m::asm::nop();
            }
        }
        self.task_count = self.task_count.wrapping_add(1);
    }

    /// Issue one SPI transaction and block until it completes. `data_ptr` points
    /// into RAM the FLPR DMAs to/from (must be 4-byte aligned for real data).
    #[allow(clippy::too_many_arguments)]
    fn xfer(
        &mut self,
        cmd: u32,
        cmd_bits: u32,
        address: u64,
        addr_bits: u32,
        dummy_cycles: u32,
        data_ptr: u32,
        data_len: usize,
        dir: u32,
        spifrf: u32,
        transtype: u32,
    ) -> Result<(), &'static str> {
        unsafe {
            // SPICTRLR0: address length (in nibbles), instruction length, dummy.
            let addrl = (addr_bits / 4) & 0xF;
            let instl = match cmd_bits {
                4 => 1,
                8 => 2,
                16 => 3,
                _ => 0,
            };
            let waitcycles = dummy_cycles & 0x1F;
            let spictrlr0 = (transtype & 0x3) | (addrl << 2) | (instl << 8) | (waitcycles << 11);

            // With FORMAT.DFS=7 (8-bit frames) and FORMAT.BPP=8, both pixels and
            // NDF equal the byte count.
            let ndf = data_len as u32;
            reg_wr(FORMAT_PIXELS, data_len as u32);
            reg_wr(CORE_CTRLR1, ndf & 0xFFFF);
            reg_wr(dr(23), data_len as u32); // firmware-private "data byte count"
            reg_wr(FORMAT_CILEN, (cmd_bits + 31) / 32);

            // CTRLR0: controller, SPI frame format, mode 0, 8-bit frames.
            let ctrlr0 = (1 << 31)      // SQSPIISMST = controller
                | 0x7                   // DFS = 8-bit
                | (0x7 << 16)           // CFS = 8-bit
                | ((dir & 0x3) << 10)   // TMOD
                | ((spifrf & 0x3) << 22); // SPIFRF
            reg_wr(0xa8, ctrlr0); // CORE.CORE.CTRLR0
            reg_wr(CORE_SPICTRLR0, spictrlr0);

            // Command / address / DMA descriptor.
            reg_wr(dr(0), cmd);
            reg_wr(dr(1), (address & 0xFFFF_FFFF) as u32);
            reg_wr(dr(2), ((address >> 31) & 0xFFFF_FFFF) as u32);
            reg_wr(dr(3), data_ptr);
            reg_wr(dr(4), data_len as u32);

            self.xsb(TASK_CONFIG); // __CSB - latch the configuration

            reg_wr(CORE_SQSPIENR, 1); // enable the QSPI core
            self.xsb(TASK_ACTION); // __ASB

            // Kick off the transfer.
            pac::VPR00.tasks_trigger(TASK_START).write_value(1);

            // Poll for completion (the firmware sets EVENTS_DMA.DONE in shared RAM).
            let mut timeout = 20_000_000u32;
            loop {
                if reg_rd(EV_DMA_DONE) != 0 {
                    reg_wr(EV_DMA_DONE, 0);
                    break;
                }
                if reg_rd(EV_DMA_ABORTED) != 0 {
                    reg_wr(EV_DMA_ABORTED, 0);
                    reg_wr(CORE_SQSPIENR, 0);
                    self.xsb(TASK_ACTION);
                    return Err("transfer aborted");
                }
                timeout -= 1;
                if timeout == 0 {
                    return Err("transfer timeout");
                }
            }
            pac::VPR00.events_triggered(EVENT_DONE).write_value(0);

            reg_wr(CORE_SQSPIENR, 0); // disable the core
            self.xsb(TASK_ACTION); // __ASB
        }
        Ok(())
    }

    // ---- Single-line flash primitives (MX25R64) --------------------------

    /// Command with neither address nor data (e.g. write-enable).
    fn command(&mut self, op: u32) -> Result<(), &'static str> {
        self.xfer(op, 8, 0, 0, 0, 0, 0, DIR_TX, FRF_STD, TT0)
    }

    fn write_enable(&mut self) -> Result<(), &'static str> {
        self.command(0x06)
    }

    fn read_id(&mut self) -> Result<[u8; 3], &'static str> {
        let mut buf = Scratch([0; 4]);
        self.xfer(0x9F, 8, 0, 0, 0, buf.0.as_mut_ptr() as u32, 3, DIR_RX, FRF_STD, TT0)?;
        Ok([buf.0[0], buf.0[1], buf.0[2]])
    }

    fn read_status(&mut self) -> Result<u8, &'static str> {
        let mut buf = Scratch([0; 4]);
        self.xfer(0x05, 8, 0, 0, 0, buf.0.as_mut_ptr() as u32, 1, DIR_RX, FRF_STD, TT0)?;
        Ok(buf.0[0])
    }

    /// Write the status register (opcode 0x01), e.g. to clear block protection.
    fn write_status(&mut self, val: u8) -> Result<(), &'static str> {
        self.write_enable()?;
        let buf = Scratch([val, 0, 0, 0]);
        self.xfer(0x01, 8, 0, 0, 0, buf.0.as_ptr() as u32, 1, DIR_TX, FRF_STD, TT0)?;
        self.wait_wip()
    }

    /// Spin on RDSR until the write-in-progress (WIP) bit clears.
    fn wait_wip(&mut self) -> Result<(), &'static str> {
        while self.read_status()? & 0x01 != 0 {}
        Ok(())
    }

    /// Erase one 4 KiB sector (opcode 0x20).
    fn erase_sector(&mut self, address: u32) -> Result<(), &'static str> {
        self.write_enable()?;
        self.xfer(0x20, 8, address as u64, 24, 0, 0, 0, DIR_TX, FRF_STD, TT0)?;
        self.wait_wip()
    }

    /// Program up to 256 bytes within a single flash page (opcode 0x02).
    fn page_program(&mut self, address: u32, data: &[u8]) -> Result<(), &'static str> {
        self.write_enable()?;
        self.xfer(
            0x02,
            8,
            address as u64,
            24,
            0,
            data.as_ptr() as u32,
            data.len(),
            DIR_TX,
            FRF_STD,
            TT0,
        )?;
        self.wait_wip()
    }

    /// Read `data.len()` bytes (opcode 0x03, no dummy cycles).
    fn read(&mut self, address: u32, data: &mut [u8]) -> Result<(), &'static str> {
        self.xfer(
            0x03,
            8,
            address as u64,
            24,
            0,
            data.as_mut_ptr() as u32,
            data.len(),
            DIR_RX,
            FRF_STD,
            TT0,
        )
    }

    /// Write an arbitrary span, splitting into ≤256-byte page-program bursts.
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), &'static str> {
        let mut addr = address;
        let mut rest = data;
        while !rest.is_empty() {
            // Don't cross a 256-byte page boundary in a single program command.
            let page_room = 256 - (addr as usize % 256);
            let n = core::cmp::min(page_room, rest.len());
            self.page_program(addr, &rest[..n])?;
            addr += n as u32;
            rest = &rest[n..];
        }
        Ok(())
    }
}

const PAGE_SIZE: usize = 4096;

#[repr(C, align(4))]
struct AlignedBuf([u8; PAGE_SIZE]);

/// Configure one P2 pin and route it to the FLPR via CTRLSEL = VPR.
fn cfg_pin(pin: usize, input: Input, pull: Pull) {
    pac::P2.pin_cnf(pin).write(|w| {
        w.set_dir(Dir::Output);
        w.set_input(input);
        w.set_pull(pull);
        w.set_drive0(Drive::H);
        w.set_drive1(Drive::H);
        w.set_ctrlsel(Ctrlsel::Vpr);
    });
}

/// Stop and reset the FLPR (VPR00). A previously-started FLPR is NOT cleared by
/// the debugger's soft reset (SYSRESETREQ), and a running FLPR stalls
/// `embassy_nrf::init`. Do this before anything else so re-runs recover cleanly.
fn flpr_stop_reset() {
    use pac::spu::vals::Dmasec;
    use pac::vpr::vals::CpurunEn;
    // VPR00 resets to non-secure; mark it secure first or the secure-alias
    // register writes below bus-fault. (Vpr::new does this too, but too late
    // for us: a leftover FLPR must be stopped before embassy_nrf::init.)
    pac::SPU00.periph(12).perm().write(|w| {
        w.set_secattr(true);
        w.set_dmasec(Dmasec::Secure);
    });
    pac::VPR00.cpurun().write(|w| w.set_en(CpurunEn::Stopped));
    // Pulse the RISC-V debug-module non-debug reset (matches the C uninit path).
    pac::VPR00.debugif().dmcontrol().write(|w| {
        w.set_ndmreset(true);
        w.set_dmactive(true);
    });
    pac::VPR00.debugif().dmcontrol().write(|w| {
        w.set_ndmreset(false);
        w.set_dmactive(false);
    });
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("sqspi example: boot");
    flpr_stop_reset();
    let p = embassy_nrf::init(Default::default());
    info!("sqspi example: embassy init done");

    // --- Boot the FLPR firmware -------------------------------------------
    // Vpr::new marks the FLPR secure in the SPU and sets INITPC.
    let mut vpr = unwrap!(Vpr::new(p.VPR, SP_FIRMWARE_ADDR as *const u8));
    info!("sqspi example: vpr created");

    unsafe {
        // Zero the virtual register block, then arm the boot handshake: the
        // host sets ENABLE=1 and the firmware clears it once it is ready.
        core::ptr::write_bytes(P_REG as *mut u8, 0, 0x1B8);
        reg_wr(ENABLE, 1);
    }

    // Copy firmware to RAM and zero-fill the .bss tail of the code region.
    unwrap!(vpr.load(FW));
    unsafe {
        let tail = SP_FIRMWARE_ADDR as usize + FW.len();
        core::ptr::write_bytes(tail as *mut u8, 0, FW_CODE_REGION - FW.len());
    }

    info!("starting FLPR sQSPI firmware at {:#010x}", SP_FIRMWARE_ADDR);
    vpr.start();

    // Wait for the firmware to signal readiness by clearing ENABLE.
    while unsafe { reg_rd(ENABLE) } != 0 {}
    info!("FLPR firmware ready");

    // --- Pin configuration (port P2) --------------------------------------
    // Single-line SPI only needs SCK / IO0 (MOSI) / IO1 (MISO) / CSN routed to
    // the FLPR. IO2 (WP#) and IO3 (HOLD#) are not driven by the firmware in
    // single-line mode, so hold them HIGH as plain GPIO to keep write-protect
    // and hold deasserted. When moving to quad they instead become FLPR-routed
    // data lines (use cfg_pin for them too).
    cfg_pin(1, Input::Disconnect, Pull::Disabled); // SCK  = P2.01
    cfg_pin(2, Input::Connect, Pull::Pullup); // IO0  = P2.02
    cfg_pin(4, Input::Connect, Pull::Pullup); // IO1  = P2.04
    cfg_pin(5, Input::Disconnect, Pull::Disabled); // CSN  = P2.05
    let _io2 = Output::new(p.P2_03, Level::High, OutputDrive::Standard); // WP#
    let _io3 = Output::new(p.P2_00, Level::High, OutputDrive::Standard); // HOLD#

    // --- Default data format (8-bit frames, MSB first), matches the C init.
    unsafe {
        reg_wr(FORMAT_DFS, 7);
        reg_wr(dr(22), 32); // firmware-private "32 - padding"
        reg_wr(FORMAT_BPP, 8);
        reg_wr(FORMAT_BITORDER, 0);
        reg_wr(INTEN, (1 << 9) | (1 << 12) | (1 << 5)); // DMADONE | DMAABORTED | DMADONEJOB

        // Baud rate: SCKDV field = base_freq / sck.
        let sck_hz: u32 = 8_000_000;
        let clkdiv = VPR_BASE_FREQ_HZ / sck_hz;
        reg_wr(CORE_BAUDR, (clkdiv << 1) & 0xFFFE);

        // RX sample delay. Must be > 0 for normal (non-high-speed) transfers
        // per the sQSPI limitations doc; with 0 the sampled RX data is
        // unreliable. The Zephyr mspi_sqspi driver always uses 1.
        reg_wr(CORE_RXSAMPLEDELAY, 1);

        // Activate the emulated peripheral.
        reg_wr(ENABLE, 1);
    }
    let mut q = Sqspi { task_count: 1 };
    q.xsb(TASK_ACTION); // __ASB after enable
    unsafe {
        reg_wr(EV_DMA_DONE, 0);
        reg_wr(EV_DMA_ABORTED, 0);
        reg_wr(EV_DMA_DONEJOB, 0);
    }

    // --- Same operations as the nRF52840 qspi.rs example ------------------
    let id = unwrap!(q.read_id());
    info!("id: {}", id);

    let status = unwrap!(q.read_status());
    info!("status: {:?}", status);

    // Diagnostic: WREN must latch the write-enable (WEL) bit, or program/erase
    // commands are silently ignored by the flash.
    unwrap!(q.write_enable());
    let st = unwrap!(q.read_status());
    info!("status after WREN: {=u8:#04x} (WEL={=u8})", st, (st >> 1) & 1);

    // Clear the block-protection bits if any are set (they too make
    // program/erase silently fail). Keep QE set, like the nRF52840 example.
    if st & 0x3C != 0 {
        info!("clearing block protection...");
        unwrap!(q.write_status(0x40));
        info!("status now: {=u8:#04x}", unwrap!(q.read_status()));
    }

    let mut buf = AlignedBuf([0u8; PAGE_SIZE]);
    let pattern = |a: u32| (a ^ (a >> 8) ^ (a >> 16) ^ (a >> 24)) as u8;

    for i in 0..8 {
        info!("page {:?}: erasing...", i);
        unwrap!(q.erase_sector(i * PAGE_SIZE as u32));

        for j in 0..PAGE_SIZE {
            buf.0[j] = pattern(j as u32 + i * PAGE_SIZE as u32);
        }

        info!("programming...");
        unwrap!(q.write(i * PAGE_SIZE as u32, &buf.0));
    }

    for i in 0..8 {
        info!("page {:?}: reading...", i);
        unwrap!(q.read(i * PAGE_SIZE as u32, &mut buf.0));

        info!("verifying...");
        for j in 0..PAGE_SIZE {
            assert_eq!(buf.0[j], pattern(j as u32 + i * PAGE_SIZE as u32));
        }
    }

    info!("done!");

    loop {
        cortex_m::asm::wfi();
    }
}
