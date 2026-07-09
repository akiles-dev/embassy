/* Linker script for the sQSPI FLPR firmware on the nRF54L15.
 *
 * Memory model (all addresses are global system-bus RAM addresses, the same
 * the application core sees — the FLPR shares main SRAM):
 *
 *   0x20020000  +----------------------------+  <- load base (INITPC); 128-aligned
 *               | 32-byte metadata header    |     .fw_header (c.j _start + fields)
 *   0x20020020  +----------------------------+  <- _stext / _start (riscv-rt entry)
 *               | .text .rodata .data .bss   |
 *               |            ...             |
 *               |             stack (grows v)|
 *   0x20023b40  +----------------------------+  <- shared "virtual register block"
 *               | host<->FLPR registers      |     (owned by the host, base+0x3b40)
 *   0x20024000  +----------------------------+  <- end of the host's FW_RAM buffer
 *
 * The host (`embassy_nrf::sqspi`) loads the firmware blob at the base, sets
 * INITPC = base, and starts the core there. Execution therefore begins at the
 * header's first word, a compressed `c.j` that jumps 32 bytes ahead into
 * `_start`. Everything must stay below the register block at base+0x3b40.
 *
 * IMPORTANT: the application-core example's FW_RAM buffer must be pinned at
 * 0x20020000 so `base` matches this link address (this firmware is *not*
 * position-independent, unlike Nordic's blob). See the app example's memory.x.
 */
MEMORY
{
  /* base .. register block. Stack starts at the top (= register block base) and
   * grows down, so it can never run into the host's registers. */
  RAM : ORIGIN = 0x20020000, LENGTH = 0x3b40

  /* riscv-rt's link.x references REGION_HEAP; give it the spare coprocessor RAM
   * above the FW_RAM buffer. Unused (heap size is 0) but must resolve. */
  REGION_HEAP : ORIGIN = 0x20024000, LENGTH = 0x1b000
}

/* Reserve the 32-byte header: shift the text entry past it. This overrides
 * riscv-rt's `PROVIDE(_stext = ORIGIN(REGION_TEXT))` because memory.x is linked
 * first (see build.rs). */
_stext = ORIGIN(RAM) + 0x20;

REGION_ALIAS("REGION_TEXT", RAM);
REGION_ALIAS("REGION_RODATA", RAM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_STACK", RAM);

/* Place the metadata header at the very base of REGION_TEXT, as the first
 * output section. We insert before riscv-rt's `.text.dummy` (the first section
 * in its link.x) rather than before `.text`: `.text.dummy` advances the
 * location counter to `_stext` (= base+0x20), so inserting before `.text` would
 * force the header to place itself *backwards*. Inserting before `.text.dummy`
 * makes the header the first allocation, naturally at the region origin
 * (base+0x00); the explicit ORIGIN(RAM) just documents/pins that. */
SECTIONS
{
  .fw_header ORIGIN(RAM) :
  {
    KEEP(*(.fw_header));
  } > RAM
} INSERT BEFORE .text.dummy;
