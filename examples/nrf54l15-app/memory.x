MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1524K /* leave some for the coprocessor */
  RAM : ORIGIN = 0x20000000, LENGTH = 256K - 128K /* leave some for the coprocessor example */

  /* RAM reserved for the FLPR (coprocessor): firmware code, its working RAM and
   * the shared sQSPI register block. The reimplemented (non-PIC) sQSPI firmware
   * is linked at exactly 0x20020000, so the host's FW_RAM buffer must live here
   * (see the `.flpr_ram` section below and src/bin/sqspi_reimpl.rs). Nordic's
   * PIC blob doesn't care where it lands, so the stock sqspi.rs example, which
   * keeps FW_RAM in normal .bss, is unaffected by this region existing. */
  FLPR_RAM : ORIGIN = 0x20020000, LENGTH = 128K
}

/* Place anything tagged `.flpr_ram` at the FLPR_RAM origin. NOLOAD: the bytes
 * are not in the image (the host zero-initializes the buffer before loading the
 * firmware). Empty for every bin that doesn't reference it. */
SECTIONS
{
  .flpr_ram (NOLOAD) : ALIGN(128)
  {
    KEEP(*(.flpr_ram));
    KEEP(*(.flpr_ram.*));
  } > FLPR_RAM
} INSERT AFTER .bss;
