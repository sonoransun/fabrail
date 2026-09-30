/* STM32G474RE memory map (RM0440 Rev 8, §2.2 "Memory organization" and §2.4
 * "Embedded SRAM"; DS12288 §3.4–3.5).
 *
 * FLASH: 512 KB main flash at 0x0800_0000 (dual-bank mode by default, DBANK = 1:
 *        2 × 256 KB banks of 2 KB pages; RM0440 §5.3.1).
 * RAM:   128 KB contiguous at 0x2000_0000 = SRAM1 80 KB (0x2000_0000–0x2001_3FFF)
 *        + SRAM2 16 KB (0x2001_4000–0x2001_7FFF) + CCM SRAM 32 KB aliased at
 *        0x2001_8000–0x2001_FFFF (CCM also at 0x1000_0000). Category-3 devices
 *        (G47x/G48x) map CCM contiguously after SRAM2, so one 128 KB region is valid.
 *        The stack grows down from the top of CCM (no DMA ever targets the stack).
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Parameter A/B pages (bsp/flash_params.rs): the top 8 KB of bank 2, i.e. pages
 * 124..127 of bank 2 (2 KB each): slot A = 0x0807_E000 (4 KB), slot B = 0x0807_F000
 * (4 KB). The program image must end below them. */
_param_pages_start = ORIGIN(FLASH) + LENGTH(FLASH) - 8K;

/* The image (vector table, .text, .rodata, .data load image, veneers) must not reach
 * the parameter pages. Evaluated after layout. */
ASSERT(__veneer_limit <= _param_pages_start, "
ERROR(fabrail-fw): the program image overlaps the parameter A/B flash pages");
