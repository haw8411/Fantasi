/* Minimal command-port SPI NOR driver for the PM3's onboard "flashmem" (the SOIC-8
 * on SPI0 NPCS2 / PA10; present on RDV4 and on Easy/clone boards that populate it,
 * absent on others). Polled PIO, no XIP - the SAM7 SPI has none. Backs a second
 * LittleFS instance at /mnt/ext0. Every op serialises against the FPGA on the shared
 * bus via spi_bus.h. Mirrors the PM5's qspi_flash API. */
#ifndef PM3_SPI_FLASH_H
#define PM3_SPI_FLASH_H

#include <stdint.h>
#include <stdbool.h>

/* JEDEC-probe the chip and cache its id + size. Returns true if a chip answered
 * (id not all-0x00 / all-0xFF), false otherwise. Safe to call repeatedly. */
bool     spi_flash_init(void);

/* Detected capacity in bytes (0 if absent). Derived from the capacity nibble as
 * the flashmem convention: 1 << (jedec & 0xF) 64 KB pages. */
uint32_t spi_flash_size(void);

/* Copy the cached JEDEC id (manufacturer, memory-type, capacity) into out[3]. */
void     spi_flash_id(uint8_t out[3]);

/* Read the chip's 64-bit unique id (cmd 0x4B) into out[8], first-transmitted byte in
 * out[0]. Bus-locked, so it is safe alongside the FPGA/RFID and the ext-storage mount.
 * Used to derive the device name. Returns all-0xFF/0x00 when no chip is present. */
void     spi_flash_unique_id(uint8_t out[8]);

/* Read `len` bytes from `addr` (single-line Read 0x03). 0, or -1 on range. */
int      spi_flash_read(uint32_t addr, void *buf, uint32_t len);

/* Program `len` bytes at `addr` (auto page-split at 256 B; range must be erased
 * first). 0, or -1 on range. */
int      spi_flash_program(uint32_t addr, const void *buf, uint32_t len);

/* Erase the 4 KB sector containing `addr` (0x20). 0, or -1 on range. */
int      spi_flash_erase4k(uint32_t addr);

#endif /* PM3_SPI_FLASH_H */
