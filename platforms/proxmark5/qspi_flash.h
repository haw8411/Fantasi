/* Minimal Winbond W25Q QSPI NOR driver for the Proxmark5 external flash chip
 * (QSPI1 command-port mode, polled PIO - no XIP, no DMA). Backs a second
 * LittleFS instance mounted at /mnt/ext0. Ported from the PM5 fork's
 * common_arm/flash_data/flashmem_hw_at32.c, reduced to read/program/erase. */
#ifndef PM5_QSPI_FLASH_H
#define PM5_QSPI_FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Bring up QSPI1 + pins and detect the chip via JEDEC ID. Returns true and sets
 * the size on success; false if no/unknown chip or comms fail the WREN check. */
bool     qspi_flash_init(void);

/* Detected capacity in bytes (0 if init failed / no chip). */
uint32_t qspi_flash_size(void);

/* True once the flash's Quad Enable bit is set and IO2/IO3 are muxed for quad reads. */
bool     qspi_flash_is_quad(void);

/* Copy the last-read JEDEC id (mfr, memtype, capacity) into out[3]. */
void     qspi_flash_id(uint8_t out[3]);

/* Read `len` bytes from `addr` (single-line Read, 0x03). 0, or -1 on range. */
int      qspi_flash_read(uint32_t addr, void *buf, uint32_t len);

/* Program `len` bytes at `addr` (page-split at 256 B boundaries; the target
 * range must be erased first). 0, or -1. */
int      qspi_flash_program(uint32_t addr, const void *buf, uint32_t len);

/* Erase the 4 KB sector containing `addr`. 0, or -1. */
int      qspi_flash_erase4k(uint32_t addr);

#endif /* PM5_QSPI_FLASH_H */
