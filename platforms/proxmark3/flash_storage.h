#ifndef FANTASI_FLASH_STORAGE_H
#define FANTASI_FLASH_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/* AT91SAM7S512: 256 KB storage in flash plane 1 (0x140000).
 * Pages are 256 bytes with auto-erase-on-write via EFC1. */
#define STORAGE_SIZE        (256U * 1024U)
#define STORAGE_PAGE_SIZE   256U
#define STORAGE_PAGE_COUNT  (STORAGE_SIZE / STORAGE_PAGE_SIZE)
#define STORAGE_PROG_SIZE   4U   /* 32-bit word */
#define STORAGE_CACHE_SIZE  128U
#define STORAGE_LOOKAHEAD_SIZE 128U  /* ceil(1024/8) */

int      storage_flash_init(void);
uint32_t storage_flash_base(void);

int  storage_flash_read(uint32_t offset, void *buf, size_t len);
int  storage_flash_erase(uint32_t page_index);
int  storage_flash_program(uint32_t offset, const void *buf, size_t len);

#endif
