#ifndef FANTASI_FLASH_STORAGE_H
#define FANTASI_FLASH_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/* LittleFS storage region for the Proxmark5 (AT32F435, 1024 KB dual-bank).
 * The whole region lives in bank2 (0x08080000..0x08100000); the firmware is
 * linked into bank1, so runtime erase/program never touches the executing
 * bank. 2 KB sectors (RM Table 5-2) are the LittleFS block. 256 KB gives a
 * comfortable filesystem while leaving the upper half of bank2 free. */
#define STORAGE_BASE       0x08080000U        /* bank2 base */
#define STORAGE_SIZE       (256U * 1024U)
#define STORAGE_PAGE_SIZE  2048U              /* AT32F435 1024 KB sector = LittleFS block */
#define STORAGE_PAGE_COUNT (STORAGE_SIZE / STORAGE_PAGE_SIZE)   /* 128 */
#define STORAGE_PROG_SIZE  4U                 /* word program */
#define STORAGE_CACHE_SIZE 256U
#define STORAGE_LOOKAHEAD_SIZE 16U            /* ceil(128/8) rounded up */

int      storage_flash_init(void);
uint32_t storage_flash_base(void);

int  storage_flash_read(uint32_t offset, void *buf, size_t len);
int  storage_flash_erase(uint32_t page_index);
int  storage_flash_program(uint32_t offset, const void *buf, size_t len);

#endif
