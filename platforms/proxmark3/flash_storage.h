#ifndef FANTASI_FLASH_STORAGE_H
#define FANTASI_FLASH_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/* AT91SAM7S512 flash 0x100000-0x180000 (512 KB) layout:
 *   0x100000-0x102000  bootloader (8 KB, preserved)
 *   0x102000-0x160000  app / osimage (<=376 KB)     - must match linker.ld osimage
 *   0x160000-0x180000  LittleFS storage (128 KB, fixed)
 * Storage sits at a fixed base above the app (not at the plane-1 base 0x140000):
 * a >256 KB firmware (e.g. with Berry linked) exceeds 0x140000, so storage
 * there would overlap the image and the two would corrupt each other's writes.
 * The fixed base above the app also leaves room for the firmware to grow (e.g. an
 * NFC lib). Pages are 256 B, auto-erase-on-write via EFC1 (EFC1 page numbers are
 * relative to the plane-1 hardware base 0x140000, see flash_storage.c). S512 only. */
#define STORAGE_BASE        0x00160000U
#define STORAGE_SIZE        (128U * 1024U)
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
