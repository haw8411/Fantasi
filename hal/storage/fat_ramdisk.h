/* fat_ramdisk - a synthetic FAT16 view of the whole device storage, exposed as
 * the USB MSC drive so the host OS mounts it as a normal disk.
 *
 * The root contains two folders, RAMFS and APPS, listing the files reported by
 * the VFS (/ramfs in RAM, /apps on flash LittleFS). Nothing is stored in FAT
 * layout: reads synthesize the boot sector / FAT / directories / file data from
 * the VFS on the fly; writes are parsed back (directory entries -> create/delete,
 * data sectors -> staged then flushed to the file via the VFS, routing /ramfs to
 * RAM and /apps to flash). The geometry advertises a fixed capacity; writes that
 * exceed available RAM/flash are rejected. */
#ifndef HAL_FAT_RAMDISK_H
#define HAL_FAT_RAMDISK_H

#include <stdint.h>

#define FATRD_SECTOR_SIZE   512u
#define FATRD_SECTOR_COUNT  8258u   /* 8192 data clusters -> FAT16 */

int  fatrd_read(uint32_t lba, uint32_t offset, void *buf, uint32_t len);
int  fatrd_write(uint32_t lba, uint32_t offset, const uint8_t *buf, uint32_t len);

/* Mark the synthetic model stale so the next access rebuilds it. Call after any
 * change to the underlying filesystem made OUTSIDE this module (the VFS layer,
 * the BLE/serial proto file handlers, settings/bond writes); writes that go
 * through fatrd_write invalidate themselves. Cheap - the rebuild is deferred to
 * the next read/write. */
void fatrd_invalidate(void);

#endif /* HAL_FAT_RAMDISK_H */
