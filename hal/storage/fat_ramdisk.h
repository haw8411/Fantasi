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
#include <stdbool.h>

#define FATRD_SECTOR_SIZE   512u

/* The synthetic drive is FAT32 with a runtime geometry (see fat_ramdisk.c), so the
 * MSC capacity is queried rather than a compile-time constant. */
uint32_t fatrd_sector_count(void);

/* ---- Real-FAT external-device passthrough (weak hooks) ----
 * A platform with a genuinely-FAT external device (the Flipper microSD) provides
 * strong versions of these. When one is present, the synthetic drive becomes
 * card-sized: the card's data region is windowed into the low clusters verbatim
 * (its own cluster numbers, so no rebasing) and served by raw sector passthrough,
 * while the internal RAM/LittleFS content is synthesized in higher clusters. This
 * keeps the card's many files off the eager model entirely. The default weak
 * versions report "no external device", leaving a small internal-only FAT32. */
typedef struct {
    uint32_t sec_per_clus;   /* card cluster size, in 512-byte sectors */
    uint32_t data_lba;       /* card LBA of cluster 2 (first data cluster) */
    uint32_t data_clusters;  /* number of data clusters on the card */
    uint32_t fat_lba;        /* card LBA of the (first) FAT */
    uint32_t root_clus;      /* card root-directory cluster (FAT32) */
    uint32_t free_clusters;  /* free data clusters on the card (from FatFs) */
} fatrd_ext_t;

bool fatrd_ext_present(fatrd_ext_t *out);      /* true + fills *out when a FAT card is ready */
int  fatrd_ext_read(uint32_t lba, uint8_t *buf);         /* raw card sector -> buf[512]; 0 ok, -1 err */
int  fatrd_ext_read_multi(uint32_t lba, uint8_t *buf, uint32_t count);   /* count consecutive sectors */
int  fatrd_ext_write(uint32_t lba, const uint8_t *buf);  /* buf[512] -> raw card sector; 0 ok, -1 err */

int  fatrd_read(uint32_t lba, uint32_t offset, void *buf, uint32_t len);
int  fatrd_write(uint32_t lba, uint32_t offset, const uint8_t *buf, uint32_t len);

/* Mark the synthetic model stale so the next access rebuilds it. Call after any
 * change to the underlying filesystem made OUTSIDE this module (the VFS layer,
 * the BLE/serial proto file handlers, settings/bond writes); writes that go
 * through fatrd_write invalidate themselves. Cheap - the rebuild is deferred to
 * the next read/write. */
void fatrd_invalidate(void);

/* Storage critical section serialising all LittleFS access between the MSC model
 * path (TinyUSB task) and the proto/VFS file ops (cli/proto tasks) - LittleFS is
 * not reentrant. fatrd_store_init() creates the (recursive) lock and must run once
 * at boot before any task touches storage. Hold the lock only across lfs_* calls,
 * never across a USB emit. */
void fatrd_store_init(void);
void fatrd_store_lock(void);
void fatrd_store_unlock(void);

/* Commit host-over-MSC file writes durably (call on SCSI SYNCHRONIZE CACHE). */
void fatrd_sync(void);

/* True (self-clearing) when the filesystem changed via a non-MSC path since the
 * host last polled - the MSC layer turns this into a "medium may have changed"
 * unit-attention so the host re-reads instead of serving a stale cached view. */
bool fatrd_media_changed(void);

/* Free the synthetic-FAT model (~6 KB). Call when the host can no longer be using the drive - MSC eject or
 * USB unplug. The model is rebuilt lazily on the next read/write, so an idle device (no drive mounted)
 * keeps none of it resident. Must be called on the usb task (same context as fatrd_read/fatrd_write). */
void fatrd_release(void);

#endif /* HAL_FAT_RAMDISK_H */
