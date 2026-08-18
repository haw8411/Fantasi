/* Flipper external storage: the microSD card, a FAT volume registered with the
 * VFS as /mnt/ext0. Strong override of the weak hal_ext_storage_init() (default
 * no-op in hal/storage/hal_storage.c), called once from the CLI task after
 * hal_post_init(). The SD driver is platforms/flipper/sd_spi.c; the FAT ops go
 * through platforms/flipper/vfs_fat.c (FatFs). */
#include "sd_spi.h"
#include "../../hal/hal.h"
#include "../../core/vfs.h"
#include "../../core/log.h"
#include "../../hal/storage/fat_ramdisk.h"

#include "ff.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static FATFS s_fatfs;
static bool  s_ext_ready;         /* card mounted + geometry available for passthrough */

/* ---- Strong overrides of the fat_ramdisk passthrough hooks ----
 * The synthetic MSC drive windows the card's data region into its low clusters
 * verbatim; these feed it the card's FAT32 geometry (straight from the mounted
 * FATFS, which already parsed the BPB) and raw sector reads. */
bool fatrd_ext_present(fatrd_ext_t *out)
{
    if (!s_ext_ready || s_fatfs.fs_type != FS_FAT32) return false;
    if (out) {
        out->sec_per_clus  = s_fatfs.csize;
        out->data_lba      = (uint32_t)s_fatfs.database;
        out->data_clusters = s_fatfs.n_fatent - 2u;
        out->fat_lba       = (uint32_t)s_fatfs.fatbase;
        out->root_clus     = (uint32_t)s_fatfs.dirbase;
        out->free_clusters = s_fatfs.free_clst;   /* populated by f_getfree at mount */
    }
    return true;
}

int fatrd_ext_read(uint32_t lba, uint8_t *buf)
{
    return sd_spi_read(lba, buf, 1) == 0 ? 0 : -1;
}

int fatrd_ext_read_multi(uint32_t lba, uint8_t *buf, uint32_t count)
{
    return sd_spi_read(lba, buf, count) == 0 ? 0 : -1;   /* one multi-block transfer */
}

int fatrd_ext_write(uint32_t lba, const uint8_t *buf)
{
    return sd_spi_write(lba, buf, 1) == 0 ? 0 : -1;
}

/* Bring up the card + FAT on its own task: a slow card init must never delay the
 * CLI coming up. Unlike the Proxmark5 QSPI (a blank chip we format), the SD card
 * is the user's own media - we never format it on mount failure; we just leave
 * /mnt/ext0 unregistered so an unformatted/absent card is a no-op, not data loss. */
static void ext_storage_task(void *arg)
{
    (void)arg;
    TickType_t t0 = xTaskGetTickCount();

    if (!sd_spi_init()) {
        fantasi_log(LOG_WARN, "sd: init fail stage=%d r1=0x%02X ctype=%d",
                    sd_spi_diag_stage(), sd_spi_diag_r1(), sd_spi_diag_card_type());
        vTaskDelete(NULL);
        return;
    }
    uint32_t sectors = sd_spi_sector_count();

    /* Mount immediately (opt=1) so a bad/foreign filesystem is detected here. */
    FRESULT r = f_mount(&s_fatfs, "0:", 1);
    if (r != FR_OK) {
        fantasi_log(LOG_WARN, "sd: card ok (%lu MB) but f_mount=%d (not FAT?)",
                    (unsigned long)((uint64_t)sectors * 512u / (1024u * 1024u)), (int)r);
        vTaskDelete(NULL);
        return;
    }

    /* Self-test the FatFs path (root listing + free space) and log it. */
    int entries = 0;
    DIR dir;
    FILINFO fno;
    if (f_opendir(&dir, "0:/") == FR_OK) {
        while (entries < 100000 && f_readdir(&dir, &fno) == FR_OK && fno.fname[0])
            entries++;
        f_closedir(&dir);
    }
    DWORD nfree = 0;
    FATFS *fs = NULL;
    unsigned long free_mb = 0;
    if (f_getfree("0:", &nfree, &fs) == FR_OK)
        free_mb = (unsigned long)((uint64_t)nfree * fs->csize * 512u / (1024u * 1024u));
    uint32_t ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    fantasi_log(LOG_INFO, "sd: OK %lu MB, %d root entries, %lu MB free, %lu ms",
                (unsigned long)((uint64_t)sectors * 512u / (1024u * 1024u)),
                entries, free_mb, (unsigned long)ms);

    /* The card is FAT32 and mounted: expose it to the passthrough hooks (for the
     * MSC overlay) and register it as /mnt/ext0 for the CLI/proto file API. The MSC
     * model no longer walks it (build_model treats a FAT ext mount as passthrough),
     * so this is safe. */
    if (s_fatfs.fs_type == FS_FAT32) {
        s_ext_ready = true;
        fatrd_invalidate();            /* rebuild the MSC model with the card window */
    }
    vfs_mount_ext_fat(0);              /* -> /mnt/ext0 (CLI via FatFs) */
    vTaskDelete(NULL);
}

void hal_ext_storage_init(void)
{
    xTaskCreate(ext_storage_task, "extstor", configMINIMAL_STACK_SIZE * 8,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
