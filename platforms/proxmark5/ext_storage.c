/* Proxmark5 external storage: a second LittleFS instance on the QSPI NOR flash,
 * registered with the VFS as /mnt/ext0. Strong override of the weak
 * hal_ext_storage_init() (default no-op in hal/storage/hal_storage.c), called
 * once from the CLI task after hal_post_init(). Independent of the internal
 * bank2 LittleFS: its own block device, config, buffers, and lfs_t. */
#include "qspi_flash.h"
#include "../../hal/hal.h"
#include "../../core/vfs.h"
#include "../../core/log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lfs.h"
#include <string.h>

#define EXT_BLOCK_SIZE     4096u   /* 4 KB sector erase = LittleFS block */
#define EXT_PROG_SIZE      256u    /* 256 B page program                 */
#define EXT_CACHE_SIZE     256u
#define EXT_LOOKAHEAD_SIZE 16u

static lfs_t            ext_lfs;
static struct lfs_config ext_cfg;                 /* block_count filled at runtime */
static uint8_t ext_read_buf[EXT_CACHE_SIZE]      __attribute__((aligned(4)));
static uint8_t ext_prog_buf[EXT_CACHE_SIZE]      __attribute__((aligned(4)));
static uint8_t ext_look_buf[EXT_LOOKAHEAD_SIZE]  __attribute__((aligned(4)));

static int ext_read(const struct lfs_config *c, lfs_block_t b, lfs_off_t off,
                    void *buf, lfs_size_t sz)
{
    (void)c;
    return qspi_flash_read(b * EXT_BLOCK_SIZE + off, buf, sz) == 0 ? 0 : LFS_ERR_IO;
}
static int ext_prog(const struct lfs_config *c, lfs_block_t b, lfs_off_t off,
                    const void *buf, lfs_size_t sz)
{
    (void)c;
    return qspi_flash_program(b * EXT_BLOCK_SIZE + off, buf, sz) == 0 ? 0 : LFS_ERR_IO;
}
static int ext_erase(const struct lfs_config *c, lfs_block_t b)
{
    (void)c;
    return qspi_flash_erase4k(b * EXT_BLOCK_SIZE) == 0 ? 0 : LFS_ERR_IO;
}
static int ext_sync(const struct lfs_config *c) { (void)c; return 0; }

/* Detect + mount the QSPI LittleFS and register it as /mnt/ext0. Runs on its own
 * task (not the CLI task): a blank/foreign chip needs a one-time format that can
 * take a while, and that must never delay the CLI coming up. */
static void ext_storage_task(void *arg)
{
    (void)arg;
    TickType_t t0 = xTaskGetTickCount();

    if (!qspi_flash_init()) {
        fantasi_log(LOG_WARN, "ext: QSPI flash not detected");
        vTaskDelete(NULL);
        return;
    }
    uint8_t id[3]; qspi_flash_id(id);
    uint32_t size = qspi_flash_size();
    fantasi_log(LOG_INFO, "ext: QSPI id %02X %02X %02X, %lu KB, quad=%d",
                id[0], id[1], id[2], (unsigned long)(size / 1024u), qspi_flash_is_quad());
    if (size < EXT_BLOCK_SIZE) { vTaskDelete(NULL); return; }

    memset(&ext_cfg, 0, sizeof ext_cfg);
    ext_cfg.read  = ext_read;
    ext_cfg.prog  = ext_prog;
    ext_cfg.erase = ext_erase;
    ext_cfg.sync  = ext_sync;
    ext_cfg.read_size        = 1;
    ext_cfg.prog_size        = EXT_PROG_SIZE;
    ext_cfg.block_size       = EXT_BLOCK_SIZE;
    ext_cfg.block_count      = size / EXT_BLOCK_SIZE;
    ext_cfg.cache_size       = EXT_CACHE_SIZE;
    ext_cfg.lookahead_size   = EXT_LOOKAHEAD_SIZE;
    ext_cfg.block_cycles     = 500;
    ext_cfg.read_buffer      = ext_read_buf;
    ext_cfg.prog_buffer      = ext_prog_buf;
    ext_cfg.lookahead_buffer = ext_look_buf;

    /* Mount; format-on-fail for a blank/foreign chip (matches the internal LFS). */
    bool formatted = false;
    if (lfs_mount(&ext_lfs, &ext_cfg) != 0) {
        if (lfs_format(&ext_lfs, &ext_cfg) != 0) {
            fantasi_log(LOG_ERROR, "ext: format failed");
            vTaskDelete(NULL); return;
        }
        if (lfs_mount(&ext_lfs, &ext_cfg) != 0) {
            fantasi_log(LOG_ERROR, "ext: remount after format failed");
            vTaskDelete(NULL); return;
        }
        formatted = true;
    }
    vfs_mount_ext_lfs(&ext_lfs);                   /* -> /mnt/ext0 */
    uint32_t ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    fantasi_log(LOG_INFO, "ext: /mnt/ext0 mounted%s in %lu ms",
                formatted ? " (formatted)" : "", (unsigned long)ms);
    vTaskDelete(NULL);
}

void hal_ext_storage_init(void)
{
    /* Spawn the bring-up so it never blocks the CLI task that calls us. */
    xTaskCreate(ext_storage_task, "extstor", configMINIMAL_STACK_SIZE * 8,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
