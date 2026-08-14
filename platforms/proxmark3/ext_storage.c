/* Proxmark3 external storage: a second LittleFS instance on the onboard SPI NOR
 * flash (flashmem, NPCS2/PA10), registered with the VFS as /mnt/ext0. Strong
 * override of the weak hal_ext_storage_init() (default no-op), called once from the
 * CLI task after hal_post_init(). Gated on a runtime JEDEC probe: boards without the
 * chip populated simply get no /mnt/ext0, so this is safe on every PM3 variant.
 * Independent of the internal bank-2 LittleFS: own block device, config, buffers,
 * lfs_t. Every flash access serialises against the FPGA on the shared SPI bus
 * (spi_flash.c -> spi_bus.c). Mirrors the PM5's ext_storage.c. */
#include "spi_flash.h"
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
    return spi_flash_read(b * EXT_BLOCK_SIZE + off, buf, sz) == 0 ? 0 : LFS_ERR_IO;
}
static int ext_prog(const struct lfs_config *c, lfs_block_t b, lfs_off_t off,
                    const void *buf, lfs_size_t sz)
{
    (void)c;
    return spi_flash_program(b * EXT_BLOCK_SIZE + off, buf, sz) == 0 ? 0 : LFS_ERR_IO;
}
static int ext_erase(const struct lfs_config *c, lfs_block_t b)
{
    (void)c;
    return spi_flash_erase4k(b * EXT_BLOCK_SIZE) == 0 ? 0 : LFS_ERR_IO;
}
static int ext_sync(const struct lfs_config *c) { (void)c; return 0; }

/* Probe + mount + register. On its own task (not the CLI task): a blank/foreign chip
 * needs a one-time format that can take a while, and that must never delay the CLI. */
static void ext_storage_task(void *arg)
{
    (void)arg;

    if (!spi_flash_init()) {
        fantasi_log(LOG_INFO, "ext: no SPI flash on this board");
        vTaskDelete(NULL);
        return;
    }
    uint8_t id[3]; spi_flash_id(id);
    uint32_t size = spi_flash_size();
    fantasi_log(LOG_INFO, "ext: SPI flash id %02X %02X %02X, %lu KB",
                id[0], id[1], id[2], (unsigned long)(size / 1024u));
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
    fantasi_log(LOG_INFO, "ext: /mnt/ext0 mounted%s", formatted ? " (formatted)" : "");
    vTaskDelete(NULL);
}

void hal_ext_storage_init(void)
{
    /* Spawn the bring-up so a format never blocks the CLI task that calls us. The stack
     * is freed on vTaskDelete once mounting finishes (or the chip is absent). 6 KB covers
     * lfs_mount/format with margin. */
    xTaskCreate(ext_storage_task, "extstor", configMINIMAL_STACK_SIZE * 6,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
