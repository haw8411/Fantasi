/* LittleFS integration and MSC block-device layer.
 *
 * LittleFS manages the flash storage region with 4 KB blocks. MSC
 * exposes the same region as 512-byte sectors. They are mutually
 * exclusive: MSC start/stop callbacks unmount/remount LittleFS. */

#include "hal_storage.h"
#include "flash_storage.h"
#include "fat_ramdisk.h"   /* fatrd_invalidate() */
#include "lfs.h"
#include <string.h>

/* ---- LittleFS callbacks ---- */

static int lfs_read_cb(const struct lfs_config *c, lfs_block_t block,
                       lfs_off_t off, void *buf, lfs_size_t size)
{
    (void)c;
    return storage_flash_read(block * STORAGE_PAGE_SIZE + off, buf, size);
}

static int lfs_prog_cb(const struct lfs_config *c, lfs_block_t block,
                       lfs_off_t off, const void *buf, lfs_size_t size)
{
    (void)c;
    return storage_flash_program(block * STORAGE_PAGE_SIZE + off, buf, size);
}

static int lfs_erase_cb(const struct lfs_config *c, lfs_block_t block)
{
    (void)c;
    return storage_flash_erase(block);
}

static int lfs_sync_cb(const struct lfs_config *c)
{
    (void)c;
    return 0;
}

/* ---- LittleFS state ---- */

static lfs_t lfs;
static volatile bool mounted;

/* Word-aligned: lfs hands prog_buf to the flash driver as the program source,
 * and the Chameleon's SoftDevice sd_flash_write requires a word-aligned source.
 * As plain uint8_t[] the linker packs these at arbitrary addresses; an odd
 * lfs_prog_buf makes every SoftDevice flash write fail (retried forever, never
 * committed → writes silently lost). */
static uint8_t lfs_read_buf[STORAGE_CACHE_SIZE]          __attribute__((aligned(4)));
static uint8_t lfs_prog_buf[STORAGE_CACHE_SIZE]          __attribute__((aligned(4)));
static uint8_t lfs_lookahead_buf[STORAGE_LOOKAHEAD_SIZE] __attribute__((aligned(4)));

static const struct lfs_config lfs_cfg = {
    .read  = lfs_read_cb,
    .prog  = lfs_prog_cb,
    .erase = lfs_erase_cb,
    .sync  = lfs_sync_cb,

    .read_size      = 1,
    .prog_size      = STORAGE_PROG_SIZE,
    .block_size     = STORAGE_PAGE_SIZE,
    .block_count    = STORAGE_PAGE_COUNT,
    .cache_size     = STORAGE_CACHE_SIZE,
    .lookahead_size = STORAGE_LOOKAHEAD_SIZE,
    .block_cycles   = 500,

    .read_buffer      = lfs_read_buf,
    .prog_buffer      = lfs_prog_buf,
    .lookahead_buffer = lfs_lookahead_buf,
};

int hal_storage_init(void)
{
    if (storage_flash_init() < 0)
        return -1;

    int err = lfs_mount(&lfs, &lfs_cfg);
    if (err) {
        lfs_format(&lfs, &lfs_cfg);
        err = lfs_mount(&lfs, &lfs_cfg);
    }
    mounted = (err == 0);

#ifdef FANTASI_ENABLE_APPS
    /* App store: ensure /apps exists (idempotent). Runs at boot before the
     * SoftDevice is up, so the flash write is synchronous. */
    if (mounted) lfs_mkdir(&lfs, "/apps");
#endif

    fatrd_invalidate();   /* (re)mount changed the tree the synthetic FAT mirrors */
    return err;
}

int hal_storage_mount(void)
{
    int err = lfs_mount(&lfs, &lfs_cfg);
    mounted = (err == 0);
    return err;
}

void hal_storage_unmount(void)
{
    if (mounted) {
        lfs_unmount(&lfs);
        mounted = false;
    }
}

bool hal_storage_mounted(void)
{
    return mounted;
}

int hal_storage_read_file(const char *path, void *buf, uint32_t max_len)
{
    if (!mounted) hal_storage_mount();
    if (!mounted) return -1;

    static uint8_t file_cache[STORAGE_CACHE_SIZE] __attribute__((aligned(4)));
    struct lfs_file_config fcfg = { .buffer = file_cache };

    lfs_file_t f;
    if (lfs_file_opencfg(&lfs, &f, path, LFS_O_RDONLY, &fcfg) < 0)
        return -1;
    lfs_ssize_t n = lfs_file_read(&lfs, &f, buf, max_len);
    lfs_file_close(&lfs, &f);
    return (n < 0) ? -1 : (int)n;
}

int hal_storage_write_file(const char *path, const void *buf, uint32_t len)
{
    if (!mounted) hal_storage_mount();
    if (!mounted) return -1;
    fatrd_invalidate();   /* settings.cfg / ble_bond.bin appear in the FAT model */

    static uint8_t file_cache_w[STORAGE_CACHE_SIZE] __attribute__((aligned(4)));
    struct lfs_file_config fcfg = { .buffer = file_cache_w };

    lfs_file_t f;
    int rc = lfs_file_opencfg(&lfs, &f, path,
                              LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &fcfg);
    if (rc < 0) return -1;
    lfs_ssize_t n = lfs_file_write(&lfs, &f, buf, len);
    int crc = lfs_file_close(&lfs, &f);
    if (n < 0 || crc < 0) return -1;
    return (int)n;
}

struct lfs *hal_storage_lfs(void)
{
    if (!mounted) hal_storage_mount();
    return mounted ? &lfs : NULL;
}

/* ---- Settings KV (key=value\n text in /settings.cfg) ---- */

#define SETTINGS_PATH "/settings.cfg"
#define SETTINGS_MAX  256

int hal_settings_get(const char *key, char *buf, int len)
{
    char raw[SETTINGS_MAX];
    int n = hal_storage_read_file(SETTINGS_PATH, raw, SETTINGS_MAX - 1);
    if (n < 0) return -1;
    raw[n] = '\0';

    int klen = 0;
    while (key[klen]) klen++;

    char *p = raw;
    while (*p) {
        if (memcmp(p, key, klen) == 0 && p[klen] == '=') {
            char *v = p + klen + 1;
            int vlen = 0;
            while (v[vlen] && v[vlen] != '\n') vlen++;
            if (vlen >= len) vlen = len - 1;
            memcpy(buf, v, vlen);
            buf[vlen] = '\0';
            return vlen;
        }
        while (*p && *p != '\n') p++;
        if (*p == '\n') p++;
    }
    return -1;
}

int hal_settings_set(const char *key, const char *value)
{
    char raw[SETTINGS_MAX];
    int n = hal_storage_read_file(SETTINGS_PATH, raw, SETTINGS_MAX - 1);
    if (n < 0) n = 0;
    raw[n] = '\0';

    int klen = 0;
    while (key[klen]) klen++;

    char out[SETTINGS_MAX];
    int olen = 0;
    char *p = raw;
    while (*p) {
        char *line = p;
        while (*p && *p != '\n') p++;
        int llen = (int)(p - line);
        if (*p == '\n') p++;
        if (memcmp(line, key, klen) == 0 && line[klen] == '=')
            continue;
        bool has_eq = false;
        for (int j = 0; j < llen; j++)
            if (line[j] == '=') { has_eq = true; break; }
        if (!has_eq) continue;
        if (olen + llen + 1 < SETTINGS_MAX) {
            memcpy(out + olen, line, llen);
            olen += llen;
            out[olen++] = '\n';
        }
    }

    int vlen = 0;
    while (value[vlen]) vlen++;
    if (olen + klen + 1 + vlen + 1 < SETTINGS_MAX) {
        memcpy(out + olen, key, klen);
        olen += klen;
        out[olen++] = '=';
        memcpy(out + olen, value, vlen);
        olen += vlen;
        out[olen++] = '\n';
    }

    return (hal_storage_write_file(SETTINGS_PATH, out, olen) >= 0) ? 0 : -1;
}

int hal_settings_dump(char *buf, int len)
{
    int n = hal_storage_read_file(SETTINGS_PATH, buf, len - 1);
    if (n < 0) return -1;
    buf[n] = '\0';
    return n;
}

/* The MSC LUN is the synthetic FAT (msc_device.c → fatrd_*); reads/writes go
 * through the lfs file API above. The old raw 512-byte block layer and its
 * deferred page-cache flush machinery (hal_storage_flush/request_flush/service,
 * flush_for_reuse, the sd_active/flush_yield hooks) were its only users and are
 * gone. */
