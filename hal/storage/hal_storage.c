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

/* Per-open-file cache buffers (lfs_file_config.buffer). Two of them so one read
 * and one write file can be open at once (the settings rewrite streams the old
 * config into a temp file). Shared by all the file helpers and the settings KV -
 * storage access is serialized, so there's no aliasing. Word-aligned for the
 * same reason as lfs_prog_buf (SoftDevice flash source alignment). */
static uint8_t s_rcache[STORAGE_CACHE_SIZE] __attribute__((aligned(4)));
static uint8_t s_wcache[STORAGE_CACHE_SIZE] __attribute__((aligned(4)));

int hal_storage_read_file(const char *path, void *buf, uint32_t max_len)
{
    if (!mounted) hal_storage_mount();
    if (!mounted) return -1;

    struct lfs_file_config fcfg = { .buffer = s_rcache };

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

    struct lfs_file_config fcfg = { .buffer = s_wcache };

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
#define SETTINGS_TMP  "/settings.tmp"

/* Buffered byte reader over an lfs file: a 64-byte window so the settings parser
 * never holds the whole config in RAM. Returns the next byte, or -1 at EOF/error.
 * Uses the module-global `lfs`. */
typedef struct { lfs_file_t *f; uint8_t buf[64]; int pos, n; } cfg_reader;
static int cfg_getc(cfg_reader *r)
{
    if (r->pos >= r->n) {
        lfs_ssize_t g = lfs_file_read(&lfs, r->f, r->buf, sizeof r->buf);
        if (g <= 0) return -1;
        r->n = (int)g;
        r->pos = 0;
    }
    return r->buf[r->pos++];
}

/* Scan /settings.cfg line by line for "key=" and copy its value (up to the
 * newline) into buf, NUL-terminated. Returns the value length, or -1 if the key
 * is not set. Streamed through cfg_getc, so the config size is unbounded. */
int hal_settings_get(const char *key, char *buf, int len)
{
    if (!mounted) hal_storage_mount();
    if (!mounted || len < 1) return -1;

    int klen = 0;
    while (key[klen]) klen++;

    struct lfs_file_config fcfg = { .buffer = s_rcache };
    lfs_file_t f;
    if (lfs_file_opencfg(&lfs, &f, SETTINGS_PATH, LFS_O_RDONLY, &fcfg) < 0)
        return -1;

    cfg_reader r = { &f, {0}, 0, 0 };
    int result = -1;
    int c = cfg_getc(&r);
    while (c >= 0) {
        int i = 0;                                     /* match key at line start */
        while (i < klen && c == (uint8_t)key[i]) { i++; c = cfg_getc(&r); }
        if (i == klen && c == '=') {
            int o = 0;
            while ((c = cfg_getc(&r)) >= 0 && c != '\n')
                if (o < len - 1) buf[o++] = (char)c;
            buf[o] = '\0';
            result = o;
            break;
        }
        while (c >= 0 && c != '\n') c = cfg_getc(&r);   /* skip to the next line */
        if (c == '\n') c = cfg_getc(&r);
    }

    lfs_file_close(&lfs, &f);
    return result;
}

/* Rewrite /settings.cfg: stream it into a temp file dropping the existing "key="
 * line, optionally append "key=value" (value != NULL), then atomically rename
 * over the original. value == NULL removes the key (unset). Only the read window
 * and one output byte are buffered, so the config is unbounded. Returns 0/-1. */
static int settings_rewrite(const char *key, const char *value)
{
    if (!mounted) hal_storage_mount();
    if (!mounted) return -1;

    int klen = 0;
    while (key[klen]) klen++;

    struct lfs_file_config fw = { .buffer = s_wcache };
    lfs_file_t dst;
    if (lfs_file_opencfg(&lfs, &dst, SETTINGS_TMP,
                         LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &fw) < 0)
        return -1;

    /* Copy every line of the old config except the one that sets `key`. */
    struct lfs_file_config fr = { .buffer = s_rcache };
    lfs_file_t src;
    if (lfs_file_opencfg(&lfs, &src, SETTINGS_PATH, LFS_O_RDONLY, &fr) >= 0) {
        cfg_reader r = { &src, {0}, 0, 0 };
        int c = cfg_getc(&r);
        while (c >= 0) {
            int i = 0;                                 /* match key at line start */
            while (i < klen && c == (uint8_t)key[i]) { i++; c = cfg_getc(&r); }
            if (i == klen && c == '=') {
                while (c >= 0 && c != '\n') c = cfg_getc(&r);   /* drop old line */
                if (c == '\n') c = cfg_getc(&r);
            } else {
                if (i) lfs_file_write(&lfs, &dst, key, i);       /* matched prefix */
                while (c >= 0 && c != '\n') {
                    uint8_t ch = (uint8_t)c;
                    lfs_file_write(&lfs, &dst, &ch, 1);
                    c = cfg_getc(&r);
                }
                uint8_t nl = '\n';
                lfs_file_write(&lfs, &dst, &nl, 1);              /* terminate line */
                if (c == '\n') c = cfg_getc(&r);
            }
        }
        lfs_file_close(&lfs, &src);
    }

    /* Append the new key=value (unless unsetting). */
    if (value) {
        int vlen = 0;
        while (value[vlen]) vlen++;
        uint8_t eq = '=', nl = '\n';
        lfs_file_write(&lfs, &dst, key, klen);
        lfs_file_write(&lfs, &dst, &eq, 1);
        lfs_file_write(&lfs, &dst, value, vlen);
        lfs_file_write(&lfs, &dst, &nl, 1);
    }

    if (lfs_file_close(&lfs, &dst) < 0) return -1;
    if (lfs_rename(&lfs, SETTINGS_TMP, SETTINGS_PATH) < 0) return -1;
    fatrd_invalidate();   /* settings.cfg changed in the synthetic FAT model */
    return 0;
}

int hal_settings_set(const char *key, const char *value) { return settings_rewrite(key, value); }
int hal_settings_unset(const char *key)                  { return settings_rewrite(key, NULL); }

int hal_settings_foreach(void (*cb)(const char *line, void *ctx), void *ctx)
{
    if (!mounted) hal_storage_mount();
    if (!mounted) return -1;

    struct lfs_file_config fcfg = { .buffer = s_rcache };
    lfs_file_t f;
    if (lfs_file_opencfg(&lfs, &f, SETTINGS_PATH, LFS_O_RDONLY, &fcfg) < 0)
        return 0;   /* no config yet = no lines */

    /* Stream line by line: only one line is buffered (256 B, ample for a key
     * plus a filesystem path), so the config itself is unbounded. */
    cfg_reader r = { &f, {0}, 0, 0 };
    char line[256];
    int o = 0, c;
    while ((c = cfg_getc(&r)) >= 0) {
        if (c == '\n') {
            line[o] = '\0';
            if (o) cb(line, ctx);              /* skip empty lines */
            o = 0;
        } else if (o < (int)sizeof(line) - 1) {
            line[o++] = (char)c;
        }
    }
    if (o) { line[o] = '\0'; cb(line, ctx); }   /* last line, no trailing newline */

    lfs_file_close(&lfs, &f);
    return 0;
}

/* The MSC LUN is the synthetic FAT (msc_device.c → fatrd_*); reads/writes go
 * through the lfs file API above. The old raw 512-byte block layer and its
 * deferred page-cache flush machinery (hal_storage_flush/request_flush/service,
 * flush_for_reuse, the sd_active/flush_yield hooks) were its only users and are
 * gone. */
