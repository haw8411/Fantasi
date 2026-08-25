#include "vfs.h"
#include "../hal/storage/hal_storage.h"
#include "../hal/storage/fat_ramdisk.h"   /* fatrd_invalidate() */

#include "FreeRTOS.h"
#include "lfs.h"
#include <string.h>

/* ramfs is present on every target that enables the app feature (all of them
 * today); without it the VFS is a thin pass-through to LittleFS. */
#ifdef FANTASI_ENABLE_APPS
#include "ramfs.h"
#define HAS_RAMFS 1
#else
#define HAS_RAMFS 0
#endif

/* ---- Mount table ----
 * Fixed built-ins ("/ramfs" when apps are enabled, "/" internal LittleFS) plus
 * up to VFS_MAX_EXT external LittleFS devices registered at boot as /mnt/extN.
 * The internal "/" mount stores lfs == NULL and resolves on demand via
 * hal_storage_lfs(); external mounts carry their own instance. */
#define VFS_MAX_EXT 4

static vfs_mount_t s_mounts[1 + HAS_RAMFS + VFS_MAX_EXT];
static char        s_ext_prefix[VFS_MAX_EXT][12];   /* "/mnt/extN" */
static int         s_nmounts;
static int         s_next_ext;
static int         s_inited;
/* Every LittleFS file operation below holds this same recursive store lock
 * from open through close, so their per-open caches can share one buffer. */
static uint8_t     s_file_cache[256];

static void vfs_init_mounts(void)
{
    if (s_inited) return;
    int i = 0;
#if HAS_RAMFS
    s_mounts[i].prefix = VFS_RAMFS_MOUNT; s_mounts[i].kind = VFS_BK_RAMFS; s_mounts[i].lfs = NULL; i++;
#endif
    s_mounts[i].prefix = "/"; s_mounts[i].kind = VFS_BK_LFS; s_mounts[i].lfs = NULL; i++;  /* internal */
    s_nmounts = i;
    s_inited = 1;
}

static bool is_ext_mount(const vfs_mount_t *m)
{
    return (m->kind == VFS_BK_LFS || m->kind == VFS_BK_FAT) &&
           strncmp(m->prefix, VFS_EXT_MOUNT "/", 5) == 0;
}

/* Fill the next /mnt/extN prefix slot (shared by the LFS and FAT registrars).
 * Returns the slot index N, or -1 if the table is full. */
static int ext_prefix_alloc(void)
{
    if (s_next_ext >= VFS_MAX_EXT) return -1;
    int n = s_next_ext;
    char *p = s_ext_prefix[n];
    memcpy(p, VFS_EXT_MOUNT "/ext", 8);   /* "/mnt/ext" */
    p[8] = (char)('0' + n);               /* single digit; VFS_MAX_EXT < 10 */
    p[9] = '\0';
    s_mounts[s_nmounts].prefix = p;
    return n;
}

int vfs_mount_ext_lfs(struct lfs *lfs)
{
    vfs_init_mounts();
    if (!lfs) return -1;
    int n = ext_prefix_alloc();
    if (n < 0) return -1;
    s_mounts[s_nmounts].kind   = VFS_BK_LFS;
    s_mounts[s_nmounts].lfs    = lfs;
    s_mounts[s_nmounts].fatdrv = -1;
    s_nmounts++;
    s_next_ext++;
    return n;
}

int vfs_mount_ext_fat(int drive)
{
    vfs_init_mounts();
    if (drive < 0) return -1;
    int n = ext_prefix_alloc();
    if (n < 0) return -1;
    s_mounts[s_nmounts].kind   = VFS_BK_FAT;
    s_mounts[s_nmounts].lfs    = NULL;
    s_mounts[s_nmounts].fatdrv = drive;
    s_nmounts++;
    s_next_ext++;
    return n;
}

/* ---- Weak FAT-backend stubs. The platform that owns a FAT volume overrides
 * these (platforms/flipper/vfs_fat.c). On a platform with no FAT mount they are
 * never reached (no VFS_BK_FAT mount is ever registered), so the failing
 * defaults just satisfy the linker. ---- */
__attribute__((weak)) int32_t vfs_fat_pread(int drv, const char *leaf, uint32_t off,
                                            void *buf, uint32_t len)
{ (void)drv; (void)leaf; (void)off; (void)buf; (void)len; return -1; }
__attribute__((weak)) int32_t vfs_fat_size(int drv, const char *leaf)
{ (void)drv; (void)leaf; return -1; }
__attribute__((weak)) int vfs_fat_wchunk(int drv, const char *leaf, uint32_t off,
                                         const void *buf, uint32_t len, bool last)
{ (void)drv; (void)leaf; (void)off; (void)buf; (void)len; (void)last; return -1; }
__attribute__((weak)) int vfs_fat_remove(int drv, const char *leaf)
{ (void)drv; (void)leaf; return -1; }
__attribute__((weak)) int vfs_fat_mkdir(int drv, const char *leaf)
{ (void)drv; (void)leaf; return -1; }
__attribute__((weak)) int vfs_fat_rename(int drv, const char *from, const char *to)
{ (void)drv; (void)from; (void)to; return -1; }
__attribute__((weak)) int vfs_fat_statfs(int drv, uint32_t *total, uint32_t *freeb)
{ (void)drv; if (total) *total = 0; if (freeb) *freeb = 0; return -1; }
__attribute__((weak)) int vfs_fat_list(int drv, const char *leaf, vfs_list_cb cb, void *ctx)
{ (void)drv; (void)leaf; (void)cb; (void)ctx; return -1; }

const vfs_mount_t *vfs_resolve(const char *path, const char **leaf_out)
{
    vfs_init_mounts();
    if (!path) { if (leaf_out) *leaf_out = ""; return NULL; }

    const vfs_mount_t *best = NULL, *root = NULL;
    size_t bestlen = 0;
    for (int i = 0; i < s_nmounts; i++) {
        const char *pfx = s_mounts[i].prefix;
        if (pfx[0] == '/' && pfx[1] == '\0') { root = &s_mounts[i]; continue; }   /* "/" catch-all */
        size_t plen = strlen(pfx);
        if (strncmp(path, pfx, plen) == 0 && (path[plen] == '\0' || path[plen] == '/')) {
            if (plen > bestlen) { best = &s_mounts[i]; bestlen = plen; }
        }
    }

    if (best) {
        const char *l = path + bestlen;
        if (best->kind == VFS_BK_RAMFS) {
            if (*l == '/') l++;
            if (leaf_out) *leaf_out = l;             /* flat name; "" for the root */
        } else {
            if (leaf_out) *leaf_out = (*l == '\0') ? "/" : l;   /* lfs-rooted path */
        }
        return best;
    }
    if (leaf_out) *leaf_out = path;                  /* internal lfs takes the full path */
    return root;
}

bool vfs_mount_is_ramfs(const vfs_mount_t *m) { return m && m->kind == VFS_BK_RAMFS; }
bool vfs_mount_is_fat(const vfs_mount_t *m)   { return m && m->kind == VFS_BK_FAT; }

struct lfs *vfs_mount_lfs(const vfs_mount_t *m)
{
    if (!m || m->kind != VFS_BK_LFS) return NULL;
    return m->lfs ? m->lfs : hal_storage_lfs();
}

bool vfs_is_ramfs(const char *path)
{
    if (!HAS_RAMFS || !path) return false;
    size_t n = strlen(VFS_RAMFS_MOUNT);
    if (strncmp(path, VFS_RAMFS_MOUNT, n) != 0) return false;
    return path[n] == '\0' || path[n] == '/';
}

const char *vfs_ramfs_leaf(const char *path)
{
    if (!vfs_is_ramfs(path)) return NULL;
    const char *p = path + strlen(VFS_RAMFS_MOUNT);
    if (*p == '/') p++;
    return p;   /* "" for the mount root */
}

int vfs_read_all(const char *path, const uint8_t **data, uint32_t *len, bool *owned)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        uint32_t n = 0;
        const uint8_t *p = ramfs_get(leaf, &n);
        if (!p) return -1;
        *data = p; *len = n; *owned = false;
        return 0;
    }
#endif
    if (m->kind == VFS_BK_FAT) {
        int32_t sz = vfs_fat_size(m->fatdrv, leaf);
        if (sz < 0) return -1;
        uint8_t *buf = pvPortMalloc((size_t)sz ? (size_t)sz : 1);
        if (!buf) return -1;
        int32_t got = sz ? vfs_fat_pread(m->fatdrv, leaf, 0, buf, (uint32_t)sz) : 0;
        if (got != sz) { vPortFree(buf); return -1; }
        *data = buf; *len = (uint32_t)sz; *owned = true;
        return 0;
    }
    lfs_t *lfs = vfs_mount_lfs(m);
    if (!lfs) return -1;

    struct lfs_file_config fcfg = { .buffer = s_file_cache };
    lfs_file_t f;
    fatrd_store_lock();
    if (lfs_file_opencfg(lfs, &f, leaf, LFS_O_RDONLY, &fcfg) < 0) { fatrd_store_unlock(); return -1; }

    lfs_ssize_t sz = lfs_file_size(lfs, &f);
    if (sz < 0) { lfs_file_close(lfs, &f); fatrd_store_unlock(); return -1; }

    uint8_t *buf = pvPortMalloc((size_t)sz ? (size_t)sz : 1);
    if (!buf) { lfs_file_close(lfs, &f); fatrd_store_unlock(); return -1; }

    lfs_ssize_t got = lfs_file_read(lfs, &f, buf, (lfs_size_t)sz);
    lfs_file_close(lfs, &f);
    fatrd_store_unlock();
    if (got != sz) { vPortFree(buf); return -1; }

    *data = buf; *len = (uint32_t)sz; *owned = true;
    return 0;
}

void vfs_free(const uint8_t *data)
{
    if (data) vPortFree((void *)data);
}

int vfs_take_ramfs(const char *path, uint8_t **data, uint32_t *len)
{
#if HAS_RAMFS
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (m && m->kind == VFS_BK_RAMFS)
        return ramfs_take(leaf, data, len);
#else
    (void)path; (void)data; (void)len;
#endif
    return -1;
}

int32_t vfs_pread(const char *path, uint32_t off, void *buf, uint32_t len)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS)
        return ramfs_read(leaf, off, buf, len);
#endif
    if (m->kind == VFS_BK_FAT)
        return vfs_fat_pread(m->fatdrv, leaf, off, buf, len);
    lfs_t *lfs = vfs_mount_lfs(m);
    if (!lfs) return -1;
    struct lfs_file_config fcfg = { .buffer = s_file_cache };
    lfs_file_t f;
    fatrd_store_lock();
    if (lfs_file_opencfg(lfs, &f, leaf, LFS_O_RDONLY, &fcfg) < 0) { fatrd_store_unlock(); return -1; }
    if (off) lfs_file_seek(lfs, &f, off, LFS_SEEK_SET);
    lfs_ssize_t n = lfs_file_read(lfs, &f, buf, len);
    lfs_file_close(lfs, &f);
    fatrd_store_unlock();
    return n < 0 ? -1 : (int32_t)n;
}

int32_t vfs_read_file(const char *path, void *buf, uint32_t max)
{
    return vfs_pread(path, 0, buf, max);
}

/* Every mutator here invalidates the synthetic-FAT model after the change commits,
 * never before: a pre-commit bump lets a concurrent MSC read cache a model that
 * predates the write under the new generation, and that poisoned cache survives.
 * See the fuller note in proto.c handle_file_write. */
static int vfs_write_file_origin(const char *path, const void *buf, uint32_t len,
                                 bool from_msc)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        rc = (ramfs_truncate(leaf) != 0) ? -1 : ramfs_write_at(leaf, 0, buf, len);
    } else
#endif
    if (m->kind == VFS_BK_FAT) {
        rc = vfs_fat_wchunk(m->fatdrv, leaf, 0, buf, len, true);   /* truncate + write */
    } else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        struct lfs_file_config fcfg = { .buffer = s_file_cache };
        lfs_file_t f;
        fatrd_store_lock();
        if (lfs_file_opencfg(lfs, &f, leaf,
                             LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &fcfg) < 0) { fatrd_store_unlock(); return -1; }
        lfs_ssize_t n = lfs_file_write(lfs, &f, buf, len);
        int crc = lfs_file_close(lfs, &f);
        fatrd_store_unlock();
        rc = (n == (lfs_ssize_t)len && crc >= 0) ? 0 : -1;
    }
    if (from_msc) {
        fatrd_invalidate_msc();
    } else {
        if (rc == 0) fatrd_external_forget(path);
        fatrd_invalidate();
    }
    return rc;
}

int vfs_write_file(const char *path, const void *buf, uint32_t len)
{ return vfs_write_file_origin(path, buf, len, false); }

int vfs_msc_write_file(const char *path, const void *buf, uint32_t len)
{ return vfs_write_file_origin(path, buf, len, true); }

/* Append `buf` to the end of `path` (created if absent). Lets a module stream a large log incrementally
 * without buffering it all - the whole-file write_file would truncate. */
int vfs_append(const char *path, const void *buf, uint32_t len)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        int32_t sz = ramfs_size(leaf); if (sz < 0) sz = 0;
        rc = ramfs_write_at(leaf, (uint32_t)sz, buf, len);
    } else
#endif
    if (m->kind == VFS_BK_FAT) {
        int32_t sz = vfs_fat_size(m->fatdrv, leaf); if (sz < 0) sz = 0;
        rc = vfs_fat_wchunk(m->fatdrv, leaf, (uint32_t)sz, buf, len, true);
    } else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        struct lfs_file_config fcfg = { .buffer = s_file_cache };
        lfs_file_t f;
        fatrd_store_lock();
        if (lfs_file_opencfg(lfs, &f, leaf,
                             LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND, &fcfg) < 0) { fatrd_store_unlock(); return -1; }
        lfs_ssize_t n = lfs_file_write(lfs, &f, buf, len);
        int crc = lfs_file_close(lfs, &f);
        fatrd_store_unlock();
        rc = (n == (lfs_ssize_t)len && crc >= 0) ? 0 : -1;
    }
    if (rc == 0) fatrd_external_forget(path);
    fatrd_invalidate();
    return rc;
}

/* Write `len` bytes at absolute offset `off` (file created if absent, not truncated).
 * Lets a caller stream a file one piece at a time in any order without buffering the
 * whole thing, and - being positional rather than truncate-then-append - a repeated
 * write of the same offsets just overwrites in place instead of destroying the tail. */
int vfs_pwrite(const char *path, uint32_t off, const void *buf, uint32_t len)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        rc = ramfs_write_at(leaf, off, buf, len);
    } else
#endif
    if (m->kind == VFS_BK_FAT) {
        rc = vfs_fat_wchunk(m->fatdrv, leaf, off, buf, len, true);
    } else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        struct lfs_file_config fcfg = { .buffer = s_file_cache };
        lfs_file_t f;
        fatrd_store_lock();
        if (lfs_file_opencfg(lfs, &f, leaf,
                             LFS_O_WRONLY | LFS_O_CREAT, &fcfg) < 0) { fatrd_store_unlock(); return -1; }
        if (off) lfs_file_seek(lfs, &f, off, LFS_SEEK_SET);
        lfs_ssize_t n = lfs_file_write(lfs, &f, buf, len);
        int crc = lfs_file_close(lfs, &f);
        fatrd_store_unlock();
        rc = (n == (lfs_ssize_t)len && crc >= 0) ? 0 : -1;
    }
    if (rc == 0) fatrd_external_forget(path);
    fatrd_invalidate();
    return rc;
}

int vfs_msc_write_chunks(const char *path, uint32_t off, uint32_t size,
                         vfs_write_chunk_cb next, void *ctx)
{
    if (!next || off > size) return -1;

    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc = 0;

#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        /* Resize once so incremental sector writes do not repeatedly grow and
         * copy the RAMFS allocation. A failed callback leaves a partial file,
         * matching the failure semantics of the other write helpers. */
        if (ramfs_resize(leaf, size) != 0) return -1;
        for (uint32_t pos = off; pos < size;) {
            const void *data = NULL;
            uint32_t len = size - pos;
            if (!next(ctx, pos, &data, &len) || !data || !len || len > size - pos ||
                ramfs_write_at(leaf, pos, data, len) != 0) {
                rc = -1;
                break;
            }
            pos += len;
        }
    } else
#endif
    if (m->kind == VFS_BK_FAT) {
        /* The first stream for a file starts at zero (FA_CREATE_ALWAYS in the
         * backend); later streams append to the exact length committed by the
         * preceding one. This keeps the primitive backend interface intact. */
        for (uint32_t pos = off; pos < size;) {
            const void *data = NULL;
            uint32_t len = size - pos;
            if (!next(ctx, pos, &data, &len) || !data || !len || len > size - pos ||
                vfs_fat_wchunk(m->fatdrv, leaf, pos, data, len,
                               pos + len == size) != 0) {
                rc = -1;
                break;
            }
            pos += len;
        }
    } else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;

        struct lfs_file_config fcfg = { .buffer = s_file_cache };
        lfs_file_t f;
        fatrd_store_lock();
        if (lfs_file_opencfg(lfs, &f, leaf,
                             LFS_O_WRONLY | LFS_O_CREAT, &fcfg) < 0) {
            fatrd_store_unlock();
            return -1;
        }
        /* Offset zero is replacement, not an in-place patch. Drop the prior
         * tail before allocating new copy-on-write blocks; otherwise a nearly
         * full LittleFS needs space for both the old large file and its new
         * first checkpoint even when the replacement is much smaller. */
        if (off == 0 && lfs_file_truncate(lfs, &f, 0) < 0) rc = -1;
        if (off && lfs_file_seek(lfs, &f, off, LFS_SEEK_SET) < 0) rc = -1;

        for (uint32_t pos = off; rc == 0 && pos < size;) {
            const void *data = NULL;
            uint32_t len = size - pos;
            if (!next(ctx, pos, &data, &len) || !data || !len || len > size - pos) {
                rc = -1;
                break;
            }
            lfs_ssize_t n = lfs_file_write(lfs, &f, data, len);
            if (n != (lfs_ssize_t)len) {
                rc = -1;
                break;
            }
            pos += len;
        }
        if (rc == 0 && lfs_file_truncate(lfs, &f, size) < 0) rc = -1;
        if (lfs_file_close(lfs, &f) < 0) rc = -1;
        fatrd_store_unlock();
    }

    fatrd_invalidate_msc();
    return rc;
}

/* Set a file's length to exactly `size` (creating it if absent): shrinking drops the
 * tail, growing zero-fills. Used by the MSC reconcile so an overwrite that makes a
 * file smaller doesn't leave the old, longer tail readable. */
int vfs_truncate(const char *path, uint32_t size)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        rc = ramfs_resize(leaf, size);
    } else
#endif
    if (m->kind == VFS_BK_FAT) {
        return -1;   /* the FAT (card) backend maintains its own real filesystem - the
                      * MSC reconcile never truncates through here (card writes pass through) */
    } else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        struct lfs_file_config fcfg = { .buffer = s_file_cache };
        lfs_file_t f;
        fatrd_store_lock();
        if (lfs_file_opencfg(lfs, &f, leaf, LFS_O_WRONLY | LFS_O_CREAT, &fcfg) < 0) { fatrd_store_unlock(); return -1; }
        int trc = lfs_file_truncate(lfs, &f, size);
        int crc = lfs_file_close(lfs, &f);
        fatrd_store_unlock();
        rc = (trc >= 0 && crc >= 0) ? 0 : -1;
    }
    if (rc == 0) fatrd_external_forget(path);
    fatrd_invalidate();
    return rc;
}

int32_t vfs_size(const char *path)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS)
        return ramfs_size(leaf);
#endif
    if (m->kind == VFS_BK_FAT)
        return vfs_fat_size(m->fatdrv, leaf);
    lfs_t *lfs = vfs_mount_lfs(m);
    if (!lfs) return -1;
    struct lfs_info info;
    fatrd_store_lock();
    int srv = lfs_stat(lfs, leaf, &info);
    fatrd_store_unlock();
    if (srv < 0) return -1;
    return (info.type == LFS_TYPE_REG) ? (int32_t)info.size : -1;
}

static int vfs_remove_origin(const char *path, bool from_msc)
{
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) rc = ramfs_remove(leaf);
    else
#endif
    if (m->kind == VFS_BK_FAT) rc = vfs_fat_remove(m->fatdrv, leaf);
    else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        fatrd_store_lock();
        rc = lfs_remove(lfs, leaf) < 0 ? -1 : 0;   /* lfs_remove also drops empty dirs */
        fatrd_store_unlock();
    }
    if (from_msc) {
        fatrd_invalidate_msc();
    } else {
        if (rc == 0) fatrd_external_forget(path);
        fatrd_invalidate();
    }
    return rc;
}

int vfs_remove(const char *path)
{ return vfs_remove_origin(path, false); }

int vfs_msc_remove(const char *path)
{ return vfs_remove_origin(path, true); }

static int vfs_mkdir_origin(const char *path, bool from_msc)
{
    /* /mnt is a synthetic container for the external mounts, not a real dir -
     * accept but never materialise it in the internal FS (it would then shadow
     * the synthetic one and list twice). */
    if (path && strcmp(path, VFS_EXT_MOUNT) == 0) return 0;
    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
    int rc;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) return -1;   /* ramfs is flat - no subdirectories */
#endif
    if (m->kind == VFS_BK_FAT) {
        rc = vfs_fat_mkdir(m->fatdrv, leaf);
    } else {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        fatrd_store_lock();
        int mrc = lfs_mkdir(lfs, leaf);
        fatrd_store_unlock();
        rc = (mrc == 0 || mrc == LFS_ERR_EXIST) ? 0 : -1;
    }
    if (from_msc) {
        fatrd_invalidate_msc();
    } else {
        fatrd_invalidate();
    }
    return rc;
}

int vfs_mkdir(const char *path)
{ return vfs_mkdir_origin(path, false); }

int vfs_msc_mkdir(const char *path)
{ return vfs_mkdir_origin(path, true); }

static int vfs_rename_origin(const char *src, const char *dst, bool from_msc)
{
    const char *sl, *dl;
    const vfs_mount_t *ms = vfs_resolve(src, &sl);
    const vfs_mount_t *md = vfs_resolve(dst, &dl);
    if (!ms || !md) return -1;
    if (ms != md) return VFS_ERR_XDEV;            /* different backends - caller copies */
    int rc;
#if HAS_RAMFS
    if (ms->kind == VFS_BK_RAMFS) {
        rc = ramfs_rename(sl, dl);
    } else
#endif
    if (ms->kind == VFS_BK_FAT) {
        rc = vfs_fat_rename(ms->fatdrv, sl, dl);
    } else {
        lfs_t *lfs = vfs_mount_lfs(ms);
        if (!lfs) return -1;
        fatrd_store_lock();
        rc = lfs_rename(lfs, sl, dl) < 0 ? -1 : 0;
        fatrd_store_unlock();
    }
    if (from_msc) {
        fatrd_invalidate_msc();
    } else {
        if (rc == 0) fatrd_external_rename(src, dst);
        fatrd_invalidate();
    }
    return rc;
}

int vfs_rename(const char *src, const char *dst)
{ return vfs_rename_origin(src, dst, false); }

int vfs_msc_rename(const char *src, const char *dst)
{ return vfs_rename_origin(src, dst, true); }

int vfs_mount_count(void)
{
    vfs_init_mounts();
    return s_nmounts;
}

int vfs_statfs(int index, const char **path, uint32_t *total, uint32_t *freeb, bool *is_ram)
{
    vfs_init_mounts();
    if (index < 0 || index >= s_nmounts) return -1;
    const vfs_mount_t *m = &s_mounts[index];
    if (path)   *path   = m->prefix;
    if (is_ram) *is_ram = (m->kind == VFS_BK_RAMFS);
    if (total)  *total  = 0;
    if (freeb)  *freeb  = 0;
    if (m->kind == VFS_BK_LFS) {
        lfs_t *lfs = vfs_mount_lfs(m);
        if (!lfs) return -1;
        uint32_t bs  = lfs->cfg->block_size;
        uint32_t tot = lfs->block_count * bs;
        fatrd_store_lock();
        lfs_ssize_t ub = lfs_fs_size(lfs);   /* walks the whole FS - must not race a write */
        fatrd_store_unlock();
        uint32_t used = (ub < 0) ? 0 : (uint32_t)ub * bs;
        if (used > tot) used = tot;
        if (total) *total = tot >> 10;         /* KiB (see vfs.h) */
        if (freeb) *freeb = (tot - used) >> 10;
    } else if (m->kind == VFS_BK_FAT) {
        vfs_fat_statfs(m->fatdrv, total, freeb);
    }
    return 0;
}

#if HAS_RAMFS
typedef struct { vfs_list_cb cb; void *ctx; } list_adapt_t;
static void ramfs_list_adapt(const char *name, uint32_t size, void *vctx)
{
    list_adapt_t *a = vctx;
    a->cb(name, size, false, a->ctx);
}
#endif

int vfs_list(const char *path, vfs_list_cb cb, void *ctx)
{
    vfs_init_mounts();

    /* Synthetic /mnt: list the mounted external devices (ext0, ext1, ...). */
    if (path && (strcmp(path, VFS_EXT_MOUNT) == 0 || strcmp(path, VFS_EXT_MOUNT "/") == 0)) {
        for (int i = 0; i < s_nmounts; i++)
            if (is_ext_mount(&s_mounts[i]))
                cb(s_mounts[i].prefix + 5, 0, true, ctx);   /* "extN" (skip "/mnt/") */
        return 0;
    }

    const char *leaf;
    const vfs_mount_t *m = vfs_resolve(path, &leaf);
    if (!m) return -1;
#if HAS_RAMFS
    if (m->kind == VFS_BK_RAMFS) {
        /* RAMFS is deliberately flat. Only the mount itself is a directory;
         * a file or missing leaf must not masquerade as another view of root. */
        if (leaf && leaf[0]) return -1;
        list_adapt_t a = { cb, ctx };
        ramfs_iterate(ramfs_list_adapt, &a);
        return 0;
    }
#endif
    if (m->kind == VFS_BK_FAT) {
        return vfs_fat_list(m->fatdrv, leaf, cb, ctx);
    }
    lfs_t *lfs = vfs_mount_lfs(m);
    if (!lfs) return -1;

    /* /ramfs and /mnt are synthetic mounts, not real LittleFS entries, so they
     * never appear in the internal root read below. Inject them when listing the
     * root so every consumer (proto `ls`, the MSC view) sees them. */
    bool internal_root = (m->prefix[0] == '/' && m->prefix[1] == '\0') &&
                         path && (path[0] == '\0' || (path[0] == '/' && path[1] == '\0'));
    lfs_dir_t dir;
    fatrd_store_lock();
    if (lfs_dir_open(lfs, &dir, leaf) < 0) {
        fatrd_store_unlock();
        return -1;
    }

    if (internal_root) {
#if HAS_RAMFS
        fatrd_store_unlock();
        cb(VFS_RAMFS_MOUNT + 1, 0, true, ctx);   /* "ramfs" */
        fatrd_store_lock();
#endif
        if (s_next_ext > 0) {
            fatrd_store_unlock();
            cb(VFS_EXT_MOUNT + 1, 0, true, ctx);  /* "mnt" */
            fatrd_store_lock();
        }
    }

    struct lfs_info info;
    int rc = 0;
    for (;;) {
        int drc = lfs_dir_read(lfs, &dir, &info);
        if (drc <= 0) { if (drc < 0) rc = -1; break; }
        if (info.name[0] == '.' && (info.name[1] == '\0' ||
            (info.name[1] == '.' && info.name[2] == '\0')))
            continue;
        /* At the real internal root, hide any dir that collides with a synthetic
         * mount name (ramfs / mnt): those are injected above, and a stray real
         * dir of the same name would otherwise show up twice. */
        if (internal_root &&
            (strcmp(info.name, VFS_RAMFS_MOUNT + 1) == 0 ||
             strcmp(info.name, VFS_EXT_MOUNT + 1) == 0))
            continue;
        /* Drop the lock across the emit: cb() may stream a proto response that waits
         * on the TinyUSB task to drain its FIFO, and that task can be blocked on this
         * lock. `info` is a local copy, so it stays valid while unlocked. */
        fatrd_store_unlock();
        cb(info.name, info.type == LFS_TYPE_REG ? info.size : 0,
           info.type == LFS_TYPE_DIR, ctx);
        fatrd_store_lock();
    }
    lfs_dir_close(lfs, &dir);
    fatrd_store_unlock();
    return rc;
}
