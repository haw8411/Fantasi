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
#if HAS_RAMFS
    if (vfs_is_ramfs(path)) {
        const char *leaf = vfs_ramfs_leaf(path);
        uint32_t n = 0;
        const uint8_t *p = ramfs_get(leaf, &n);
        if (!p) return -1;
        *data = p; *len = n; *owned = false;
        return 0;
    }
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;

    static uint8_t fcache[256];
    struct lfs_file_config fcfg = { .buffer = fcache };
    lfs_file_t f;
    if (lfs_file_opencfg(lfs, &f, path, LFS_O_RDONLY, &fcfg) < 0) return -1;

    lfs_ssize_t sz = lfs_file_size(lfs, &f);
    if (sz < 0) { lfs_file_close(lfs, &f); return -1; }

    uint8_t *buf = pvPortMalloc((size_t)sz ? (size_t)sz : 1);
    if (!buf) { lfs_file_close(lfs, &f); return -1; }

    lfs_ssize_t got = lfs_file_read(lfs, &f, buf, (lfs_size_t)sz);
    lfs_file_close(lfs, &f);
    if (got != sz) { vPortFree(buf); return -1; }

    *data = buf; *len = (uint32_t)sz; *owned = true;
    return 0;
}

void vfs_free(const uint8_t *data)
{
    if (data) vPortFree((void *)data);
}

int32_t vfs_pread(const char *path, uint32_t off, void *buf, uint32_t len)
{
#if HAS_RAMFS
    if (vfs_is_ramfs(path))
        return ramfs_read(vfs_ramfs_leaf(path), off, buf, len);
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;
    static uint8_t fc[256];
    struct lfs_file_config fcfg = { .buffer = fc };
    lfs_file_t f;
    if (lfs_file_opencfg(lfs, &f, path, LFS_O_RDONLY, &fcfg) < 0) return -1;
    if (off) lfs_file_seek(lfs, &f, off, LFS_SEEK_SET);
    lfs_ssize_t n = lfs_file_read(lfs, &f, buf, len);
    lfs_file_close(lfs, &f);
    return n < 0 ? -1 : (int32_t)n;
}

int32_t vfs_read_file(const char *path, void *buf, uint32_t max)
{
    return vfs_pread(path, 0, buf, max);
}

int vfs_write_file(const char *path, const void *buf, uint32_t len)
{
    fatrd_invalidate();          /* the synthetic-FAT model must reflect this */
#if HAS_RAMFS
    if (vfs_is_ramfs(path)) {
        const char *leaf = vfs_ramfs_leaf(path);
        if (ramfs_truncate(leaf) != 0) return -1;
        return ramfs_write_at(leaf, 0, buf, len);
    }
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;
    static uint8_t fc[256];
    struct lfs_file_config fcfg = { .buffer = fc };
    lfs_file_t f;
    if (lfs_file_opencfg(lfs, &f, path,
                         LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &fcfg) < 0) return -1;
    lfs_ssize_t n = lfs_file_write(lfs, &f, buf, len);
    int crc = lfs_file_close(lfs, &f);
    return (n == (lfs_ssize_t)len && crc >= 0) ? 0 : -1;
}

/* Append `buf` to the end of `path` (created if absent). Lets a module stream a large log incrementally
 * without buffering it all - the whole-file write_file would truncate. */
int vfs_append(const char *path, const void *buf, uint32_t len)
{
    fatrd_invalidate();
#if HAS_RAMFS
    if (vfs_is_ramfs(path)) {
        const char *leaf = vfs_ramfs_leaf(path);
        int32_t sz = ramfs_size(leaf); if (sz < 0) sz = 0;
        return ramfs_write_at(leaf, (uint32_t)sz, buf, len);
    }
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;
    static uint8_t fc[256];
    struct lfs_file_config fcfg = { .buffer = fc };
    lfs_file_t f;
    if (lfs_file_opencfg(lfs, &f, path,
                         LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND, &fcfg) < 0) return -1;
    lfs_ssize_t n = lfs_file_write(lfs, &f, buf, len);
    int crc = lfs_file_close(lfs, &f);
    return (n == (lfs_ssize_t)len && crc >= 0) ? 0 : -1;
}

int32_t vfs_size(const char *path)
{
#if HAS_RAMFS
    if (vfs_is_ramfs(path))
        return ramfs_size(vfs_ramfs_leaf(path));
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;
    struct lfs_info info;
    if (lfs_stat(lfs, path, &info) < 0) return -1;
    return (info.type == LFS_TYPE_REG) ? (int32_t)info.size : -1;
}

int vfs_remove(const char *path)
{
    fatrd_invalidate();
#if HAS_RAMFS
    if (vfs_is_ramfs(path))
        return ramfs_remove(vfs_ramfs_leaf(path));
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;
    return lfs_remove(lfs, path) < 0 ? -1 : 0;   /* lfs_remove also drops empty dirs */
}

int vfs_mkdir(const char *path)
{
    fatrd_invalidate();
#if HAS_RAMFS
    if (vfs_is_ramfs(path)) return -1;   /* ramfs is flat - no subdirectories */
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return -1;
    int rc = lfs_mkdir(lfs, path);
    return (rc == 0 || rc == LFS_ERR_EXIST) ? 0 : -1;
}

#if HAS_RAMFS
typedef struct { vfs_list_cb cb; void *ctx; } list_adapt_t;
static void ramfs_list_adapt(const char *name, uint32_t size, void *vctx)
{
    list_adapt_t *a = vctx;
    a->cb(name, size, false, a->ctx);
}
#endif

void vfs_list(const char *path, vfs_list_cb cb, void *ctx)
{
#if HAS_RAMFS
    if (vfs_is_ramfs(path)) {
        list_adapt_t a = { cb, ctx };
        ramfs_iterate(ramfs_list_adapt, &a);
        return;
    }
    /* /ramfs is a synthetic RAM mount, not a real LittleFS entry, so it never
     * appears in the directory read below. Surface it when listing the root so
     * every consumer (proto `ls`, etc.) matches the MSC view, which injects it
     * in build_model(). */
    if (path && (path[0] == '\0' || (path[0] == '/' && path[1] == '\0')))
        cb(VFS_RAMFS_MOUNT + 1, 0, true, ctx);   /* leaf "ramfs", is_dir */
#endif
    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) return;
    lfs_dir_t dir;
    if (lfs_dir_open(lfs, &dir, path) < 0) return;
    struct lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
        if (info.name[0] == '.' && (info.name[1] == '\0' ||
            (info.name[1] == '.' && info.name[2] == '\0')))
            continue;
        cb(info.name, info.type == LFS_TYPE_REG ? info.size : 0,
           info.type == LFS_TYPE_DIR, ctx);
    }
    lfs_dir_close(lfs, &dir);
}
