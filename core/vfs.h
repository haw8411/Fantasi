/* vfs - path router across storage backends: the RAM filesystem (/ramfs), the
 * internal on-flash LittleFS (everything else under /), and any external devices
 * mounted at /mnt/extN (LittleFS on the Proxmark5 QSPI flash; a Flipper SD card
 * later). Backends register in mount order; /mnt/ext0 is the first external
 * device, /mnt/ext1 the second, and so on. The proto file handlers and the app
 * loader route through here regardless of which backend holds a path. */
#ifndef CORE_VFS_H
#define CORE_VFS_H

#include <stdint.h>
#include <stdbool.h>

struct lfs;

#define VFS_RAMFS_MOUNT "/ramfs"
#define VFS_EXT_MOUNT   "/mnt"      /* external devices live under /mnt/extN */

/* Cross-device: vfs_rename can't move between two different backends. */
#define VFS_ERR_XDEV (-2)

/* Directory-listing callback: each entry's leaf name and size (0 for subdirs).
 * Declared up here so the FAT-backend hooks below can reference it. */
typedef void (*vfs_list_cb)(const char *name, uint32_t size, bool is_dir, void *ctx);

/* ---- Mount table ---- */
typedef enum { VFS_BK_RAMFS, VFS_BK_LFS, VFS_BK_FAT } vfs_backend_t;

typedef struct {
    const char   *prefix;   /* "/ramfs", "/", "/mnt/ext0", ... */
    vfs_backend_t kind;
    struct lfs   *lfs;       /* LFS backends; NULL means the internal on-demand lfs */
    int           fatdrv;    /* FAT backends: FatFs logical drive (0-9) */
} vfs_mount_t;

/* Register an external LittleFS instance as the next /mnt/extN (N in mount order).
 * Returns N (>= 0), or -1 if the table is full. */
int vfs_mount_ext_lfs(struct lfs *lfs);

/* Register an external FAT volume (e.g. a Flipper SD card) as the next /mnt/extN.
 * `drive` is the FatFs logical drive number already mounted with f_mount. The FAT
 * ops route through the weak vfs_fat_* hooks below, which the owning platform
 * overrides with a FatFs-backed implementation. Returns N (>= 0), or -1. */
int vfs_mount_ext_fat(int drive);

/* Weak FAT-backend interface. core/vfs.c provides failing stubs; a platform that
 * has a FAT volume (platforms/flipper/vfs_fat.c) overrides them, backed by FatFs.
 * `drv` is the mount's FatFs drive; `leaf` is the mount-relative path (lfs-rooted,
 * i.e. it begins with '/'). Keeping the interface in primitive types lets core/vfs.c
 * stay free of any FatFs dependency. */
int32_t vfs_fat_pread(int drv, const char *leaf, uint32_t off, void *buf, uint32_t len);
int32_t vfs_fat_size(int drv, const char *leaf);
int     vfs_fat_wchunk(int drv, const char *leaf, uint32_t off,
                       const void *buf, uint32_t len, bool last);
int     vfs_fat_remove(int drv, const char *leaf);
int     vfs_fat_mkdir(int drv, const char *leaf);
int     vfs_fat_rename(int drv, const char *from, const char *to);
int     vfs_fat_statfs(int drv, uint32_t *total, uint32_t *freeb);
void    vfs_fat_list(int drv, const char *leaf, vfs_list_cb cb, void *ctx);

/* Longest-prefix match. Returns the owning mount and, in *leaf_out, the path
 * relative to that mount to hand the backend (a flat name for ramfs, an
 * lfs-rooted path for LittleFS). Never NULL for an absolute path (/ is the
 * catch-all). */
const vfs_mount_t *vfs_resolve(const char *path, const char **leaf_out);

/* Backend accessors for callers that hold a resolved mount (e.g. the proto
 * handlers). vfs_mount_lfs resolves the internal mount's on-demand instance. */
bool         vfs_mount_is_ramfs(const vfs_mount_t *m);
bool         vfs_mount_is_fat(const vfs_mount_t *m);
struct lfs  *vfs_mount_lfs(const vfs_mount_t *m);

/* Mount enumeration + stats for `df`. vfs_statfs fills the mount path and, for
 * LittleFS/FAT mounts, total/free in KiB (is_ram true for /ramfs, which has no
 * fixed on-disk size). KiB rather than bytes so a multi-GB card fits uint32.
 * Returns 0, or -1 for an out-of-range/unavailable mount. */
int vfs_mount_count(void);
int vfs_statfs(int index, const char **path, uint32_t *total, uint32_t *freeb, bool *is_ram);

/* True for "/ramfs" or any "/ramfs/..." path. */
bool        vfs_is_ramfs(const char *path);

/* The leaf name after "/ramfs/" (empty string for the mount root itself).
 * Returns NULL if `path` is not a ramfs path. */
const char *vfs_ramfs_leaf(const char *path);

/* Read an entire file (any backend) into memory for the loader. On success
 * returns 0 with data and len set; owned is true when data is a heap copy the
 * caller must release with vfs_free() (LittleFS), false for a borrowed in-place
 * ramfs pointer. Returns -1 on error. */
int  vfs_read_all(const char *path, const uint8_t **data, uint32_t *len, bool *owned);
void vfs_free(const uint8_t *data);

/* Whole-file helpers (any backend) for the app storage API. */
int32_t vfs_pread(const char *path, uint32_t off, void *buf, uint32_t len); /* bytes, or -1 */
int32_t vfs_read_file(const char *path, void *buf, uint32_t max);   /* bytes, or -1 */
int     vfs_write_file(const char *path, const void *buf, uint32_t len); /* 0, or -1 */
int     vfs_append(const char *path, const void *buf, uint32_t len);     /* append; 0, or -1 */
int     vfs_pwrite(const char *path, uint32_t off, const void *buf, uint32_t len); /* positional; 0, or -1 */
int     vfs_truncate(const char *path, uint32_t size); /* set exact length; 0, or -1 */
int32_t vfs_size(const char *path);                                 /* bytes, or -1 */
int     vfs_remove(const char *path);                               /* 0, or -1 (also rmdir, empty) */
int     vfs_mkdir(const char *path);                                /* 0 (or already-exists), or -1 */

/* Rename/move within one backend. Returns 0, VFS_ERR_XDEV when src and dst are on
 * different mounts (caller must copy+delete), or -1 on other error. */
int     vfs_rename(const char *src, const char *dst);

/* Enumerate a directory (ramfs root, a LittleFS dir, or the synthetic /mnt).
 * vfs_list_cb is declared near the top so the FAT hooks can use it. */
void    vfs_list(const char *path, vfs_list_cb cb, void *ctx);

#endif /* CORE_VFS_H */
