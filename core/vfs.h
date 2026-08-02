/* vfs - minimal path router between the RAM filesystem (/ramfs) and the on-flash
 * LittleFS (everything else). The proto file handlers branch on vfs_is_ramfs();
 * the app loader reads images through vfs_read_all() regardless of backend. */
#ifndef CORE_VFS_H
#define CORE_VFS_H

#include <stdint.h>
#include <stdbool.h>

#define VFS_RAMFS_MOUNT "/ramfs"

/* True for "/ramfs" or any "/ramfs/..." path. */
bool        vfs_is_ramfs(const char *path);

/* The leaf name after "/ramfs/" (empty string for the mount root itself).
 * Returns NULL if `path` is not a ramfs path. */
const char *vfs_ramfs_leaf(const char *path);

/* Read an entire file (ramfs or LittleFS) into memory for the loader. On success
 * returns 0 with data and len set; owned is true when data is a heap copy the
 * caller must release with vfs_free() (LittleFS), false for a borrowed in-place
 * ramfs pointer. Returns -1 on error. */
int  vfs_read_all(const char *path, const uint8_t **data, uint32_t *len, bool *owned);
void vfs_free(const uint8_t *data);

/* Whole-file helpers (ramfs or LittleFS) for the app storage API. */
int32_t vfs_pread(const char *path, uint32_t off, void *buf, uint32_t len); /* bytes, or -1 */
int32_t vfs_read_file(const char *path, void *buf, uint32_t max);   /* bytes, or -1 */
int     vfs_write_file(const char *path, const void *buf, uint32_t len); /* 0, or -1 */
int     vfs_append(const char *path, const void *buf, uint32_t len);     /* append; 0, or -1 */
int32_t vfs_size(const char *path);                                 /* bytes, or -1 */
int     vfs_remove(const char *path);                               /* 0, or -1 (also rmdir, empty) */
int     vfs_mkdir(const char *path);                                /* 0 (or already-exists), or -1 */

/* Enumerate a directory (ramfs root or a LittleFS dir). The callback gets each
 * entry's leaf name and size (0 for subdirectories). */
typedef void (*vfs_list_cb)(const char *name, uint32_t size, bool is_dir, void *ctx);
void    vfs_list(const char *path, vfs_list_cb cb, void *ctx);

#endif /* CORE_VFS_H */
