/* ramfs - a small, RAM-backed filesystem mounted at /ramfs.
 *
 * Files are stored verbatim in contiguous heap buffers (no wear-leveling, no
 * block structure), so an uploaded image is already a contiguous in-RAM blob the
 * app loader can read directly. The store grows and shrinks with its contents:
 * a file's buffer is sized to the file (or to a reserved capacity, see
 * ramfs_reserve), and removing a file frees it.
 * It is flat (no subdirectories) and empty at boot. Names are leaf names (the
 * VFS strips the "/ramfs/" prefix) and may not contain spaces or slashes. */
#ifndef CORE_RAMFS_H
#define CORE_RAMFS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RAMFS_NAME_MAX  48
#define RAMFS_MAX_FILES 16

/* Empty an existing file or create a new empty one. Returns 0, or -1 if the file
 * table is full. */
int      ramfs_truncate(const char *name);

/* Set a file's length to `size` exactly (shrinking drops the tail, growing
 * zero-fills). Creates the file if absent. Returns 0, -1 on OOM or a full table. */
int      ramfs_resize(const char *name, uint32_t size);

/* Write len bytes at byte offset off, growing the file (and zero-filling any
 * gap) as needed. Creates the file if absent. Returns 0 on success, -1 on OOM or
 * a full table. */
int      ramfs_write_at(const char *name, uint32_t off, const void *data, uint32_t len);

/* Pre-grow a file's buffer to `cap` bytes without changing its logical size,
 * creating it if absent. A following write sequence up to `cap` then needs no
 * per-chunk reallocation - one allocation instead of a grow-by-copy that would
 * need ~2x the file live at once. Returns 0, -1 on OOM or a full table. */
int      ramfs_reserve(const char *name, uint32_t cap);

/* Copy up to len bytes from off into buf. Returns bytes copied, or -1 if absent. */
int32_t  ramfs_read(const char *name, uint32_t off, void *buf, uint32_t len);

/* File size, or -1 if absent. */
int32_t  ramfs_size(const char *name);

/* Borrowed pointer to the file's contiguous bytes (NULL if absent). Valid until
 * the file is written or removed. Used by the app loader to read in place. */
const uint8_t *ramfs_get(const char *name, uint32_t *len);

/* Remove a file-table entry without freeing its backing buffer and transfer
 * that allocation to the caller. Used to turn a transient module ELF into its
 * loaded image without requiring a second large contiguous heap block. */
int      ramfs_take(const char *name, uint8_t **data, uint32_t *len);

int      ramfs_remove(const char *name);

/* Rename by relabeling the existing entry; the data buffer is not moved. */
int      ramfs_rename(const char *from, const char *to);

typedef void (*ramfs_iter_fn)(const char *name, uint32_t size, void *ctx);
void     ramfs_iterate(ramfs_iter_fn fn, void *ctx);

#endif /* CORE_RAMFS_H */
