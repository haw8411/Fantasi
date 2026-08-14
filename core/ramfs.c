#include "ramfs.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

typedef struct {
    char     name[RAMFS_NAME_MAX];
    uint8_t *buf;        /* `cap` bytes allocated, or NULL when cap == 0 */
    uint32_t size;       /* logical file size (<= cap) */
    uint32_t cap;        /* allocated buffer capacity */
    bool     used;
} ramfs_file_t;

static ramfs_file_t s_files[RAMFS_MAX_FILES];

/* ramfs is reachable from the proto/CLI task (uploads) and from a running app's
 * own task (api storage calls); a mutex keeps the table + buffers consistent. */
static SemaphoreHandle_t s_lock;

static void lock(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

static ramfs_file_t *find(const char *name)
{
    for (int i = 0; i < RAMFS_MAX_FILES; i++)
        if (s_files[i].used && strncmp(s_files[i].name, name, RAMFS_NAME_MAX) == 0)
            return &s_files[i];
    return NULL;
}

static ramfs_file_t *alloc_entry(const char *name)
{
    for (int i = 0; i < RAMFS_MAX_FILES; i++) {
        if (!s_files[i].used) {
            s_files[i].used = true;
            s_files[i].buf  = NULL;
            s_files[i].size = 0;
            s_files[i].cap  = 0;
            strncpy(s_files[i].name, name, RAMFS_NAME_MAX - 1);
            s_files[i].name[RAMFS_NAME_MAX - 1] = '\0';
            return &s_files[i];
        }
    }
    return NULL;
}

static void free_entry(ramfs_file_t *f)
{
    if (f->buf) vPortFree(f->buf);
    f->buf = NULL;
    f->size = 0;
    f->cap = 0;
    f->used = false;
}

/* Grow the backing buffer to at least `want` bytes (heap_4 has no realloc),
 * preserving the logical content. A no-op when capacity already suffices - so a
 * caller that reserves the final size up front (ramfs_reserve) turns the whole
 * write sequence into a single allocation instead of a realloc-per-chunk
 * grow-by-copy, which needs ~2x the file live at once and overflows a tight
 * heap (e.g. the PM3's ~25 KB). Returns 0 / -1 on OOM. */
static int ensure_cap(ramfs_file_t *f, uint32_t want)
{
    if (want <= f->cap) return 0;
    uint8_t *nb = pvPortMalloc(want);
    if (!nb) return -1;
    if (f->buf) { memcpy(nb, f->buf, f->size); vPortFree(f->buf); }
    f->buf = nb;
    f->cap = want;
    return 0;
}

/* Set the logical size to `want`, zero-filling growth. Returns 0 / -1 on OOM. */
static int resize(ramfs_file_t *f, uint32_t want)
{
    if (want == 0) {
        if (f->buf) vPortFree(f->buf);
        f->buf = NULL; f->size = 0; f->cap = 0;
        return 0;
    }
    if (ensure_cap(f, want) != 0) return -1;
    if (want > f->size) memset(f->buf + f->size, 0, want - f->size);
    f->size = want;
    return 0;
}

int ramfs_truncate(const char *name)
{
    lock();
    ramfs_file_t *f = find(name);
    if (!f) f = alloc_entry(name);
    if (!f) { unlock(); return -1; }
    resize(f, 0);
    unlock();
    return 0;
}

int ramfs_resize(const char *name, uint32_t size)
{
    lock();
    ramfs_file_t *f = find(name);
    if (!f) f = alloc_entry(name);
    if (!f) { unlock(); return -1; }
    int rc = resize(f, size);
    unlock();
    return rc;
}

int ramfs_reserve(const char *name, uint32_t cap)
{
    lock();
    ramfs_file_t *f = find(name);
    if (!f) f = alloc_entry(name);
    if (!f) { unlock(); return -1; }
    int rc = ensure_cap(f, cap);
    unlock();
    return rc;
}

int ramfs_write_at(const char *name, uint32_t off, const void *data, uint32_t len)
{
    lock();
    ramfs_file_t *f = find(name);
    if (!f) f = alloc_entry(name);
    if (!f) { unlock(); return -1; }

    uint32_t need = off + len;
    if (need > f->size && resize(f, need) != 0) { unlock(); return -1; }
    if (len) memcpy(f->buf + off, data, len);
    unlock();
    return 0;
}

int32_t ramfs_read(const char *name, uint32_t off, void *buf, uint32_t len)
{
    lock();
    ramfs_file_t *f = find(name);
    if (!f) { unlock(); return -1; }
    if (off >= f->size) { unlock(); return 0; }
    uint32_t n = f->size - off;
    if (n > len) n = len;
    memcpy(buf, f->buf + off, n);
    unlock();
    return (int32_t)n;
}

int32_t ramfs_size(const char *name)
{
    lock();
    ramfs_file_t *f = find(name);
    int32_t r = f ? (int32_t)f->size : -1;
    unlock();
    return r;
}

const uint8_t *ramfs_get(const char *name, uint32_t *len)
{
    /* No lock held on return: callers (the loader) use this single-threaded
     * while no concurrent writer to the same file exists. */
    ramfs_file_t *f = find(name);
    if (!f) return NULL;
    if (len) *len = f->size;
    return f->buf;
}

int ramfs_remove(const char *name)
{
    lock();
    ramfs_file_t *f = find(name);
    if (!f) { unlock(); return -1; }
    free_entry(f);
    unlock();
    return 0;
}

void ramfs_iterate(ramfs_iter_fn fn, void *ctx)
{
    lock();
    for (int i = 0; i < RAMFS_MAX_FILES; i++)
        if (s_files[i].used) fn(s_files[i].name, s_files[i].size, ctx);
    unlock();
}
