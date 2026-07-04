/* Fantasi app ABI - the contract between a loadable app and the firmware.
 *
 * A loadable app is a freestanding, relocatable ARM object. Its single entry
 * point is app_main(); everything it needs from the firmware (console, heap,
 * storage, hardware) arrives through the fantasi_api_t function table passed in.
 * Apps link against nothing else - no libc, no firmware symbols. A function
 * pointer may be NULL when the running device lacks that capability (e.g. the
 * display on a device without a screen); check before calling. */
#ifndef FANTASI_APP_API_H
#define FANTASI_APP_API_H

#include <stdint.h>
#include <stddef.h>

/* Bumped whenever the struct layout below changes. New members are only ever
 * appended, so existing field offsets stay stable; an app should check
 * api->abi_version before using a member added in a later ABI than it was built
 * against. */
#define FANTASI_APP_ABI 1

/* Button bitmask bits returned by api->buttons(). Not every device has every
 * button; absent buttons simply never set their bit. */
#define FANTASI_BTN_OK     (1u << 0)
#define FANTASI_BTN_BACK   (1u << 1)
#define FANTASI_BTN_UP     (1u << 2)
#define FANTASI_BTN_DOWN   (1u << 3)
#define FANTASI_BTN_LEFT   (1u << 4)
#define FANTASI_BTN_RIGHT  (1u << 5)

typedef struct fantasi_api {
    uint16_t abi_version;

    /* ---- Console (streamed to the launching CLI) ---- */
    int  (*print)(const char *s);
    int  (*printf)(const char *fmt, ...);

    /* ---- Heap (tracked; anything still allocated is reclaimed at exit) ---- */
    void *(*malloc)(size_t n);
    void  (*free)(void *p);

    /* ---- Storage (full paths: /ramfs/.., /apps/.., /..) ---- */
    int32_t (*read_file)(const char *path, void *buf, uint32_t max);   /* bytes, or -1 */
    int     (*write_file)(const char *path, const void *buf, uint32_t len); /* 0, or -1 */
    int32_t (*file_size)(const char *path);                            /* bytes, or -1 */
    int     (*remove)(const char *path);                               /* 0, or -1 */

    /* ---- Hardware (a pointer is NULL where unsupported on this device) ---- */
    void     (*led)(uint8_t r, uint8_t g, uint8_t b);   /* best-effort RGB */
    uint32_t (*buttons)(void);                          /* FANTASI_BTN_* bitmask */
    void     (*display_clear)(void);                    /* screen-only devices */
    void     (*display_print)(int col, int row, const char *s);
    void     (*display_flush)(void);

    /* ---- Uninterruptible regions (tight hardware timing) ---- */
    void (*critical_enter)(void);
    void (*critical_exit)(void);

    /* ---- Timing ---- */
    void (*delay)(uint32_t ms);   /* sleep this app for ms; yields the CPU so
                                   * other tasks (and low-power idle) can run. */
} fantasi_api_t;

/* The app's entry point. Its return value is reported as the exit code. */
int app_main(const fantasi_api_t *api);

#endif /* FANTASI_APP_API_H */
