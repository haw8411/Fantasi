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
#define FANTASI_APP_ABI 2

/* Button bitmask bits returned by api->buttons(). Not every device has every
 * button; absent buttons simply never set their bit. */
#define FANTASI_BTN_OK     (1u << 0)
#define FANTASI_BTN_BACK   (1u << 1)
#define FANTASI_BTN_UP     (1u << 2)
#define FANTASI_BTN_DOWN   (1u << 3)
#define FANTASI_BTN_LEFT   (1u << 4)
#define FANTASI_BTN_RIGHT  (1u << 5)

/* ---- USB HID keyboard emulation (ABI >= 2) ---- */
/* Modifier bits for api->hid_send() - standard USB HID keyboard usage. */
#define FANTASI_HID_LCTRL   0x01
#define FANTASI_HID_LSHIFT  0x02
#define FANTASI_HID_LALT    0x04
#define FANTASI_HID_LGUI    0x08
#define FANTASI_HID_RCTRL   0x10
#define FANTASI_HID_RSHIFT  0x20
#define FANTASI_HID_RALT    0x40
#define FANTASI_HID_RGUI    0x80

/* Host hint bits returned by api->hid_host(). The low bits mirror the host's
 * last keyboard-LED output report, which is the cheap "is anyone listening /
 * which OS" signal a Ducky-style payload can branch on. */
#define FANTASI_HID_LED_NUMLOCK     0x01
#define FANTASI_HID_LED_CAPSLOCK    0x02
#define FANTASI_HID_LED_SCROLLLOCK  0x04
#define FANTASI_HID_HOST_MOUNTED    0x100   /* a host has enumerated the keyboard */

/* Directory-entry callback for api->list_dir(). is_dir is nonzero for a
 * subdirectory (size is then 0). */
typedef void (*fantasi_dirent_fn)(const char *name, uint32_t size, int is_dir, void *ctx);

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

    /* api->abi_version = 2 */

    /* ---- USB HID keyboard emulation ---- */
    /* Arm the keyboard for typing (on=1) or release held keys + disarm (on=0).
     * On the composite targets (FZ/CU/Kiisu) the keyboard is always enumerated
     * alongside CDC/MSC, so on=1 just waits until a host is bound; the
     * endpoint-scarce PM3 re-enumerates as a HID-only device here. Blocks until
     * ready or a timeout. Returns 0 on success, -1 if this device can't emulate
     * HID (or no host is present). */
    int (*hid_mode)(int on);
    /* Send one keyboard report: `modifiers` (FANTASI_HID_* bits) held together
     * with up to `n` (<=6) simultaneous key usages in `keys`. Pass n=0 to
     * release all keys. Waits for the previous report to drain first. A keypress
     * is press-then-release: two calls. Returns 0, or -1 (HID down / timeout). */
    int (*hid_send)(uint8_t modifiers, const uint8_t *keys, uint8_t n);
    /* Host hint bits (FANTASI_HID_LED_* | FANTASI_HID_HOST_*). 0 when unknown. */
    uint32_t (*hid_host)(void);

    /* ---- Directory listing ---- */
    /* Enumerate `path` (e.g. "/scripts", "/apps", "/ramfs"), invoking cb per
     * entry. Returns 0, or -1 on a bad path / no such directory. */
    int (*list_dir)(const char *path, fantasi_dirent_fn cb, void *ctx);
    /* Create a directory (and it's fine if it already exists). Not supported on
     * the flat ramfs mount. Returns 0, or -1. */
    int (*mkdir)(const char *path);

    /* ---- Berry scripting ---- */
    /* Run the Berry script at `path` (a VFS path) on a fresh VM with the firmware
     * modules. The simple entry for apps that just execute a script; apps that
     * need to expose their own native module build the VM directly against Berry's
     * C API (the loader resolves be_*). Returns 0, or -1 (error, or no Berry). */
    int (*be_exec)(const char *path);
} fantasi_api_t;

/* The app's entry point. Its return value is reported as the exit code. */
int app_main(const fantasi_api_t *api);

#endif /* FANTASI_APP_API_H */
