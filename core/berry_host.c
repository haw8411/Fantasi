/* Berry scripting host.
 *
 * Embeds the Berry VM (third_party/berry) in the firmware so apps and `launch`
 * can run Berry scripts. This file is the platform port: it routes Berry's
 * allocation to the FreeRTOS heap, its output to the CLI, and provides the be_*
 * file/abort hooks the interpreter links against (the file hooks are backed by
 * the VFS). be_exec() runs a script from a path, and berry_install_hardware()
 * exposes the `hardware` module. */
#include "berry.h"
#include "be_sys.h"     /* bdirinfo */
#include "cli.h"
#include "log.h"
#include "vfs.h"
#include "app_run.h"    /* hal_app_buttons / hal_app_led for the hardware module */
#include "app_api.h"    /* FANTASI_BTN_* */

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ---- Allocation over the FreeRTOS heap ----
 * heap_4 has no realloc, so we size-track: an 8-byte header (keeps the returned
 * pointer 8-aligned) holds the block size for realloc's copy. */
#define BE_HDR 8

void *fantasi_be_malloc(unsigned long n)
{
    if (!n) return NULL;
    char *p = pvPortMalloc(n + BE_HDR);
    if (!p) return NULL;
    *(size_t *)p = (size_t)n;
    return p + BE_HDR;
}

void fantasi_be_free(void *p)
{
    if (p) vPortFree((char *)p - BE_HDR);
}

void *fantasi_be_realloc(void *p, unsigned long n)
{
    if (!n) { fantasi_be_free(p); return NULL; }
    if (!p) return fantasi_be_malloc(n);
    size_t old = *(size_t *)((char *)p - BE_HDR);
    void *np = fantasi_be_malloc(n);
    if (!np) return NULL;
    memcpy(np, p, old < n ? old : n);
    fantasi_be_free(p);
    return np;
}

void fantasi_be_abort(void)
{
    fantasi_log(LOG_ERROR, "berry: fatal abort");
    for (;;) vTaskDelay(portMAX_DELAY);
}

/* ---- Berry I/O port ---- */
void be_writebuffer(const char *buffer, size_t length)
{
    char tmp[128];
    while (length) {
        size_t n = length < sizeof(tmp) - 1 ? length : sizeof(tmp) - 1;
        memcpy(tmp, buffer, n);
        tmp[n] = '\0';
        cli_write(tmp);
        buffer += n;
        length -= n;
    }
}

char *be_readstring(char *buffer, size_t size) { (void)buffer; (void)size; return NULL; }

/* ---- File-system port, backed by the VFS (ramfs + LittleFS) ----
 * The VFS is whole-file (read_all / write_file); Berry wants a stdio-ish handle,
 * so a read handle consumes the file and serves reads from RAM, and a write handle
 * buffers and flushes on close. Handles use the FreeRTOS heap directly, kept off
 * Berry's GC heap. */
typedef struct {
    int      write;       /* 0 = read, 1 = write */
    uint8_t *buf;         /* read: vfs_read_all data; write: growable buffer */
    int      owned;       /* read: free with vfs_free; write: pvPortMalloc'd */
    uint32_t size;        /* valid bytes */
    uint32_t cap;         /* write capacity */
    uint32_t pos;
    char     path[128];   /* write-back target */
} be_file;

size_t be_fwrite(void *hfile, const void *buffer, size_t length)
{
    be_file *f = hfile;
    if (!f || !f->write) return 0;
    if (f->pos + length > f->cap) {
        uint32_t nc = f->cap ? f->cap : 256;
        while (nc < f->pos + length) nc *= 2;
        uint8_t *nb = pvPortMalloc(nc);
        if (!nb) return 0;
        memcpy(nb, f->buf, f->size);
        vPortFree(f->buf);
        f->buf = nb; f->cap = nc;
    }
    memcpy(f->buf + f->pos, buffer, length);
    f->pos += length;
    if (f->pos > f->size) f->size = f->pos;
    return length;
}

void *be_fopen(const char *filename, const char *modes)
{
    if (!filename || !modes) return NULL;
    be_file *f = pvPortMalloc(sizeof *f);
    if (!f) return NULL;
    memset(f, 0, sizeof *f);
    f->write = (modes[0] == 'w' || modes[0] == 'a');

    if (!f->write) {                          /* read: consume the whole file */
        const uint8_t *data; uint32_t len; bool owned;
        if (vfs_read_all(filename, &data, &len, &owned) != 0) { vPortFree(f); return NULL; }
        f->buf = (uint8_t *)data; f->owned = owned; f->size = len;
    } else {                                  /* write: buffer, flush on close */
        f->cap = 256;
        f->buf = pvPortMalloc(f->cap);
        if (!f->buf) { vPortFree(f); return NULL; }
        f->owned = 1;
        strncpy(f->path, filename, sizeof(f->path) - 1);
        if (modes[0] == 'a') {                /* append: preload existing content */
            const uint8_t *data; uint32_t len; bool owned;
            if (vfs_read_all(filename, &data, &len, &owned) == 0) {
                be_fwrite(f, data, len);
                if (owned) vfs_free(data);
            }
        }
    }
    return f;
}

int be_fclose(void *hfile)
{
    be_file *f = hfile;
    if (!f) return -1;
    int rc = 0;
    if (f->write) { rc = vfs_write_file(f->path, f->buf, f->size); vPortFree(f->buf); }
    else if (f->owned) vfs_free(f->buf);
    vPortFree(f);
    return rc;
}

size_t be_fread(void *hfile, void *buffer, size_t length)
{
    be_file *f = hfile;
    if (!f) return 0;
    uint32_t avail = f->size - f->pos;
    if (length > avail) length = avail;
    memcpy(buffer, f->buf + f->pos, length);
    f->pos += length;
    return length;
}

char *be_fgets(void *hfile, void *buffer, int size)
{
    be_file *f = hfile;
    char *b = buffer;
    if (!f || size <= 0 || f->pos >= f->size) return NULL;
    int i = 0;
    while (i < size - 1 && f->pos < f->size) {
        char c = (char)f->buf[f->pos++];
        b[i++] = c;
        if (c == '\n') break;
    }
    b[i] = '\0';
    return b;
}

int be_fseek(void *hfile, long offset)
{
    be_file *f = hfile;
    if (!f) return -1;
    if (offset < 0) offset = 0;
    if ((uint32_t)offset > f->size) offset = (long)f->size;
    f->pos = (uint32_t)offset;
    return 0;
}

long   be_ftell(void *hfile) { be_file *f = hfile; return f ? (long)f->pos : -1; }
long   be_fflush(void *hfile) { (void)hfile; return 0; }   /* write-back is on close */
size_t be_fsize(void *hfile) { be_file *f = hfile; return f ? f->size : 0; }

/* ---- path queries ---- */
int be_isfile(const char *path) { return vfs_size(path) >= 0; }

static void isdir_cb(const char *n, uint32_t s, bool d, void *ctx) { (void)n; (void)s; (void)d; *(int *)ctx = 1; }
int be_isdir(const char *path)
{
    if (vfs_size(path) >= 0) return 0;    /* a regular file */
    int found = 0;
    vfs_list(path, isdir_cb, &found);     /* enumerable -> a directory */
    return found;
}
int be_isexist(const char *path) { return be_isfile(path) || be_isdir(path); }

char *be_getcwd(char *buf, size_t size) { if (size) { buf[0] = '/'; buf[1] = '\0'; } return buf; }
int   be_chdir(const char *path) { (void)path; return 0; }   /* no cwd; paths are absolute */
int   be_mkdir(const char *path) { return vfs_mkdir(path); }
int   be_unlink(const char *path) { return vfs_remove(path); }

/* ---- directory iteration (over vfs_list) ---- */
typedef struct { char *names; uint32_t len, cap, cur; } be_dir;
static void dir_cb(const char *n, uint32_t s, bool d, void *ctx)
{
    (void)s; (void)d;
    be_dir *bd = ctx;
    uint32_t nl = (uint32_t)strlen(n) + 1;
    if (bd->len + nl > bd->cap) {
        uint32_t nc = bd->cap ? bd->cap : 128;
        while (nc < bd->len + nl) nc *= 2;
        char *nb = pvPortMalloc(nc);
        if (!nb) return;
        if (bd->names) { memcpy(nb, bd->names, bd->len); vPortFree(bd->names); }
        bd->names = nb; bd->cap = nc;
    }
    memcpy(bd->names + bd->len, n, nl);
    bd->len += nl;
}
int be_dirnext(bdirinfo *info)
{
    be_dir *bd = info->dir;
    if (!bd || bd->cur >= bd->len) { info->name = ""; return 1; }   /* nonzero = end */
    info->name = bd->names + bd->cur;
    bd->cur += (uint32_t)strlen(info->name) + 1;
    return 0;
}
int be_dirfirst(bdirinfo *info, const char *path)
{
    be_dir *bd = pvPortMalloc(sizeof *bd);
    if (!bd) return 1;
    memset(bd, 0, sizeof *bd);
    vfs_list(path, dir_cb, bd);
    info->dir = bd;
    info->name = "";
    return be_dirnext(info);
}
int be_dirclose(bdirinfo *info)
{
    be_dir *bd = info->dir;
    if (bd) { if (bd->names) vPortFree(bd->names); vPortFree(bd); info->dir = NULL; }
    return 0;
}

/* ---------------------------------------------------------------------------
 * The `hardware` Berry module - buttons + LEDs, the firmware's own I/O. Provided
 * as a runtime-registered global (like an app's hid module) so it needs no
 * be_modtab/coc machinery. be_exec() installs it for `launch foo.be`; apps that
 * build their own VM can install it too (berry_install_hardware is loader-
 * resolvable). Event loop: hardware.loop(fn) calls fn(button_mask) on each press
 * until fn returns false.
 * ------------------------------------------------------------------------- */
static int m_hw_led(bvm *vm)
{
    hal_app_led((uint8_t)be_toint(vm, 1), (uint8_t)be_toint(vm, 2), (uint8_t)be_toint(vm, 3));
    be_return_nil(vm);
}

static int m_hw_buttons(bvm *vm)
{
    be_pushint(vm, (bint)hal_app_buttons());
    be_return(vm);
}

static int m_hw_delay(bvm *vm)
{
    vTaskDelay(pdMS_TO_TICKS((uint32_t)be_toint(vm, 1)));
    be_return_nil(vm);
}

/* hardware.loop(fn): poll buttons (edge-detected); call fn(mask) on each press.
 * Stops when the callback returns false (or errors). */
static int m_hw_loop(bvm *vm)
{
    uint32_t prev = hal_app_buttons();
    for (;;) {
        uint32_t now = hal_app_buttons();
        uint32_t pressed = now & ~prev;
        prev = now;
        if (pressed) {
            be_pushvalue(vm, 1);                 /* the callback (arg 1) */
            be_pushint(vm, (bint)pressed);
            int err = be_pcall(vm, 1);
            /* be_pcall stores the result in the callee's slot and leaves the
             * argument in place: result is at -2, the pushed arg at -1. */
            int stop = err                                     /* error -> stop */
                       || (be_isbool(vm, -2) && !be_tobool(vm, -2)); /* false -> stop */
            be_pop(vm, 2);                        /* drop result + arg (stay balanced) */
            if (stop) break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    be_return_nil(vm);
}

static void hw_add_fn(bvm *vm, const char *name, bntvfunc fn)
{
    be_pushntvfunction(vm, fn);
    be_setmember(vm, -2, name);
    be_pop(vm, 1);
}
static void hw_add_int(bvm *vm, const char *name, bint v)
{
    be_pushint(vm, v);
    be_setmember(vm, -2, name);
    be_pop(vm, 1);
}

void berry_install_hardware(bvm *vm)
{
    be_newmodule(vm);
    hw_add_fn(vm, "led",     m_hw_led);
    hw_add_fn(vm, "buttons", m_hw_buttons);
    hw_add_fn(vm, "delay",   m_hw_delay);
    hw_add_fn(vm, "loop",    m_hw_loop);
    hw_add_int(vm, "OK",    FANTASI_BTN_OK);
    hw_add_int(vm, "BACK",  FANTASI_BTN_BACK);
    hw_add_int(vm, "UP",    FANTASI_BTN_UP);
    hw_add_int(vm, "DOWN",  FANTASI_BTN_DOWN);
    hw_add_int(vm, "LEFT",  FANTASI_BTN_LEFT);
    hw_add_int(vm, "RIGHT", FANTASI_BTN_RIGHT);
    be_setglobal(vm, "hardware");
    be_pop(vm, 1);
}

/* Resolve a Berry public-API symbol for the ELF loader (core/elf_loader.c calls
 * this via a weak hook). Lets a freestanding app #include berry.h and embed
 * scripts - build a VM, register native modules, drive its own hardware - with
 * the VM implementation living once in firmware. Taking each address also forces
 * the function into the firmware image past --gc-sections. */
uint32_t berry_resolve_api(const char *name)
{
    static const struct { const char *n; uint32_t a; } api[] = {
        { "be_vm_new",          (uint32_t)(uintptr_t)be_vm_new },
        { "be_vm_delete",       (uint32_t)(uintptr_t)be_vm_delete },
        { "be_loadmode",        (uint32_t)(uintptr_t)be_loadmode },
        { "be_loadbuffer",      (uint32_t)(uintptr_t)be_loadbuffer },
        { "be_pcall",           (uint32_t)(uintptr_t)be_pcall },
        { "be_top",             (uint32_t)(uintptr_t)be_top },
        { "be_pop",             (uint32_t)(uintptr_t)be_pop },
        { "be_tostring",        (uint32_t)(uintptr_t)be_tostring },
        { "be_toint",           (uint32_t)(uintptr_t)be_toint },
        { "be_toreal",          (uint32_t)(uintptr_t)be_toreal },
        { "be_tobool",          (uint32_t)(uintptr_t)be_tobool },
        { "be_isnil",           (uint32_t)(uintptr_t)be_isnil },
        { "be_isbool",          (uint32_t)(uintptr_t)be_isbool },
        { "be_isint",           (uint32_t)(uintptr_t)be_isint },
        { "be_isreal",          (uint32_t)(uintptr_t)be_isreal },
        { "be_isnumber",        (uint32_t)(uintptr_t)be_isnumber },
        { "be_isstring",        (uint32_t)(uintptr_t)be_isstring },
        { "be_isinstance",      (uint32_t)(uintptr_t)be_isinstance },
        { "be_pushnil",         (uint32_t)(uintptr_t)be_pushnil },
        { "be_pushbool",        (uint32_t)(uintptr_t)be_pushbool },
        { "be_pushint",         (uint32_t)(uintptr_t)be_pushint },
        { "be_pushreal",        (uint32_t)(uintptr_t)be_pushreal },
        { "be_pushstring",      (uint32_t)(uintptr_t)be_pushstring },
        { "be_pushvalue",       (uint32_t)(uintptr_t)be_pushvalue },
        { "be_pushntvfunction", (uint32_t)(uintptr_t)be_pushntvfunction },
        { "be_returnvalue",     (uint32_t)(uintptr_t)be_returnvalue },
        { "be_returnnilvalue",  (uint32_t)(uintptr_t)be_returnnilvalue },
        { "be_newmodule",       (uint32_t)(uintptr_t)be_newmodule },
        { "be_setmember",       (uint32_t)(uintptr_t)be_setmember },
        { "be_setglobal",       (uint32_t)(uintptr_t)be_setglobal },
        { "be_getglobal",       (uint32_t)(uintptr_t)be_getglobal },
        { "be_regfunc",         (uint32_t)(uintptr_t)be_regfunc },
        { "be_module_path_set", (uint32_t)(uintptr_t)be_module_path_set },
        { "berry_install_hardware", (uint32_t)(uintptr_t)berry_install_hardware },
    };
    for (unsigned i = 0; i < sizeof(api) / sizeof(api[0]); i++)
        if (strcmp(name, api[i].n) == 0) return api[i].a;
    return 0;
}

/* Load + run a Berry script file (via the VFS-backed be_fopen) on a fresh VM.
 * The simple entry for `launch foo.be` and script-only apps - firmware modules
 * only. Apps needing their own native modules build the VM themselves. */
int be_exec(const char *path)
{
    bvm *vm = be_vm_new();
    if (!vm) { cli_write("berry: vm_new failed (OOM)\r\n"); return -1; }

    berry_install_hardware(vm);

    int rc = be_loadfile(vm, path);
    if (rc == 0) rc = be_pcall(vm, 0);
    if (rc != 0) {
        const char *err = be_tostring(vm, -1);
        cli_printf("berry: %s\r\n", err ? err : "error");
    }

    be_vm_delete(vm);
    return rc;
}
