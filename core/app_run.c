#include "app_run.h"
#include "elf_loader.h"
#include "vfs.h"
#include "cli.h"
#include "app_api.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef APP_TASK_STACK_WORDS
#define APP_TASK_STACK_WORDS 1024   /* 4 KB default; a platform may override in FreeRTOSConfig.h */
#endif
#define APP_OUT_BUF          1024
#define APP_IN_BUF           128    /* console input queued for the app (keystrokes) */
#define APP_DRAIN_CHUNK      128

/* Header prepended to every api->malloc block so the launcher can reclaim
 * whatever the app leaves allocated when it exits. */
typedef struct app_alloc { struct app_alloc *next; } app_alloc_t;

typedef struct {
    uint32_t          entry;
    StreamBufferHandle_t out;
    StreamBufferHandle_t in;      /* console input -> app (api->read_input) */
    SemaphoreHandle_t alloc_lock;
    app_alloc_t      *allocs;
    const fantasi_api_t *api;
    TaskHandle_t      task;
    volatile bool     done;
    volatile bool     kill_req;   /* set by the `kill` command (another channel) */
    volatile int      exit_code;

    /* ---- async session (proto channels) ---- */
    bool                     async;    /* true: driven by a pump task, not a sync loop */
    const app_session_cb_t  *cb;       /* proto engine's emit callbacks */
    uint32_t                 session;  /* echoes the app_launch request id */
    TaskHandle_t             pump;     /* drains output + services module requests */
    volatile bool            req_mod;  /* app asked for a module (api->request_module) */
    char                     req_name[32];
} app_ctx_t;

/* Launches are serialized (the streaming session owns the CLI), so one current
 * app context suffices and the API callbacks find it here. */
static app_ctx_t *g_app;

/* Cross-channel kill: the `kill` command (running on a different CLI/proto task
 * than the one streaming the app) requests termination by setting a flag; the
 * launching session's loop sees it and does the single-owner teardown. Only the
 * app task is killable this way - never a system task. */
bool app_kill_running(void)
{
    app_ctx_t *a = g_app;
    if (!a || !a->task) return false;
    a->kill_req = true;
    return true;
}

/* Cheap "is an app loaded" check for the sleep-governance path. Unlike
 * app_running_pid() this touches no kernel state and no stack-heavy
 * TaskStatus_t array, so it is safe from the idle task / tickless hook. */
bool app_is_running(void)
{
    app_ctx_t *a = g_app;
    return a != NULL && a->task != NULL;
}

/* PID of the running app as `ps` shows it, or -1 if none. `ps` prints
 * TaskStatus_t.xTaskNumber (creation order), which is a different field from
 * uxTaskGetTaskNumber() - so look the app's handle up in the system state to
 * report the same number, letting `kill <pid>` match what the user sees. */
int app_running_pid(void)
{
    app_ctx_t *a = g_app;
    if (!a || !a->task) return -1;
    TaskStatus_t st[16];
    UBaseType_t n = uxTaskGetSystemState(st, sizeof(st) / sizeof(st[0]), NULL);
    for (UBaseType_t i = 0; i < n; i++)
        if (st[i].xHandle == a->task) return (int)st[i].xTaskNumber;
    return -1;
}

/* ---- Weak hardware hooks (platforms override what they support) ---- */
__attribute__((weak)) void hal_app_led(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
__attribute__((weak)) uint32_t hal_app_buttons(void) { return 0; }
__attribute__((weak)) bool hal_app_has_display(void) { return false; }
__attribute__((weak)) void hal_app_display_clear(void) {}
__attribute__((weak)) void hal_app_display_print(int col, int row, const char *s) { (void)col; (void)row; (void)s; }
__attribute__((weak)) void hal_app_display_flush(void) {}
/* Screen ownership: while an app runs, the platform must stop its own periodic
 * redraws (status/splash) so they don't paint over the app. release() restores
 * the normal screen. No-ops where unsupported; only invoked when has_display. */
__attribute__((weak)) void hal_app_display_acquire(void) {}
__attribute__((weak)) void hal_app_display_release(void) {}
/* USB HID keyboard: default to "unsupported" so devices/platforms without it
 * link cleanly and apps see -1 / 0 from the API. */
__attribute__((weak)) int hal_hid_enable(int on) { (void)on; return -1; }
__attribute__((weak)) int hal_hid_send(uint8_t modifiers, const uint8_t *keys, uint8_t n) { (void)modifiers; (void)keys; (void)n; return -1; }
__attribute__((weak)) uint32_t hal_hid_host(void) { return 0; }

/* ---- API callbacks (run on the app task) ---- */

static int api_print(const char *s)
{
    if (!g_app || !s) return 0;
    size_t n = strlen(s);
    if (n) xStreamBufferSend(g_app->out, s, n, portMAX_DELAY);
    return (int)n;
}

static int api_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) api_print(buf);
    return n;
}

static void *api_malloc(size_t n)
{
    app_alloc_t *h = pvPortMalloc(sizeof(app_alloc_t) + n);
    if (!h) return NULL;
    xSemaphoreTake(g_app->alloc_lock, portMAX_DELAY);
    h->next = g_app->allocs;
    g_app->allocs = h;
    xSemaphoreGive(g_app->alloc_lock);
    return h + 1;
}

static void api_free(void *p)
{
    if (!p) return;
    app_alloc_t *h = (app_alloc_t *)p - 1;
    xSemaphoreTake(g_app->alloc_lock, portMAX_DELAY);
    for (app_alloc_t **pp = &g_app->allocs; *pp; pp = &(*pp)->next) {
        if (*pp == h) { *pp = h->next; break; }
    }
    xSemaphoreGive(g_app->alloc_lock);
    vPortFree(h);
}

/* Read callback the ELF loader streams the app image through - ctx is the VFS path. */
static int32_t app_elf_read(void *ctx, uint32_t off, void *dst, uint32_t len)
{
    return vfs_pread((const char *)ctx, off, dst, len);
}

static int32_t api_read_file(const char *path, void *buf, uint32_t max) { return vfs_read_file(path, buf, max); }
static int32_t api_pread(const char *path, uint32_t off, void *buf, uint32_t max) { return vfs_pread(path, off, buf, max); }
static int api_append(const char *path, const void *buf, uint32_t len) { return vfs_append(path, buf, len); }
static int     api_write_file(const char *path, const void *buf, uint32_t len) { return vfs_write_file(path, buf, len); }
static int32_t api_file_size(const char *path) { return vfs_size(path); }
static int     api_remove(const char *path) { return vfs_remove(path); }
static void    api_crit_enter(void) { taskENTER_CRITICAL(); }
static void    api_crit_exit(void)  { taskEXIT_CRITICAL(); }
static void    api_delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/* Non-blocking read of console input the launcher forwarded (keystrokes). */
static int api_read_input(void *buf, uint32_t max)
{
    if (!g_app || !g_app->in || !buf || !max) return 0;
    return (int)xStreamBufferReceive(g_app->in, buf, max, 0);
}

/* Ask the host to stream a feature module (async sessions only). Records the
 * name; the pump task emits the module_request to the host, which answers with a
 * FileWriteChunk into /ramfs. The app then polls the VFS for it to appear. */
static void api_request_module(const char *name)
{
    if (!g_app || !g_app->async || !name) return;
    size_t i = 0;
    for (; name[i] && i < sizeof(g_app->req_name) - 1; i++) g_app->req_name[i] = name[i];
    g_app->req_name[i] = '\0';
    __asm volatile("" ::: "memory");   /* publish the name before the flag */
    g_app->req_mod = true;
}

static int      api_hid_mode(int on) { return hal_hid_enable(on); }
static int      api_hid_send(uint8_t mod, const uint8_t *keys, uint8_t n) { return hal_hid_send(mod, keys, n); }
static uint32_t api_hid_host(void) { return hal_hid_host(); }

/* Bridge the app's dirent callback (int is_dir) onto vfs_list's (bool is_dir). */
typedef struct { fantasi_dirent_fn cb; void *ctx; } dir_adapt_t;
static void app_dir_adapt(const char *name, uint32_t size, bool is_dir, void *vctx)
{
    dir_adapt_t *a = (dir_adapt_t *)vctx;
    a->cb(name, size, is_dir ? 1 : 0, a->ctx);
}
static int api_list_dir(const char *path, fantasi_dirent_fn cb, void *ctx)
{
    if (!cb) return -1;
    dir_adapt_t a = { cb, ctx };
    vfs_list(path, app_dir_adapt, &a);
    return 0;
}
static int api_mkdir(const char *path) { return vfs_mkdir(path); }

/* Berry runner - weak default so non-Berry builds link; core/berry_host.c
 * overrides it where the VM is compiled in. */
__attribute__((weak)) int be_exec(const char *path) { (void)path; return -1; }
static int api_be_exec(const char *path) { return be_exec(path); }

/* Fill the API table both launch paths (sync app_run + async session) hand apps. */
static void fill_api(fantasi_api_t *api)
{
    *api = (fantasi_api_t){
        .abi_version = FANTASI_APP_ABI,
        .print = api_print, .printf = api_printf,
        .malloc = api_malloc, .free = api_free,
        .read_file = api_read_file, .pread = api_pread, .write_file = api_write_file, .append = api_append,
        .file_size = api_file_size, .remove = api_remove,
        .led = hal_app_led, .buttons = hal_app_buttons,
        .display_clear = hal_app_has_display() ? hal_app_display_clear : NULL,
        .display_print = hal_app_has_display() ? hal_app_display_print : NULL,
        .display_flush = hal_app_has_display() ? hal_app_display_flush : NULL,
        .critical_enter = api_crit_enter, .critical_exit = api_crit_exit,
        .delay = api_delay,
        .hid_mode = api_hid_mode, .hid_send = api_hid_send, .hid_host = api_hid_host,
        .list_dir = api_list_dir, .mkdir = api_mkdir,
        .be_exec = api_be_exec,
        .read_input = api_read_input,
        .request_module = api_request_module,
    };
}

/* ---- App task ---- */

static void app_task_entry(void *arg)
{
    app_ctx_t *a = (app_ctx_t *)arg;
    int (*entry)(const fantasi_api_t *) = (int (*)(const fantasi_api_t *))a->entry;
    a->exit_code = entry(a->api);
    a->done = true;
    /* Park until the launcher deletes us, so it stays the sole deleter (no race
     * between self-deletion and a ^C kill). */
    for (;;) vTaskSuspend(NULL);
}

static void drain_output(app_ctx_t *a)
{
    char buf[APP_DRAIN_CHUNK];
    size_t n;
    while ((n = xStreamBufferReceive(a->out, buf, sizeof(buf) - 1, 0)) > 0) {
        buf[n] = '\0';
        cli_write(buf);
    }
}

/* ---- In-app dynamic module loader ----
 * Load an ELF module from the VFS, run its app_main on the calling task with the
 * caller's API table, then unload it. Used by a resident shell app to hot-load
 * feature modules one at a time (the in-app dynamic loader from docs/rfid.md).
 * No task, no launch_busy, no streaming loop: the module's app_main is a plain
 * synchronous call, and its console/heap go through the same api the shell was
 * given (so its output streams to the shell's session and any leak is reclaimed
 * when the shell exits). A separate static image keeps the shell's own loaded
 * image intact while the module runs (only one module runs at a time). */
int app_run_module(const char *path, const fantasi_api_t *api, bool delete_source)
{
    if (!path || !api) return -1;
    int32_t total = vfs_size(path);
    if (total < 0) return -1;

    static app_image_t mod_img;
    if (app_load(app_elf_read, (void *)path, (uint32_t)total, &mod_img) != 0)
        return -1;

    /* Caller opted to drop the source file now that the image is fully resident. app_load streams
     * every SHF_ALLOC section into its own RAM and applies all relocations before returning, through
     * stateless path-based reads - nothing touches the file again for the rest of this run. On a
     * RAM-backed VFS the file is heap (a module ELF is ~2 KB), so holding it for the module's
     * lifetime just denies that memory to the module itself - heap the tight PM3 sniff capture
     * path needs. Only reached on a successful load, so a failed load leaves the file alone. */
    if (delete_source) vfs_remove(path);

    int (*entry)(const fantasi_api_t *) = (int (*)(const fantasi_api_t *))mod_img.entry;
    int rc = entry(api);

    app_unload(&mod_img);
    return rc;
}

/* Loader symbol resolver so an app can call fantasi_run_module (weak default in
 * core/elf_loader.c). Always available where the app loader is compiled in. */
uint32_t app_resolve_api(const char *name)
{
    if (strcmp(name, "fantasi_run_module") == 0)
        return (uint32_t)(uintptr_t)app_run_module;
    return 0;
}

/* Claimed for the whole load-run-teardown span below. The static actx/img
 * state assumes one launch at a time; with several possible launchers (serial,
 * BLE, WebUSB, the on-device GUI) a second concurrent launch must fail cleanly
 * rather than corrupt the running app's state. */
static volatile bool launch_busy;

int app_run(const char *path)
{
    cli_ctx_t *ctx = cli_current_ctx();
    if (!ctx) return -1;

    bool claimed = false;
    taskENTER_CRITICAL();
    if (!launch_busy) launch_busy = claimed = true;
    taskEXIT_CRITICAL();
    if (!claimed) {
        cli_printf("launch: an app is already running (see ps / kill)\r\n");
        return -1;
    }

    int32_t total = vfs_size(path);
    if (total < 0) {
        cli_printf("launch: not found: %s\r\n", path);
        launch_busy = false;
        return -1;
    }

    /* Stream the ELF through app_elf_read (vfs_pread): the loader holds only the
     * loaded image plus small metadata, never the whole file, so large apps fit
     * in the PM3's heap for instance. */
    static app_image_t img;
    int lr = app_load(app_elf_read, (void *)path, (uint32_t)total, &img);
    if (lr != 0) {
        cli_printf("launch: load failed: %s\r\n", app_load_error());
        launch_busy = false;
        return -1;
    }
    /* app_load relocated every section into its own RAM; the ELF file is now dead weight
     * (~30 KB for the RFID driver) that would otherwise stay resident the whole run, competing
     * with the app's own heap (e.g. the sniff's 64 KB capture ring). Free transient /ramfs sources
     * - the host re-uploads on the next launch. Persistent apps (/apps, SD) stay put. */
    if (!strncmp(path, "/ramfs/", 7)) vfs_remove(path);

    static app_ctx_t actx;
    memset(&actx, 0, sizeof(actx));
    actx.entry = img.entry;
    actx.out = xStreamBufferCreate(APP_OUT_BUF, 1);
    actx.in  = xStreamBufferCreate(APP_IN_BUF, 1);
    actx.alloc_lock = xSemaphoreCreateMutex();

    static fantasi_api_t api;
    if (actx.out && actx.alloc_lock) {
        fill_api(&api);
        actx.api = &api;
    }

    g_app = &actx;

    /* Hand the screen to the app so the platform's periodic status/splash redraw
     * stops painting over it; released after the app exits (below). Gated on a
     * real display so screenless devices skip it entirely. */
    bool owns_display = hal_app_has_display();
    if (owns_display) hal_app_display_acquire();

    /* App task one notch below us if room, else equal (round-robin) - either way
     * this CLI task keeps pumping the transport and watching for ^C. */
    UBaseType_t myprio = uxTaskPriorityGet(NULL);
    UBaseType_t appprio = (myprio > tskIDLE_PRIORITY + 1) ? myprio - 1 : myprio;

    bool spawned = actx.out && actx.alloc_lock &&
        xTaskCreate(app_task_entry, "app", APP_TASK_STACK_WORDS,
                    &actx, appprio, &actx.task) == pdPASS;

    bool aborted = false;
    if (spawned) {
        for (;;) {
            if (ctx->transport.poll) ctx->transport.poll();
            /* Pump console input: ^C aborts the app; every other byte is handed to
             * the app via api->read_input, so a launched app can be interactive. */
            uint8_t chs[32];
            size_t got = ctx->transport.read(chs, sizeof(chs), ctx->transport.ctx);
            for (size_t i = 0; i < got; i++) {
                if (chs[i] == 0x03) { aborted = true; break; }
                if (actx.in) xStreamBufferSend(actx.in, &chs[i], 1, 0);
            }
            if (aborted) break;
            drain_output(&actx);
            if (ctx->transport.flush) ctx->transport.flush();
            if (actx.done) break;
            if (actx.kill_req) { aborted = true; break; }   /* `kill` from another channel */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        /* Stop the app cleanly before deleting it. A self-exited app is already
         * parked in vTaskSuspend (app_task_entry), which is why that path is
         * safe; a ^C-killed app is still running or blocked (e.g. in a stream
         * buffer send). vTaskSuspend removes it from the ready/delayed list and
         * from any event list it's blocked on, so the delete and the stream
         * buffer / mutex teardown below can't dereference a still-queued task. */
        vTaskSuspend(actx.task);
        vTaskDelete(actx.task);
        drain_output(&actx);
    }

    /* App is done - give the screen back to the platform (restores status/splash). */
    if (owns_display) hal_app_display_release();

    /* Safety net for HID apps: release any key the app left held (a payload
     * killed mid-keypress would otherwise stick a key down on the host) and, in
     * switch mode, disarm the keyboard if the app didn't. A no-op when HID is
     * unsupported, idle, or already disarmed - so it never re-enumerates for a
     * plain (non-HID) app. */
    hal_hid_enable(0);

    /* Reclaim everything the app used - even if it leaked or was killed. */
    for (app_alloc_t *a = actx.allocs; a; ) { app_alloc_t *nx = a->next; vPortFree(a); a = nx; }
    if (actx.out) vStreamBufferDelete(actx.out);
    if (actx.in)  vStreamBufferDelete(actx.in);
    if (actx.alloc_lock) vSemaphoreDelete(actx.alloc_lock);
    app_unload(&img);
    g_app = NULL;
    launch_busy = false;

    if (!spawned) { cli_printf("launch: spawn failed\r\n"); return -1; }

    int rc = aborted ? -1 : actx.exit_code;
    if (aborted) cli_printf(actx.kill_req ? "\r\n[killed]\r\n" : "\r\n^C aborted\r\n");
    else cli_printf("\r\nexit %d\r\n", rc);
    return rc;
}

/* ---- Async launch (protobuf channels) ----
 * Spawns the app plus a pump task and returns, so the proto RX loop stays free to
 * deliver modules / input / stop to the running app. The pump drains the app's
 * output to the host, forwards its module requests, and on exit tears the session
 * down. g_app_lock serialises the lifecycle (pump teardown) against the RX-task
 * accessors (feed_input / stop), which run on a different task. */
#define PUMP_STACK_WORDS 512

static app_image_t       s_async_img;
static SemaphoreHandle_t g_app_lock;

static void app_pump_entry(void *arg)
{
    app_ctx_t *a = (app_ctx_t *)arg;
    char buf[APP_DRAIN_CHUNK];
    size_t n;

    for (;;) {
        while ((n = xStreamBufferReceive(a->out, buf, sizeof(buf), 0)) > 0)
            a->cb->output(a->session, buf, n);
        if (a->req_mod) { __asm volatile("" ::: "memory"); a->cb->module_request(a->session, a->req_name); a->req_mod = false; }
        if (a->done || a->kill_req) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    vTaskSuspend(a->task);                                   /* stop the app writing */
    while ((n = xStreamBufferReceive(a->out, buf, sizeof(buf), 0)) > 0)
        a->cb->output(a->session, buf, n);                  /* flush the tail */
    int code = a->kill_req ? -1 : a->exit_code;

    xSemaphoreTake(g_app_lock, portMAX_DELAY);
    vTaskDelete(a->task);
    for (app_alloc_t *p = a->allocs; p; ) { app_alloc_t *nx = p->next; vPortFree(p); p = nx; }
    if (a->out) vStreamBufferDelete(a->out);
    if (a->in)  vStreamBufferDelete(a->in);
    if (a->alloc_lock) vSemaphoreDelete(a->alloc_lock);
    app_unload(&s_async_img);
    g_app = NULL;
    launch_busy = false;
    xSemaphoreGive(g_app_lock);

    a->cb->done(a->session, code);
    vTaskDelete(NULL);
}

int app_launch_async(const char *path, uint32_t session, const app_session_cb_t *cb)
{
    if (!path || !cb) return -1;
    if (!g_app_lock) g_app_lock = xSemaphoreCreateMutex();

    bool claimed = false;
    taskENTER_CRITICAL();
    if (!launch_busy) launch_busy = claimed = true;
    taskEXIT_CRITICAL();
    if (!claimed) return -1;                                 /* an app is already running */

    int32_t total = vfs_size(path);
    if (total < 0)                              { launch_busy = false; return -2; }   /* not found */
    if (app_load(app_elf_read, (void *)path, (uint32_t)total, &s_async_img) != 0) {
        launch_busy = false; return -3;                                               /* load error */
    }
    if (!strncmp(path, "/ramfs/", 7)) vfs_remove(path);   /* free transient ELF source (see app_run) */

    static app_ctx_t actx;
    memset(&actx, 0, sizeof(actx));
    actx.entry = s_async_img.entry;
    actx.out = xStreamBufferCreate(APP_OUT_BUF, 1);
    actx.in  = xStreamBufferCreate(APP_IN_BUF, 1);
    actx.alloc_lock = xSemaphoreCreateMutex();
    actx.async = true;
    actx.cb = cb;
    actx.session = session;

    static fantasi_api_t api;
    if (!actx.out || !actx.in || !actx.alloc_lock) { app_unload(&s_async_img); launch_busy = false; return -3; }
    fill_api(&api);
    actx.api = &api;
    g_app = &actx;

    UBaseType_t prio = tskIDLE_PRIORITY + 1;
    if (xTaskCreate(app_task_entry, "app", APP_TASK_STACK_WORDS, &actx, prio, &actx.task) != pdPASS ||
        xTaskCreate(app_pump_entry, "apppump", PUMP_STACK_WORDS, &actx, prio, &actx.pump) != pdPASS) {
        if (actx.task) vTaskDelete(actx.task);
        vStreamBufferDelete(actx.out); vStreamBufferDelete(actx.in); vSemaphoreDelete(actx.alloc_lock);
        app_unload(&s_async_img); g_app = NULL; launch_busy = false;
        return -3;
    }
    return 0;
}

void app_session_feed_input(const uint8_t *data, size_t n)
{
    if (!g_app_lock) return;
    xSemaphoreTake(g_app_lock, portMAX_DELAY);
    if (g_app && g_app->async && g_app->in) xStreamBufferSend(g_app->in, data, n, 0);
    xSemaphoreGive(g_app_lock);
}

void app_session_stop(void)
{
    if (!g_app_lock) return;
    xSemaphoreTake(g_app_lock, portMAX_DELAY);
    if (g_app && g_app->async) g_app->kill_req = true;
    xSemaphoreGive(g_app_lock);
}

bool app_session_active(void)
{
    return g_app != NULL && g_app->async;
}
