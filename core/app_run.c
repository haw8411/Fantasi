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

#define APP_TASK_STACK_WORDS 1024   /* 4 KB - covers vsnprintf etc. */
#define APP_OUT_BUF          1024
#define APP_DRAIN_CHUNK      128

/* Header prepended to every api->malloc block so the launcher can reclaim
 * whatever the app leaves allocated when it exits. */
typedef struct app_alloc { struct app_alloc *next; } app_alloc_t;

typedef struct {
    uint32_t          entry;
    StreamBufferHandle_t out;
    SemaphoreHandle_t alloc_lock;
    app_alloc_t      *allocs;
    const fantasi_api_t *api;
    TaskHandle_t      task;
    volatile bool     done;
    volatile bool     kill_req;   /* set by the `kill` command (another channel) */
    volatile int      exit_code;
} app_ctx_t;

/* Launches are serialized (the streaming session owns the CLI), so one current
 * app context suffices and the API callbacks find it here. */
static app_ctx_t *g_app;

/* Cross-channel kill: the `kill` command (running on a DIFFERENT CLI/proto task
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

/* PID of the running app as `ps` shows it, or -1 if none. `ps` prints
 * TaskStatus_t.xTaskNumber (creation order), which is a different field from
 * uxTaskGetTaskNumber() - so look the app's handle up in the system state to
 * report the SAME number, letting `kill <pid>` match what the user sees. */
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

static int32_t api_read_file(const char *path, void *buf, uint32_t max) { return vfs_read_file(path, buf, max); }
static int     api_write_file(const char *path, const void *buf, uint32_t len) { return vfs_write_file(path, buf, len); }
static int32_t api_file_size(const char *path) { return vfs_size(path); }
static int     api_remove(const char *path) { return vfs_remove(path); }
static void    api_crit_enter(void) { taskENTER_CRITICAL(); }
static void    api_crit_exit(void)  { taskEXIT_CRITICAL(); }
static void    api_delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

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

    const uint8_t *data; uint32_t len; bool owned;
    if (vfs_read_all(path, &data, &len, &owned) != 0) {
        cli_printf("launch: not found: %s\r\n", path);
        launch_busy = false;
        return -1;
    }

    static app_image_t img;
    int lr = app_load(data, len, &img);
    if (owned) vfs_free(data);
    if (lr != 0) {
        cli_printf("launch: load failed: %s\r\n", app_load_error());
        launch_busy = false;
        return -1;
    }

    static app_ctx_t actx;
    memset(&actx, 0, sizeof(actx));
    actx.entry = img.entry;
    actx.out = xStreamBufferCreate(APP_OUT_BUF, 1);
    actx.alloc_lock = xSemaphoreCreateMutex();

    static fantasi_api_t api;
    if (actx.out && actx.alloc_lock) {
        api = (fantasi_api_t){
            .abi_version = FANTASI_APP_ABI,
            .print = api_print, .printf = api_printf,
            .malloc = api_malloc, .free = api_free,
            .read_file = api_read_file, .write_file = api_write_file,
            .file_size = api_file_size, .remove = api_remove,
            .led = hal_app_led, .buttons = hal_app_buttons,
            .display_clear = hal_app_has_display() ? hal_app_display_clear : NULL,
            .display_print = hal_app_has_display() ? hal_app_display_print : NULL,
            .display_flush = hal_app_has_display() ? hal_app_display_flush : NULL,
            .critical_enter = api_crit_enter, .critical_exit = api_crit_exit,
            .delay = api_delay,
        };
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
            uint8_t ch;
            if (ctx->transport.read(&ch, 1, ctx->transport.ctx) > 0 && ch == 0x03) {
                aborted = true;
                break;
            }
            drain_output(&actx);
            if (ctx->transport.flush) ctx->transport.flush();
            if (actx.done) break;
            if (actx.kill_req) { aborted = true; break; }   /* `kill` from another channel */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        /* Stop the app cleanly BEFORE deleting it. A self-exited app is already
         * parked in vTaskSuspend (app_task_entry), which is why that path was
         * safe; a ^C-killed app is still running or blocked (e.g. in a stream
         * buffer send). vTaskSuspend removes it from the ready/delayed list AND
         * from any event list it's blocked on, so the delete and the stream
         * buffer / mutex teardown below can't dereference a still-queued task -
         * that dangling reference was wedging the PM3 on ^C-kill. */
        vTaskSuspend(actx.task);
        vTaskDelete(actx.task);
        drain_output(&actx);
    }

    /* App is done - give the screen back to the platform (restores status/splash). */
    if (owns_display) hal_app_display_release();

    /* Reclaim everything the app used - even if it leaked or was killed. */
    for (app_alloc_t *a = actx.allocs; a; ) { app_alloc_t *nx = a->next; vPortFree(a); a = nx; }
    if (actx.out) vStreamBufferDelete(actx.out);
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
