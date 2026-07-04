#include "cli.h"
#include "../hal/hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CLI_TLS_SLOT    0
#define CLI_ARGC_MAX    16
#define CLI_PRINT_BUF   256
#define CLI_PROMPT      "fantasi> "

/* ---- Per-task context access ---- */

cli_ctx_t *cli_current_ctx(void)
{
    return (cli_ctx_t *)pvTaskGetThreadLocalStoragePointer(NULL, CLI_TLS_SLOT);
}

void cli_bind_ctx(cli_ctx_t *ctx)
{
    vTaskSetThreadLocalStoragePointer(NULL, CLI_TLS_SLOT, ctx);
}

/* ---- Output ---- */

static void cli_write_raw(cli_ctx_t *ctx, const uint8_t *p, size_t len)
{
    /* Bound the wait for transport drain. transport.write() returns 0 while the
     * TX FIFO is full - and stays 0 forever if the host/peer stopped reading or
     * disconnected (e.g. the USB serial is open elsewhere, or a BLE peer went
     * away). Retrying unconditionally hung this task indefinitely, stalling the
     * CLI: it could never get back to servicing input. Give up after ~250 ms of
     * no progress and drop the rest - losing output to a dead reader is far
     * better than locking the CLI. */
    int stalls = 0;
    while (len) {
        /* Bail at once if the link is gone - don't sit here draining into a dead
         * reader (the host closed the port / the BLE peer left). */
        if (ctx->transport.connected && !ctx->transport.connected(ctx->transport.ctx))
            return;
        size_t wrote = ctx->transport.write(p, len, ctx->transport.ctx);
        if (wrote == 0) {
            if (++stalls > 250) return;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        stalls = 0;
        p += wrote;
        len -= wrote;
    }
}

void cli_write(const char *s)
{
    cli_ctx_t *ctx = cli_current_ctx();
    if (!ctx) return;
    const uint8_t *p = (const uint8_t *)s;
    size_t len = strlen(s);
    /* Strip \x06 (framing sentinel) so commands can never produce it. */
    const uint8_t *end = p + len;
    while (p < end) {
        const uint8_t *sentinel = memchr(p, 0x06, end - p);
        size_t chunk = sentinel ? (size_t)(sentinel - p) : (size_t)(end - p);
        if (chunk) cli_write_raw(ctx, p, chunk);
        p += chunk + (sentinel ? 1 : 0);
    }
}

int cli_printf(const char *fmt, ...)
{
    char buf[CLI_PRINT_BUF];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    cli_write(buf);
    return n;
}

static void prompt(void)
{
    cli_write(CLI_PROMPT);
}

/* ---- Command dispatch ---- */

static int tokenize(char *line, char **argv, int max)
{
    int argc = 0;
    char *p = line;
    while (*p && argc < max) {
        while (*p == ' ' || *p == '\t') *p++ = '\0';
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
    }
    return argc;
}

static void cli_frame(uint8_t tag)
{
    cli_ctx_t *ctx = cli_current_ctx();
    if (!ctx) return;
    uint8_t f[2] = { 0x06, tag };
    cli_write_raw(ctx, f, 2);
}

const cli_command_t *cli_lookup(const char *name)
{
    for (const cli_command_t *c = __start_cli_cmd; c < __stop_cli_cmd; c++)
        if (strcmp(name, c->name) == 0)
            return c;
    return NULL;
}

static void dispatch(char *line)
{
    char *argv[CLI_ARGC_MAX];
    int argc = tokenize(line, argv, CLI_ARGC_MAX);
    if (argc == 0) return;

    cli_frame(0x01);
    const cli_command_t *cmd = cli_lookup(argv[0]);
    if (cmd) {
        cmd->fn(argc, argv);
    } else {
        cli_printf("unknown command: %s\r\n", argv[0]);
    }
    cli_frame(0x02);
}

/* ---- Byte processing ---- */

static void handle_byte(cli_ctx_t *ctx, uint8_t b)
{
    if (b == '\r' || b == '\n') {
        cli_write("\r\n");
        if (ctx->line_len > 0) {
            ctx->line_buf[ctx->line_len] = '\0';
            dispatch(ctx->line_buf);
            ctx->line_len = 0;
        }
        prompt();
        return;
    }
    if (b == 0x7F || b == 0x08) {
        if (ctx->line_len > 0) {
            ctx->line_len--;
            cli_write("\b \b");
        }
        return;
    }
    if (b < 0x20 || b > 0x7E) return;
    if (ctx->line_len + 1 >= CLI_LINE_MAX) return;

    ctx->line_buf[ctx->line_len++] = (char)b;
    uint8_t echo = b;
    ctx->transport.write(&echo, 1, ctx->transport.ctx);
}

/* ---- Task entry points ---- */

void cli_task_with_transport(void *arg)
{
    cli_ctx_t *ctx = (cli_ctx_t *)arg;
    cli_bind_ctx(ctx);

    while (!ctx->transport.connected(ctx->transport.ctx)) {
        if (ctx->transport.poll) ctx->transport.poll();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!ctx->transport.poll) {
        cli_write("\r\nFantasi CLI ready. Type 'help' for commands.\r\n");
        prompt();
    }

    uint8_t buf[32];
    for (;;) {
        if (ctx->transport.poll) ctx->transport.poll();
        size_t n = ctx->transport.read(buf, sizeof(buf), ctx->transport.ctx);
        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        for (size_t i = 0; i < n; i++) handle_byte(ctx, buf[i]);
    }
}

/* USB transport wrappers matching cli_transport_t signature */
static size_t usb_write(const uint8_t *buf, size_t len, void *c) {
    (void)c; return hal_serial_write(buf, len);
}
static size_t usb_read(uint8_t *buf, size_t len, void *c) {
    (void)c; return hal_serial_read(buf, len);
}
static bool usb_connected(void *c) {
    (void)c; return hal_serial_connected();
}

static cli_ctx_t usb_ctx;

void cli_task(void *arg)
{
    (void)arg;

    usb_ctx.transport.write     = usb_write;
    usb_ctx.transport.read      = usb_read;
    usb_ctx.transport.connected = usb_connected;
    usb_ctx.transport.ctx       = NULL;

    hal_post_init();

    cli_task_with_transport(&usb_ctx);
}
