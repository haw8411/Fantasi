#ifndef FANTASI_CLI_H
#define FANTASI_CLI_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- Transport abstraction ---- */

typedef struct cli_transport {
    size_t (*write)(const uint8_t *buf, size_t len, void *ctx);
    size_t (*read)(uint8_t *buf, size_t len, void *ctx);
    bool   (*connected)(void *ctx);
    void   (*poll)(void);
    void   (*flush)(void);
    /* Optional: block until input may be available or timeout_ms elapsed.
     * NULL selects fixed-period polling. May return early; callers must
     * re-check read(). */
    void   (*wait)(uint32_t timeout_ms);
    void   *ctx;
} cli_transport_t;

#define CLI_LINE_MAX 128

typedef struct cli_ctx {
    char            line_buf[CLI_LINE_MAX];
    size_t          line_len;
    cli_transport_t transport;
} cli_ctx_t;

/* ---- Command registry ----
 *
 * Commands live one-per-file under core/commands/ and self-register with the
 * CLI_COMMAND() macro below - there is no central table. Each registration is
 * emitted into the "cli_cmd" linker section; the linker concatenates them and
 * provides __start_cli_cmd / __stop_cli_cmd bounding the array. The Makefiles
 * pick up every C file under core/commands/, so adding a command is
 * just dropping in a file and rebuilding. See core/commands/README.md. */

typedef int (*cli_cmd_fn)(int argc, char **argv);

typedef struct {
    const char *name;
    const char *help;
    cli_cmd_fn  fn;
} cli_command_t;

/* Register a command. `func` must be unique within its file (it names the
 * descriptor). `used` keeps it despite being unreferenced; placing it in the
 * cli_cmd section (referenced via __start_/__stop_ from a live root) keeps it
 * through --gc-sections. */
#define CLI_COMMAND(cmd_name, cmd_help, func)                       \
    static const cli_command_t _cli_cmd_##func                      \
        __attribute__((used, section("cli_cmd"), aligned(4))) =     \
        { (cmd_name), (cmd_help), (func) }

extern const cli_command_t __start_cli_cmd[];
extern const cli_command_t __stop_cli_cmd[];

/* Find a registered command by name, or NULL. */
const cli_command_t *cli_lookup(const char *name);

/* ---- Task entry points ---- */

void cli_task(void *arg);
void cli_task_with_transport(void *arg);

/* ---- Output (routes to the calling task's transport) ---- */

int  cli_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void cli_write(const char *s);
/* Publish output accumulated by a transport at an interactive/progress
 * boundary. Byte-stream transports may leave flush NULL, making this a no-op. */
void cli_flush(void);

/* ---- Context access for commands that need raw transport I/O ---- */

cli_ctx_t *cli_current_ctx(void);

/* Bind a cli_ctx to the calling task so cli_printf/cli_write (and anything
 * that needs a session, e.g. app_run) route to its transport. cli_task_with_
 * transport does this itself; other session owners (the on-device GUI) call
 * it once at task start. */
void cli_bind_ctx(cli_ctx_t *ctx);

#endif
