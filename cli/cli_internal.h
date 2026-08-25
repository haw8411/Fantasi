/* Internal interface shared between the host CLI core (main.c) and the per-command
 * implementations under cli/commands/. Commands self-register with LOCAL_COMMAND
 * / LOCAL_COMMAND_BLE (collected into the local_cmd linker section); main.c owns
 * the transport, FAT mount, and BLE protobuf plumbing and exposes the helpers the
 * commands call. See cli/commands/README.md. */
#ifndef FANTASI_CLI_INTERNAL_H
#define FANTASI_CLI_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef HAS_PROTO
#include "ble_transport.h"
#include "fantasi.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#endif

/* ANSI colors (help highlights local commands). */
#define C_RED     "\033[31m"
#define C_YELLOW  "\033[33m"
#define C_RESET   "\033[0m"

/* Framing sentinels around a command's output in the device's serial CLI. */
#define FRAME_SENTINEL 0x06
#define FRAME_START    0x01
#define FRAME_END      0x02

/* ---- Command registry ----
 *
 * One command per file under cli/commands/. Each self-registers into the
 * local_cmd linker section; main.c iterates __start_local_cmd..__stop_local_cmd.
 * The Makefile picks up new files via a wildcard over the commands directory. A command may
 * provide a separate BLE handler (used when the host talks to the device over
 * BLE instead of the MSC mount). */

typedef void (*local_fn)(const char *arg);

typedef struct {
    const char *name;
    const char *help;
    local_fn    fn;
#ifdef HAS_PROTO
    local_fn    proto_fn;   /* handler when use_ble is set; NULL falls back to fn */
#endif
} local_cmd_t;

#define _CLI_CAT(a, b)  a##b
#define _CLI_CAT2(a, b) _CLI_CAT(a, b)

#ifdef HAS_PROTO
#define LOCAL_COMMAND(nm, hp, usb_fn)                                   \
    static const local_cmd_t _CLI_CAT2(_lc_, __LINE__)                  \
        __attribute__((used, section("local_cmd"), aligned(8))) =       \
        { (nm), (hp), (usb_fn), 0 }
#define LOCAL_COMMAND_BLE(nm, hp, usb_fn, ble)                          \
    static const local_cmd_t _CLI_CAT2(_lc_, __LINE__)                  \
        __attribute__((used, section("local_cmd"), aligned(8))) =       \
        { (nm), (hp), (usb_fn), (ble) }
#else
#define LOCAL_COMMAND(nm, hp, usb_fn)                                   \
    static const local_cmd_t _CLI_CAT2(_lc_, __LINE__)                  \
        __attribute__((used, section("local_cmd"), aligned(8))) =       \
        { (nm), (hp), (usb_fn) }
#define LOCAL_COMMAND_BLE(nm, hp, usb_fn, ble)                          \
    static const local_cmd_t _CLI_CAT2(_lc_, __LINE__)                  \
        __attribute__((used, section("local_cmd"), aligned(8))) =       \
        { (nm), (hp), (usb_fn) }
#endif

extern const local_cmd_t __start_local_cmd[];
extern const local_cmd_t __stop_local_cmd[];

/* Match the first whitespace-delimited word of `line` to a registered command
 * (NULL if none). */
const local_cmd_t *cli_local_match(const char *line);

/* The exit/quit command: empty body; the REPL detects it by function identity. */
void cmd_exit(const char *arg);

/* ---- Shared CLI internals (defined in main.c) ---- */

extern char cwd[256];                       /* device-side working directory */
void  resolve_path(const char *arg, char *out, size_t len);
/* Shared by cp/mv: when `dst_raw` denotes a directory (trailing '/', or a '.'/'..'
 * final component), append basename(src_resolved) to the resolved `dst_resolved`. */
void  dir_target(const char *src_resolved, const char *dst_raw,
                 char *dst_resolved, size_t cap);

void  load_client_settings(void);            /* (re)read device-backed client settings, e.g. the theme */

/* Build $HOME/.fantasi/<name> into `out` (creating the dir), or "" if $HOME is unset. Used for the
 * persistent readline history files (fantasi.log, fantasi.rfid.log). Defined in main.c. */
void  fantasi_state_path(const char *name, char *out, size_t len);
extern bool g_no_history;                    /* -c one-shot mode: don't read or persist any history */

bool  fat_mount(void);                       /* ensure the Fantasi FAT is mounted */
void  fat_unmount(void);
const char *fat_path(const char *vpath);     /* device path -> host mount path */
bool  fat_sync(void);

extern int  ser_fd;                          /* serial fd, <0 if not open */
extern bool msc_active;                      /* MSC mount currently active */
void  ser_send_cmd(const char *cmd);

#ifdef HAS_PROTO
extern bool     use_ble;                     /* talking over BLE, not USB/MSC */
#endif
extern bool     use_usb;                     /* talking over the USB vendor pipe */
extern bool     g_switch_mode;               /* device is switch-mode (PM3): its SAM7S
                                              * dual-bank OUT can't take pipelined chunks,
                                              * so uploads pace one chunk at a time */
#ifdef HAS_PROTO
extern uint32_t proto_req_id;                  /* monotonic protobuf request id */
extern uint32_t proto_session_id;              /* device-owned logical session; 0 = legacy */
extern size_t   proto_rx_len;                  /* protobuf RX accumulator length */
/* No complete response arrived before the receive deadline. Unlike -1, this
 * does not mean that the transport disconnected or the frame was invalid. */
#define PROTO_RECV_TIMEOUT (-2)
int   proto_send(CliRequest *req);       /* drain stale rx, then send */
int   proto_write_req(CliRequest *req);        /* send without draining (pipelined) */
int   proto_recv(CliResponse *resp);
void  proto_send_cmd(const char *cmd);
void  proto_drain_quiet(void);                 /* discard rx until the link is quiet */
void  proto_cmd_rm(const char *arg);           /* in rm.c; reused as rmdir's BLE handler */
int   proto_download(const char *devpath, FILE *out);            /* windowed download (cat.c) */
int   proto_upload(const char *localpath, const char *devpath); /* pipelined upload (upload.c) */
int   proto_copy_dev(const char *devsrc, const char *devdst);   /* device->device copy (cp.c) */
#define CAT_WINDOW 4096                       /* windowed download chunk size */
#endif

#ifdef HAS_USB_VENDOR
/* Bring a serial session up to the WebUSB vendor pipe (sets use_usb). No-op/true if already there.
 * `required` makes a failure hard. Defined in main.c; used by `edit` when it needs the protobuf file
 * path but the session is still on plain serial. */
bool  try_webusb_upgrade(bool required);
#endif

#endif
