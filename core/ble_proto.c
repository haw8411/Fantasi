#include "ble_proto.h"
#include "ble_pipe.h"
#include "cli.h"
#include "../hal/hal.h"
#include "../hal/storage/hal_storage.h"
#include "../hal/storage/fat_ramdisk.h"   /* fatrd_invalidate() */
#include "ramfs.h"
#include "vfs.h"
#include "app_run.h"
#include "../proto/fantasi.pb.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "lfs.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#define PROTO_TLS_SLOT 0
/* Must exceed the largest framed CliRequest. CliRequest_size is 597
 * (FileWriteChunk with up to 480 B of data), so 512 truncated every
 * upload chunk at the msg_len > ACCUM_SIZE-2 guard and dropped it. */
#define ACCUM_SIZE     1024
#define OUT_BUF_SIZE   499
#define FILE_CACHE_SZ  256

/* ---- Static state (single BLE client) ---- */

static CliRequest  s_req;
static CliResponse s_resp;
static uint8_t     s_encode_buf[2 + CliResponse_size];

static uint8_t     s_out_buf[OUT_BUF_SIZE];
static size_t      s_out_len;
static uint32_t    s_req_id;
static cli_ctx_t  *s_proto_ctx;

static ble_pipe_t  s_pipe;

/* Framed-response sink for the session currently dispatching (BLE pipe, USB
 * vendor, ...) and a lock serialising handler dispatch across transports that
 * share the static encode/dispatch buffers above. */
static size_t (*s_emit)(const uint8_t *buf, size_t len);
static SemaphoreHandle_t s_proto_lock;

/* Serialises the actual transport write only (not the whole dispatch), so the
 * async app pump can stream output on its own task without holding s_proto_lock -
 * which would stall the RX loop out of processing a file_write (a module the app
 * is waiting for). Held only around a send, so the RX loop can write ramfs while
 * the pump is mid-emit; a dispatch's ramfs work completes before its ack blocks. */
static SemaphoreHandle_t s_emit_lock;

static uint8_t         s_file_cache[FILE_CACHE_SZ];
static lfs_file_t      s_file;
static struct lfs_file_config s_fcfg = { .buffer = s_file_cache };

/* ---- Send a framed CliResponse into the pipe ---- */

static void send_response(cli_ctx_t *ctx, CliResponse *resp)
{
    (void)ctx;
    pb_ostream_t stream = pb_ostream_from_buffer(s_encode_buf + 2,
                                                  sizeof(s_encode_buf) - 2);
    if (!pb_encode(&stream, CliResponse_fields, resp))
        return;
    uint16_t len = (uint16_t)stream.bytes_written;
    s_encode_buf[0] = (uint8_t)(len & 0xFF);
    s_encode_buf[1] = (uint8_t)(len >> 8);

    if (s_emit_lock) xSemaphoreTake(s_emit_lock, portMAX_DELAY);
    if (s_emit) s_emit(s_encode_buf, 2 + len);
    if (s_emit_lock) xSemaphoreGive(s_emit_lock);
}

static void send_error(cli_ctx_t *ctx, uint32_t id, const char *msg)
{
    s_resp = (CliResponse){ .id = id, .has_next = false,
        .which_payload = CliResponse_error_tag };
    strncpy(s_resp.payload.error.message, msg,
            sizeof(s_resp.payload.error.message) - 1);
    send_response(ctx, &s_resp);
}

static void send_ok(cli_ctx_t *ctx, uint32_t id, const char *msg)
{
    s_resp = (CliResponse){ .id = id, .has_next = false,
        .which_payload = CliResponse_output_tag };
    strncpy(s_resp.payload.output, msg,
            sizeof(s_resp.payload.output) - 1);
    send_response(ctx, &s_resp);
}

#ifdef FANTASI_ENABLE_APPS
/* ---- Async app session: emit callbacks the pump task (core/app_run.c) drives ----
 * The pump runs on its own task, so these guard the shared encode buffer with the
 * proto lock and emit to the sink captured when the app was launched, not the
 * per-dispatch s_emit (which may belong to another channel by the time the app
 * prints). One session at a time (app_launch_async enforces it). */
static size_t (*s_session_emit)(const uint8_t *, size_t);
static CliResponse s_sresp;
static uint8_t     s_sencode[2 + CliResponse_size];   /* pump's own buffer (not shared) */

static void session_send(void)
{
    if (!s_session_emit) return;
    pb_ostream_t st = pb_ostream_from_buffer(s_sencode + 2, sizeof(s_sencode) - 2);
    if (!pb_encode(&st, CliResponse_fields, &s_sresp)) return;
    uint16_t len = (uint16_t)st.bytes_written;
    s_sencode[0] = (uint8_t)(len & 0xFF);
    s_sencode[1] = (uint8_t)(len >> 8);
    if (s_emit_lock) xSemaphoreTake(s_emit_lock, portMAX_DELAY);
    s_session_emit(s_sencode, 2 + len);
    if (s_emit_lock) xSemaphoreGive(s_emit_lock);
}

static void session_output_cb(uint32_t id, const char *data, size_t len)
{
    for (size_t off = 0; off < len; ) {
        size_t chunk = len - off;
        if (chunk > sizeof(s_sresp.payload.output) - 1) chunk = sizeof(s_sresp.payload.output) - 1;
        s_sresp = (CliResponse){ .id = id, .has_next = true, .which_payload = CliResponse_output_tag };
        memcpy(s_sresp.payload.output, data + off, chunk);
        s_sresp.payload.output[chunk] = '\0';
        session_send();
        off += chunk;
    }
}

static void session_module_request_cb(uint32_t id, const char *name)
{
    s_sresp = (CliResponse){ .id = id, .has_next = true, .which_payload = CliResponse_module_request_tag };
    strncpy(s_sresp.payload.module_request, name, sizeof(s_sresp.payload.module_request) - 1);
    session_send();
}

static void session_done_cb(uint32_t id, int code)
{
    (void)code;
    s_sresp = (CliResponse){ .id = id, .has_next = false, .which_payload = CliResponse_output_tag };
    s_sresp.payload.output[0] = '\0';
    session_send();
    s_session_emit = NULL;
}

static const app_session_cb_t s_session_cb = {
    session_output_cb, session_module_request_cb, session_done_cb,
};

static void handle_app_launch(cli_ctx_t *ctx, CliRequest *req)
{
    s_session_emit = s_emit;                 /* stream this session over this channel */
    int rc = app_launch_async(req->payload.app_launch, req->id, &s_session_cb);
    if (rc < 0) {
        s_session_emit = NULL;
        send_error(ctx, req->id,
            rc == -1 ? "an app is already running" :
            rc == -2 ? "not found" : "load failed");
    }
    /* success: the pump streams the app's output, its module requests, and a final
     * has_next=false when it exits. The RX loop is now free for file_write etc. */
}

static void handle_app_input(cli_ctx_t *ctx, CliRequest *req)
{
    app_session_feed_input(req->payload.app_input.bytes, req->payload.app_input.size);
    send_ok(ctx, req->id, "");
}

static void handle_app_stop(cli_ctx_t *ctx, CliRequest *req)
{
    app_session_stop();
    send_ok(ctx, req->id, "");
}
#endif /* FANTASI_ENABLE_APPS */

/* ---- Output capture for text commands ---- */

static void flush_output(bool has_next)
{
    if (s_out_len == 0 && has_next) return;
    s_resp = (CliResponse){ .id = s_req_id, .has_next = has_next,
        .which_payload = CliResponse_output_tag };
    memcpy(s_resp.payload.output, s_out_buf, s_out_len);
    s_resp.payload.output[s_out_len] = '\0';
    send_response(s_proto_ctx, &s_resp);
    s_out_len = 0;
}

static size_t capture_write(const uint8_t *buf, size_t len, void *c)
{
    (void)c;
    for (size_t i = 0; i < len; i++) {
        if (buf[i] == 0x06) continue;
        s_out_buf[s_out_len++] = buf[i];
        if (s_out_len >= OUT_BUF_SIZE || buf[i] == '\n')
            flush_output(true);
    }
    return len;
}

/* ---- Text command dispatch ---- */

static void handle_command(cli_ctx_t *ctx, CliRequest *req)
{
    char line[128];
    strncpy(line, req->payload.command, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';

    char *argv[16];
    int argc = 0;
    char *p = line;
    while (*p && argc < 16) {
        while (*p == ' ' || *p == '\t') *p++ = '\0';
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
    }
    if (argc == 0) { send_ok(ctx, req->id, ""); return; }

    s_req_id = req->id;
    s_proto_ctx = ctx;
    s_out_len = 0;

    size_t (*saved_write)(const uint8_t *, size_t, void *) = ctx->transport.write;
    ctx->transport.write = capture_write;
    vTaskSetThreadLocalStoragePointer(NULL, PROTO_TLS_SLOT, ctx);

    const cli_command_t *cmd = cli_lookup(argv[0]);
    if (cmd)
        cmd->fn(argc, argv);

    ctx->transport.write = saved_write;

    if (!cmd) {
        s_out_len = 0;
        send_error(ctx, req->id, "unknown command");
    } else {
        flush_output(false);
    }
}

/* ---- File read ---- */

static void handle_file_read(cli_ctx_t *ctx, CliRequest *req)
{
    if (vfs_is_ramfs(req->payload.file_read.path)) {
        const char *leaf = vfs_ramfs_leaf(req->payload.file_read.path);
        int32_t fsize = ramfs_size(leaf);
        if (fsize < 0) { send_error(ctx, req->id, "open failed"); return; }
        uint32_t offset = req->payload.file_read.offset;
        if (offset > (uint32_t)fsize) offset = (uint32_t)fsize;
        uint32_t avail = (uint32_t)fsize - offset;
        uint32_t remain = req->payload.file_read.size;
        if (remain == 0 || remain > avail) remain = avail;
        uint32_t sent = 0;
        while (sent < remain) {
            uint32_t chunk = remain - sent;
            if (chunk > 480) chunk = 480;
            s_resp = (CliResponse){ .id = req->id,
                .which_payload = CliResponse_file_data_tag };
            s_resp.payload.file_data.offset = offset + sent;
            int32_t n = ramfs_read(leaf, offset + sent,
                s_resp.payload.file_data.data.bytes, chunk);
            if (n <= 0) break;
            s_resp.payload.file_data.data.size = (pb_size_t)n;
            sent += (uint32_t)n;
            s_resp.payload.file_data.last = (sent >= remain || (uint32_t)n < chunk);
            s_resp.has_next = !s_resp.payload.file_data.last;
            send_response(ctx, &s_resp);
        }
        if (sent == 0) send_error(ctx, req->id, "read failed");
        return;
    }

    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) { send_error(ctx, req->id, "storage unavailable"); return; }

    lfs_file_t f;
    if (lfs_file_opencfg(lfs, &f, req->payload.file_read.path,
                          LFS_O_RDONLY, &s_fcfg) < 0) {
        send_error(ctx, req->id, "open failed");
        return;
    }

    lfs_ssize_t fsize = lfs_file_size(lfs, &f);
    uint32_t offset = req->payload.file_read.offset;
    if (offset > (uint32_t)fsize) offset = (uint32_t)fsize;
    /* Bytes available from `offset` onward - `remain` must be relative to the
     * read start, not the whole file, or a resumed read (offset>0) whose tail
     * is an exact multiple of the chunk size never sets `last` (the EOF read
     * returns 0 and breaks before the final flagged chunk). */
    uint32_t avail = (uint32_t)fsize - offset;
    uint32_t remain = req->payload.file_read.size;
    if (remain == 0 || remain > avail) remain = avail;

    if (offset > 0) lfs_file_seek(lfs, &f, offset, LFS_SEEK_SET);

    uint32_t sent = 0;
    while (sent < remain) {
        uint32_t chunk = remain - sent;
        if (chunk > 480) chunk = 480;

        s_resp = (CliResponse){ .id = req->id,
            .which_payload = CliResponse_file_data_tag };
        s_resp.payload.file_data.offset = offset + sent;

        lfs_ssize_t n = lfs_file_read(lfs, &f,
            s_resp.payload.file_data.data.bytes, chunk);
        if (n <= 0) break;

        s_resp.payload.file_data.data.size = (pb_size_t)n;
        sent += (uint32_t)n;
        s_resp.payload.file_data.last = (sent >= remain || (uint32_t)n < chunk);
        s_resp.has_next = !s_resp.payload.file_data.last;
        send_response(ctx, &s_resp);
    }

    lfs_file_close(lfs, &f);

    if (sent == 0)
        send_error(ctx, req->id, "read failed");
}

/* ---- File write ---- */

static bool s_file_open;

static void handle_file_write(cli_ctx_t *ctx, CliRequest *req)
{
    FileWriteChunk *fw = &req->payload.file_write;
    fatrd_invalidate();          /* keep the synthetic-FAT model in sync */

    if (vfs_is_ramfs(fw->path)) {
        const char *leaf = vfs_ramfs_leaf(fw->path);
        if (fw->offset == 0 && ramfs_truncate(leaf) != 0) {
            send_error(ctx, req->id, "create failed"); return;
        }
        /* Reserve the whole file up front from the host's size hint, so the
         * write sequence is one allocation rather than a realloc-per-chunk grow
         * that needs ~2x the file live at once - which overflows a tight heap. */
        if (fw->offset == 0 && fw->has_total && ramfs_reserve(leaf, fw->total) != 0) {
            send_error(ctx, req->id, "no space"); return;
        }
        if (ramfs_write_at(leaf, fw->offset, fw->data.bytes, fw->data.size) != 0) {
            send_error(ctx, req->id, "write failed"); return;
        }
        send_ok(ctx, req->id, "ok");
        return;
    }

    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) { send_error(ctx, req->id, "storage unavailable"); return; }

    if (fw->offset == 0) {
        if (s_file_open) { lfs_file_close(lfs, &s_file); s_file_open = false; }
        int rc = lfs_file_opencfg(lfs, &s_file, fw->path,
                    LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &s_fcfg);
        if (rc < 0) { send_error(ctx, req->id, "create failed"); return; }
        s_file_open = true;
    } else if (!s_file_open) {
        /* A retransmitted chunk (host detected a lost write and rewound) can
         * arrive after a premature `last` already closed the file. Reopen it
         * (no truncate) so the hole can be filled - writes are idempotent and
         * addressed by absolute offset. */
        int rc = lfs_file_opencfg(lfs, &s_file, fw->path,
                    LFS_O_WRONLY, &s_fcfg);
        if (rc < 0) { send_error(ctx, req->id, "reopen failed"); return; }
        s_file_open = true;
    }

    if (fw->offset > 0)
        lfs_file_seek(lfs, &s_file, fw->offset, LFS_SEEK_SET);

    lfs_ssize_t n = lfs_file_write(lfs, &s_file, fw->data.bytes, fw->data.size);
    if (n < 0) {
        lfs_file_close(lfs, &s_file);
        s_file_open = false;
        send_error(ctx, req->id, "write failed");
        return;
    }

    if (fw->last) {
        int crc = lfs_file_close(lfs, &s_file);
        s_file_open = false;
        if (crc < 0) { send_error(ctx, req->id, "flush failed"); return; }
    }

    send_ok(ctx, req->id, "ok");
}

/* ---- Directory listing ---- */

/* Stream one dir_entry per VFS listing callback (has_next=true); handle_dir_list
 * sends a terminating ok afterwards, which the host treats as end-of-list. */
typedef struct { cli_ctx_t *ctx; uint32_t id; } dirlist_ctx_t;
static void dirlist_cb(const char *name, uint32_t size, bool is_dir, void *vctx)
{
    dirlist_ctx_t *d = vctx;
    s_resp = (CliResponse){ .id = d->id, .has_next = true,
        .which_payload = CliResponse_dir_entry_tag };
    DirEntry *e = &s_resp.payload.dir_entry;
    strncpy(e->name, name, sizeof(e->name) - 1);
    e->name[sizeof(e->name) - 1] = '\0';
    e->is_dir = is_dir;
    e->size = size;
    send_response(d->ctx, &s_resp);
}

static void handle_dir_list(cli_ctx_t *ctx, CliRequest *req)
{
    /* Go through the VFS so the listing matches the MSC view exactly - including
     * the synthetic /ramfs mount that VFS injects at the root (it is not a real
     * LittleFS entry). The terminating ok ends the stream (has_next=false). */
    dirlist_ctx_t d = { ctx, req->id };
    vfs_list(req->payload.dir_list.path, dirlist_cb, &d);
    send_ok(ctx, req->id, "");
}

/* ---- File delete ---- */

static void handle_file_delete(cli_ctx_t *ctx, CliRequest *req)
{
    /* vfs_remove routes ramfs vs LittleFS, drops empty dirs (so this also backs
     * rmdir), and invalidates the synthetic-FAT model. */
    if (vfs_remove(req->payload.file_delete.path) == 0)
        send_ok(ctx, req->id, "ok");
    else
        send_error(ctx, req->id, "delete failed");
}

/* ---- Mkdir ---- */

static void handle_mkdir(cli_ctx_t *ctx, CliRequest *req)
{
    fatrd_invalidate();
    if (vfs_is_ramfs(req->payload.mkdir.path)) {
        /* ramfs is flat: the mount itself always exists, subdirs are unsupported. */
        const char *leaf = vfs_ramfs_leaf(req->payload.mkdir.path);
        if (leaf[0] == '\0') send_ok(ctx, req->id, "ok");
        else send_error(ctx, req->id, "ramfs has no subdirs");
        return;
    }

    lfs_t *lfs = hal_storage_lfs();
    if (!lfs) { send_error(ctx, req->id, "storage unavailable"); return; }

    int rc = lfs_mkdir(lfs, req->payload.mkdir.path);
    if (rc < 0 && rc != LFS_ERR_EXIST)
        send_error(ctx, req->id, "mkdir failed");
    else
        send_ok(ctx, req->id, "ok");
}

/* ---- Main task ---- */

static void pipe_flush(void) { ble_pipe_flush(&s_pipe); }

/* ---- Shared protobuf engine (used by BLE + USB vendor transports) ---- */

void fantasi_proto_init(void)
{
    if (!s_proto_lock) s_proto_lock = xSemaphoreCreateMutex();
    if (!s_emit_lock)  s_emit_lock  = xSemaphoreCreateMutex();
}

void fantasi_proto_rx(cli_ctx_t *ctx, uint8_t *accum, size_t cap, size_t *accum_len,
                      size_t (*emit)(const uint8_t *, size_t),
                      const uint8_t *in, size_t n)
{
    size_t copy = n;
    if (*accum_len + copy > cap) copy = cap - *accum_len;
    memcpy(accum + *accum_len, in, copy);
    *accum_len += copy;

    while (*accum_len >= 2) {
        uint16_t msg_len = (uint16_t)accum[0] | ((uint16_t)accum[1] << 8);
        if (msg_len > cap - 2) { *accum_len = 0; break; }
        if (*accum_len < 2u + msg_len) break;

        /* Serialise dispatch: the encode/response statics are shared, and BLE +
         * USB proto tasks can run concurrently on the composite targets. */
        if (s_proto_lock) xSemaphoreTake(s_proto_lock, portMAX_DELAY);
        s_emit = emit;

        pb_istream_t stream = pb_istream_from_buffer(accum + 2, msg_len);
        s_req = (CliRequest){0};
        if (!pb_decode(&stream, CliRequest_fields, &s_req)) {
            send_error(ctx, 0, "decode failed");
        } else {
            switch (s_req.which_payload) {
            case CliRequest_command_tag:     handle_command(ctx, &s_req); break;
            case CliRequest_file_read_tag:   handle_file_read(ctx, &s_req); break;
            case CliRequest_file_write_tag:  handle_file_write(ctx, &s_req); break;
            case CliRequest_dir_list_tag:    handle_dir_list(ctx, &s_req); break;
            case CliRequest_file_delete_tag: handle_file_delete(ctx, &s_req); break;
            case CliRequest_mkdir_tag:       handle_mkdir(ctx, &s_req); break;
#ifdef FANTASI_ENABLE_APPS
            case CliRequest_app_launch_tag:  handle_app_launch(ctx, &s_req); break;
            case CliRequest_app_input_tag:   handle_app_input(ctx, &s_req); break;
            case CliRequest_app_stop_tag:    handle_app_stop(ctx, &s_req); break;
#endif
            default: send_error(ctx, s_req.id, "unknown request"); break;
            }
        }
        if (ctx->transport.flush) ctx->transport.flush();
        if (s_proto_lock) xSemaphoreGive(s_proto_lock);

        size_t consumed = 2 + msg_len;
        *accum_len -= consumed;
        if (*accum_len > 0)
            memmove(accum, accum + consumed, *accum_len);
    }
}

/* BLE framed-response sink: through the MTU-fragmenting pipe. */
static size_t ble_emit(const uint8_t *buf, size_t len)
{
    return ble_pipe_write(buf, len, &s_pipe);
}

void ble_proto_task(void *arg)
{
    cli_ctx_t *ctx = (cli_ctx_t *)arg;
    vTaskSetThreadLocalStoragePointer(NULL, PROTO_TLS_SLOT, ctx);

    ble_pipe_init(&s_pipe, ctx->transport.write, ctx->transport.poll,
                  ctx->transport.ctx, 20);
    ctx->transport.flush = pipe_flush;

    while (!ctx->transport.connected(ctx->transport.ctx)) {
        if (ctx->transport.poll) ctx->transport.poll();
        /* Block briefly instead of taskYIELD(). taskYIELD() only re-runs the
         * scheduler; this task stays Ready, so with nothing higher-priority
         * waiting the loop just spins at 100% CPU and the core never idles.
         * Waiting for a connection isn't time-critical (setup takes hundreds of
         * ms), so a 5 ms sleep polls plenty often while letting the idle task /
         * low-power sleep run. */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    /* Wait for GATT service discovery + MTU exchange (~1.5s on BlueZ) */
    for (int i = 0; i < 40; i++) {
        if (ctx->transport.poll) ctx->transport.poll();
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_pipe.mtu_payload > 20) break;
    }

    static uint8_t accum[ACCUM_SIZE];
    static size_t accum_len;
    accum_len = 0;

    for (;;) {
        if (ctx->transport.poll) ctx->transport.poll();

        uint8_t tmp[256];
        size_t n = ctx->transport.read(tmp, sizeof(tmp), ctx->transport.ctx);
        if (n == 0) {
            /* No data: block briefly instead of taskYIELD(). taskYIELD() keeps
             * this task Ready and just busy-spins the loop at 100% CPU; data
             * only arrives about once per connection interval (~45 ms), so a
             * 2 ms sleep adds negligible latency while letting the core idle. */
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        fantasi_proto_rx(ctx, accum, ACCUM_SIZE, &accum_len, ble_emit, tmp, n);
    }
}

void ble_proto_set_mtu(uint16_t att_mtu)
{
    ble_pipe_set_mtu(&s_pipe, att_mtu);
}
