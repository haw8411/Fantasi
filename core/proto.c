#include "proto.h"

#include "app_run.h"
#include "cli.h"
#include "ramfs.h"
#include "vfs.h"
#include "../hal/hal_power.h"
#include "../hal/storage/fat_ramdisk.h"

#include <pb_decode.h>
#include <pb_encode.h>
#include "lfs.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

#define FILE_CACHE_SIZE      256
#define RESPONSE_TEXT_MAX    (sizeof(((CliResponse *)0)->payload.output) - 1)
#define COMMAND_OUTPUT_SPARE (sizeof(((CliRequest *)0)->payload) - \
                              sizeof(((CliRequest *)0)->payload.command))
#define COMMAND_OUTPUT_CAP   ((COMMAND_OUTPUT_SPARE < RESPONSE_TEXT_MAX) ? \
                              COMMAND_OUTPUT_SPARE : RESPONSE_TEXT_MAX)
#define PROTO_SESSION_QUEUE_MAX 8
#define PROTO_HANDLER __attribute__((noinline))

_Static_assert(COMMAND_OUTPUT_SPARE > 0,
               "command request union has no reusable output storage");

/* A worker exists only while its session has queued work. Command execution
 * and storage run here rather than on transport receive tasks. Platforms may
 * override the default stack size. */
#ifndef PROTO_SESSION_STACK
#define PROTO_SESSION_STACK 1024
#endif

typedef struct proto_job {
    struct proto_job *next;
    uint16_t len;
    uint8_t message[];
} proto_job_t;

struct fantasi_proto_session {
    struct fantasi_proto_session *next;
    const fantasi_proto_transport_t *transport;
    proto_job_t *head;
    TaskHandle_t worker;
    uint32_t id;
    uint32_t link;
    volatile uint32_t active_id;
    volatile uint32_t cancel_id;
    TickType_t touched;
    uint8_t queued;
    uint8_t refs;                  /* registry + worker + async app + snapshots */
    bool explicit_session;
    volatile bool closing;
};

/* Idle-session RAM scales with the number of connected hosts. Allocator
 * metadata is platform-specific, but the payload must remain within 40 bytes
 * on supported 32-bit targets. */
_Static_assert(sizeof(struct fantasi_proto_session) <= 40,
               "idle protobuf session grew beyond its RAM budget");

typedef struct {
    fantasi_proto_session_t *session;
    uint32_t request_id;
    bool cancel_delivered;
    /* Coalesce the many small cli_write() calls made by commands such as ps.
     * A command request uses only the first 128 bytes of CliRequest's large
     * payload union; output points at the otherwise-unused tail of that same
     * stack object. This adds neither a second large stack member nor a heap
     * allocation, and never grows the idle session object. */
    uint16_t output_len;
    uint8_t *output;
    uint16_t output_cap;
    cli_ctx_t cli;
} proto_exec_t;

typedef struct {
    uint32_t id;
    uint32_t session;
    uint32_t cancel_id;
    pb_size_t payload;
    bool has_session;
    bool valid;
} request_meta_t;

/* A streamed LittleFS upload needs one open handle and one cache until its
 * final chunk commits.  Keep that state on the active worker rather than in
 * fantasi_proto_session: idle sessions remain 40 bytes, while simultaneous
 * uploads still get independent handles. */
typedef struct {
    lfs_t *lfs;
    lfs_file_t file;
    struct lfs_file_config cfg;
    char path[sizeof(((FileWriteChunk *)0)->path)];
    uint8_t cache[FILE_CACHE_SIZE] __attribute__((aligned(4)));
} proto_file_write_t;

static fantasi_proto_session_t *s_sessions;
static SemaphoreHandle_t s_sessions_lock;
static SemaphoreHandle_t s_response_lock;
static union {
    CliRequest request;
    CliResponse response;
} s_codec;
#define s_probe_request s_codec.request
#define s_response      s_codec.response
static uint32_t s_next_session;

/* One async application is supported by app_run itself. Retaining the owning
 * proto session here keeps its route alive until the pump emits the final
 * response, even if the host closes the session in the meantime. */
#ifdef FANTASI_ENABLE_APPS
static fantasi_proto_session_t *s_app_session;
static uint32_t s_app_request;
#ifdef APP_KEEP_SESSION_WORKER
/* PM3 keeps the launch/upload worker only until a requested RAMFS module is
 * atomically published. It then releases that stack before module execution. */
static bool s_app_worker_hold;
#endif
#endif

static void sessions_take(void)
{
    if (s_sessions_lock) xSemaphoreTake(s_sessions_lock, portMAX_DELAY);
}

static void sessions_give(void)
{
    if (s_sessions_lock) xSemaphoreGive(s_sessions_lock);
}

static bool route_matches(const fantasi_proto_session_t *s,
                          const fantasi_proto_transport_t *transport,
                          uint32_t link)
{
    return s->transport == transport && s->link == link;
}

static fantasi_proto_session_t *find_session_locked(
    uint32_t id, const fantasi_proto_transport_t *transport, uint32_t link)
{
    for (fantasi_proto_session_t *s = s_sessions; s; s = s->next)
        if (s->id == id && route_matches(s, transport, link) && !s->closing)
            return s;
    return NULL;
}

static void destroy_session(fantasi_proto_session_t *s)
{
    if (s->transport && s->transport->release)
        s->transport->release(s->id, s->link);
    vPortFree(s);
}

static void session_put(fantasi_proto_session_t *s)
{
    bool destroy = false;
    sessions_take();
    if (s->refs > 0 && --s->refs == 0) destroy = true;
    sessions_give();
    if (destroy) destroy_session(s);
}

static fantasi_proto_session_t *session_get(
    uint32_t id, const fantasi_proto_transport_t *transport, uint32_t link)
{
    sessions_take();
    fantasi_proto_session_t *s = find_session_locked(id, transport, link);
    if (s) s->refs++;
    sessions_give();
    return s;
}

static fantasi_proto_session_t *session_create(
    const fantasi_proto_transport_t *transport, uint32_t link, bool explicit_session)
{
    fantasi_proto_session_t *s = pvPortMalloc(sizeof(*s));
    if (!s) return NULL;
    memset(s, 0, sizeof(*s));
    s->transport = transport;
    s->link = link;
    s->explicit_session = explicit_session;
    s->touched = xTaskGetTickCount();
    s->refs = 1;                         /* registry reference */

    sessions_take();
    /* Session ids are 16-bit: the WebUSB mux carries the id in the control
     * request's wValue and reserves wIndex for the READ response offset (see
     * core/usb_proto.c). 65535 concurrent sessions is far beyond what the
     * device's RAM can host; the collision check below skips any live id after
     * wrap. BLE/serial ids are the same values and fit their fields unchanged. */
    do {
        s_next_session++;
        s->id = s_next_session & 0xffffu;
        if (s->id == 0) { s_next_session++; s->id = s_next_session & 0xffffu; }
    } while (find_session_locked(s->id, transport, link));
    s->next = s_sessions;
    s_sessions = s;
    sessions_give();
    return s;
}

static bool session_is_connected(fantasi_proto_session_t *s)
{
    if (s->closing) return false;
    return !s->transport->connected ||
           s->transport->connected(s->id, s->link);
}

/* ---- Response encoding -------------------------------------------------- */

/* Called with s_response_lock held. The returned allocation contains the
 * two-byte little-endian framing prefix and remains owned by the caller. */
static uint8_t *encode_response_locked(fantasi_proto_session_t *s, size_t *frame_len)
{
    s_response.has_session = s->explicit_session;
    s_response.session = s->id;

    size_t encoded = 0;
    if (!pb_get_encoded_size(&encoded, CliResponse_fields, &s_response) ||
        encoded > UINT16_MAX)
        return NULL;

    uint8_t *frame = pvPortMalloc(encoded + 2);
    if (!frame) return NULL;
    frame[0] = (uint8_t)encoded;
    frame[1] = (uint8_t)(encoded >> 8);
    pb_ostream_t out = pb_ostream_from_buffer(frame + 2, encoded);
    if (!pb_encode(&out, CliResponse_fields, &s_response)) {
        vPortFree(frame);
        return NULL;
    }
    *frame_len = encoded + 2;
    return frame;
}

static bool emit_allocated(fantasi_proto_session_t *s, uint8_t *frame, size_t len)
{
    if (!frame) return false;
    size_t sent = 0;
    if (s->transport->emit && session_is_connected(s))
        sent = s->transport->emit(s->explicit_session ? s->id : 0,
                                  s->link, frame, len);
    vPortFree(frame);
    /* A lease measures host liveness, not device output. In particular, a dead
     * BLE process may leave the physical link up through another process; log
     * output must not renew that abandoned logical session forever. Live hosts
     * send an out-of-band heartbeat while waiting for streaming responses. */
    return sent == len;
}

static bool send_output(fantasi_proto_session_t *s, uint32_t id,
                        const uint8_t *data, size_t len, bool has_next)
{
    bool ok = true;
    if (len == 0) {
        xSemaphoreTake(s_response_lock, portMAX_DELAY);
        s_response = (CliResponse){ .id = id, .has_next = has_next,
            .which_payload = CliResponse_output_tag };
        size_t frame_len = 0;
        uint8_t *frame = encode_response_locked(s, &frame_len);
        xSemaphoreGive(s_response_lock);
        return emit_allocated(s, frame, frame_len);
    }

    for (size_t off = 0; off < len; ) {
        size_t chunk = len - off;
        if (chunk > RESPONSE_TEXT_MAX) chunk = RESPONSE_TEXT_MAX;
        xSemaphoreTake(s_response_lock, portMAX_DELAY);
        s_response = (CliResponse){ .id = id,
            .has_next = (off + chunk < len) ? true : has_next,
            .which_payload = CliResponse_output_tag };
        memcpy(s_response.payload.output, data + off, chunk);
        s_response.payload.output[chunk] = '\0';
        size_t frame_len = 0;
        uint8_t *frame = encode_response_locked(s, &frame_len);
        xSemaphoreGive(s_response_lock);
        if (!emit_allocated(s, frame, frame_len)) ok = false;
        off += chunk;
    }
    return ok;
}

static bool send_error(fantasi_proto_session_t *s, uint32_t id, const char *message)
{
    xSemaphoreTake(s_response_lock, portMAX_DELAY);
    s_response = (CliResponse){ .id = id, .has_next = false,
        .which_payload = CliResponse_error_tag };
    strncpy(s_response.payload.error.message, message,
            sizeof(s_response.payload.error.message) - 1);
    size_t frame_len = 0;
    uint8_t *frame = encode_response_locked(s, &frame_len);
    xSemaphoreGive(s_response_lock);
    return emit_allocated(s, frame, frame_len);
}

static bool send_ok(fantasi_proto_session_t *s, uint32_t id, const char *message)
{
    return send_output(s, id, (const uint8_t *)message, strlen(message), false);
}

static bool send_file_data(fantasi_proto_session_t *s, uint32_t id,
                           uint32_t offset, const uint8_t *data, uint16_t len,
                           bool last)
{
    xSemaphoreTake(s_response_lock, portMAX_DELAY);
    s_response = (CliResponse){ .id = id, .has_next = !last,
        .which_payload = CliResponse_file_data_tag };
    s_response.payload.file_data.offset = offset;
    s_response.payload.file_data.data.size = len;
    memcpy(s_response.payload.file_data.data.bytes, data, len);
    s_response.payload.file_data.last = last;
    size_t frame_len = 0;
    uint8_t *frame = encode_response_locked(s, &frame_len);
    xSemaphoreGive(s_response_lock);
    return emit_allocated(s, frame, frame_len);
}

static bool send_dir_entry(fantasi_proto_session_t *s, uint32_t id,
                           const char *name, uint32_t size, bool is_dir)
{
    xSemaphoreTake(s_response_lock, portMAX_DELAY);
    s_response = (CliResponse){ .id = id, .has_next = true,
        .which_payload = CliResponse_dir_entry_tag };
    strncpy(s_response.payload.dir_entry.name, name,
            sizeof(s_response.payload.dir_entry.name) - 1);
    s_response.payload.dir_entry.is_dir = is_dir;
    s_response.payload.dir_entry.size = size;
    size_t frame_len = 0;
    uint8_t *frame = encode_response_locked(s, &frame_len);
    xSemaphoreGive(s_response_lock);
    return emit_allocated(s, frame, frame_len);
}

#ifdef FANTASI_ENABLE_APPS
static bool send_module_request(fantasi_proto_session_t *s, uint32_t id,
                                const char *name)
{
    xSemaphoreTake(s_response_lock, portMAX_DELAY);
    s_response = (CliResponse){ .id = id, .has_next = true,
        .which_payload = CliResponse_module_request_tag };
    strncpy(s_response.payload.module_request, name,
            sizeof(s_response.payload.module_request) - 1);
    size_t frame_len = 0;
    uint8_t *frame = encode_response_locked(s, &frame_len);
    xSemaphoreGive(s_response_lock);
    return emit_allocated(s, frame, frame_len);
}
#endif

/* ---- Virtual CLI transport --------------------------------------------- */

static bool exec_cancelled(proto_exec_t *e)
{
    return e->session->closing || e->session->cancel_id == e->request_id;
}

static bool exec_emit(proto_exec_t *e, bool has_next)
{
    bool ok = send_output(e->session, e->request_id,
                          e->output, e->output_len, has_next);
    e->output_len = 0;
    return ok;
}

/* cli_transport.flush has no context argument, but command execution has bound
 * this worker's cli_ctx in FreeRTOS TLS. Streaming commands explicitly flush
 * at their natural latency boundary (log: 100 ms, apps: every pump pass). */
static void exec_flush(void)
{
    cli_ctx_t *ctx = cli_current_ctx();
    if (!ctx || ctx->transport.ctx == NULL) return;
    proto_exec_t *e = ctx->transport.ctx;
    if (e->output_len) (void)exec_emit(e, true);
}

static size_t exec_read(uint8_t *buf, size_t len, void *ctx)
{
    proto_exec_t *e = ctx;
    /* A command asking for input has crossed an interactive boundary. Make a
     * pending prompt/status visible before it waits for that input. */
    if (e->output_len) (void)exec_emit(e, true);
    if (!len || e->cancel_delivered || !exec_cancelled(e)) return 0;
    buf[0] = 0x03;
    e->cancel_delivered = true;
    return 1;
}

static bool exec_connected(void *ctx)
{
    proto_exec_t *e = ctx;
    return session_is_connected(e->session);
}

static size_t exec_write(const uint8_t *buf, size_t len, void *ctx)
{
    proto_exec_t *e = ctx;
    size_t off = 0;
    while (off < len) {
        /* 0x06 is the raw serial framing sentinel. cli_write() normally strips
         * it before this layer, but preserve the boundary if a command writes
         * directly through its transport. */
        if (buf[off] == 0x06) {
            if (e->output_len) (void)exec_emit(e, true);
            off++;
            continue;
        }

        if (e->output_len == e->output_cap)
            (void)exec_emit(e, true);

        size_t take = len - off;
        const uint8_t *sentinel = memchr(buf + off, 0x06, take);
        if (sentinel) take = (size_t)(sentinel - (buf + off));
        size_t room = e->output_cap - e->output_len;
        if (take > room) take = room;
        if (take) {
            memcpy(e->output + e->output_len, buf + off, take);
            e->output_len += (uint16_t)take;
            off += take;
        }
    }
    return len;
}

static void exec_wait(uint32_t timeout_ms)
{
    /* cancel_session() notifies this worker, so a blocked command need not wait
     * for its polling timeout before exec_read() can deliver Ctrl-C. */
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
}

/* ---- Request handlers --------------------------------------------------- */

static PROTO_HANDLER void handle_command(fantasi_proto_session_t *s,
                                         CliRequest *req)
{
    /* Decode already guaranteed the bounded protobuf string is terminated.
     * Tokenise it in place; copying it into another 128-byte local needlessly
     * raises every command worker's contiguous stack requirement. */
    char *line = req->payload.command;

    char *argv[16];
    int argc = 0;
    char *p = line;
    while (*p && argc < (int)(sizeof(argv) / sizeof(argv[0]))) {
        while (*p == ' ' || *p == '\t') *p++ = '\0';
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
    }
    if (argc == 0) { send_ok(s, req->id, ""); return; }

    proto_exec_t exec;
    memset(&exec, 0, sizeof(exec));
    exec.output = (uint8_t *)&req->payload + sizeof(req->payload.command);
    exec.output_cap = (uint16_t)COMMAND_OUTPUT_CAP;
    exec.session = s;
    exec.request_id = req->id;
    exec.cli.transport.write = exec_write;
    exec.cli.transport.read = exec_read;
    exec.cli.transport.connected = exec_connected;
    exec.cli.transport.wait = exec_wait;
    exec.cli.transport.flush = exec_flush;
    exec.cli.transport.ctx = &exec;
    cli_bind_ctx(&exec.cli);

    const cli_command_t *command = cli_lookup(argv[0]);
    if (!command) send_error(s, req->id, "unknown command");
    else {
        command->fn(argc, argv);
        /* Mark the buffered tail itself terminal. Besides saving a protobuf
         * frame, this avoids another WebUSB mailbox rendezvous and another BLE
         * notification for the common one-response command. If an explicit
         * flush already emptied it, retain the empty terminal marker. */
        if (exec.output_len) (void)exec_emit(&exec, false);
        else send_output(s, req->id, NULL, 0, false);
    }
}

static PROTO_HANDLER void handle_file_read(fantasi_proto_session_t *s,
                                           CliRequest *req)
{
    FileReadRequest *fr = &req->payload.file_read;
    const char *leaf;
    const vfs_mount_t *mount = vfs_resolve(fr->path, &leaf);
    if (!mount) { send_error(s, req->id, "open failed"); return; }

    int32_t size;
    if (vfs_mount_is_ramfs(mount)) size = ramfs_size(leaf);
    else if (vfs_mount_is_fat(mount)) size = vfs_fat_size(mount->fatdrv, leaf);
    else size = -1;

    lfs_t *lfs = NULL;
    lfs_file_t file;
    uint8_t cache[FILE_CACHE_SIZE] __attribute__((aligned(4)));
    struct lfs_file_config cfg = { .buffer = cache };
    bool lfs_open = false;

    if (!vfs_mount_is_ramfs(mount) && !vfs_mount_is_fat(mount)) {
        lfs = vfs_mount_lfs(mount);
        if (!lfs) { send_error(s, req->id, "storage unavailable"); return; }
        fatrd_store_lock();
        if (lfs_file_opencfg(lfs, &file, leaf, LFS_O_RDONLY, &cfg) >= 0) {
            size = (int32_t)lfs_file_size(lfs, &file);
            if (size >= 0) lfs_open = true;
            else lfs_file_close(lfs, &file);
        }
        fatrd_store_unlock();
    }
    if (size < 0 || (!vfs_mount_is_ramfs(mount) &&
                     !vfs_mount_is_fat(mount) && !lfs_open)) {
        send_error(s, req->id, "open failed");
        return;
    }

    uint32_t offset = fr->offset;
    if (offset > (uint32_t)size) offset = (uint32_t)size;
    uint32_t available = (uint32_t)size - offset;
    uint32_t remaining = fr->size;
    if (remaining == 0 || remaining > available) remaining = available;

    if (lfs_open && offset) {
        fatrd_store_lock();
        lfs_file_seek(lfs, &file, offset, LFS_SEEK_SET);
        fatrd_store_unlock();
    }

    uint8_t data[480];
    uint32_t sent = 0;
    while (sent < remaining && session_is_connected(s)) {
        uint16_t want = (uint16_t)(remaining - sent);
        if (want > sizeof(data)) want = sizeof(data);
        int32_t got;
        if (vfs_mount_is_ramfs(mount))
            got = ramfs_read(leaf, offset + sent, data, want);
        else if (vfs_mount_is_fat(mount))
            got = vfs_fat_pread(mount->fatdrv, leaf, offset + sent, data, want);
        else {
            fatrd_store_lock();
            got = (int32_t)lfs_file_read(lfs, &file, data, want);
            fatrd_store_unlock();
        }
        if (got <= 0) break;
        sent += (uint32_t)got;
        bool last = sent >= remaining || got < want;
        if (!send_file_data(s, req->id, offset + sent - (uint32_t)got,
                            data, (uint16_t)got, last))
            break;
    }

    if (lfs_open) {
        fatrd_store_lock();
        lfs_file_close(lfs, &file);
        fatrd_store_unlock();
    }
    if (sent == 0) send_error(s, req->id, "read failed");
}

static int file_write_close(proto_file_write_t **slot)
{
    proto_file_write_t *state = *slot;
    if (!state) return 0;
    *slot = NULL;
    fatrd_store_lock();
    int rc = lfs_file_close(state->lfs, &state->file);
    fatrd_store_unlock();
    if (rc >= 0) fatrd_invalidate();
    vPortFree(state);
    return rc < 0 ? -1 : 0;
}

static PROTO_HANDLER void handle_file_write(
    fantasi_proto_session_t *s, CliRequest *req,
    proto_file_write_t **write_state)
{
    FileWriteChunk *fw = &req->payload.file_write;
    const char *leaf;
    const vfs_mount_t *mount = vfs_resolve(fw->path, &leaf);
    if (!mount) { send_error(s, req->id, "create failed"); return; }

    /* A completed MSC write may retain its partial final sector for a later
     * append. An external replacement owns this path now; retire only that
     * cache before its first chunk so a later MSC sync cannot replay old data. */
    if (fw->offset == 0) fatrd_external_forget(fw->path);

    if (vfs_mount_is_ramfs(mount)) {
        if (file_write_close(write_state) < 0) {
            send_error(s, req->id, "flush failed"); return;
        }
        if (fw->offset == 0 && ramfs_truncate(leaf) != 0) {
            send_error(s, req->id, "create failed"); return;
        }
        if (fw->offset == 0 && fw->has_total &&
            ramfs_reserve(leaf, fw->total) != 0) {
            send_error(s, req->id, "no space"); return;
        }
        if (ramfs_write_at(leaf, fw->offset, fw->data.bytes,
                           fw->data.size) != 0) {
            send_error(s, req->id, "write failed"); return;
        }
        /* FileWriteChunk.last is the publication boundary. Invalidating the
         * synthetic MSC model for every 480-byte chunk makes a mounted host
         * rebuild the whole FAT view in TinyUSB's higher-priority task between
         * WebUSB controls, turning an app upload into dozens of ~64 ms stalls.
         * An interrupted partial upload is deliberately not advertised. */
        if (fw->last) fatrd_invalidate();
        send_ok(s, req->id, "ok");
        return;
    }

    if (vfs_mount_is_fat(mount)) {
        if (file_write_close(write_state) < 0) {
            send_error(s, req->id, "flush failed"); return;
        }
        if (vfs_fat_wchunk(mount->fatdrv, leaf, fw->offset,
                           fw->data.bytes, fw->data.size, fw->last) != 0) {
            send_error(s, req->id, "write failed"); return;
        }
        if (fw->last) fatrd_invalidate();
        send_ok(s, req->id, "ok");
        return;
    }

    lfs_t *lfs = vfs_mount_lfs(mount);
    if (!lfs) { send_error(s, req->id, "storage unavailable"); return; }

    proto_file_write_t *state = *write_state;
    bool same_file = state && state->lfs == lfs &&
                     strcmp(state->path, leaf) == 0;
    if (state && (fw->offset == 0 || !same_file)) {
        if (file_write_close(write_state) < 0) {
            send_error(s, req->id, "flush failed"); return;
        }
        state = NULL;
    }

    if (!state) {
        state = pvPortMalloc(sizeof(*state));
        if (!state) { send_error(s, req->id, "out of memory"); return; }
        memset(state, 0, sizeof(*state));
        state->lfs = lfs;
        state->cfg.buffer = state->cache;
        strncpy(state->path, leaf, sizeof(state->path) - 1);
    }

    fatrd_store_lock();
    int rc = 0;
    if (!*write_state) {
        int flags = LFS_O_WRONLY | LFS_O_CREAT;
        if (fw->offset == 0) flags |= LFS_O_TRUNC;
        rc = lfs_file_opencfg(lfs, &state->file, leaf, flags, &state->cfg);
        if (rc >= 0) *write_state = state;
    }
    if (rc >= 0 && fw->offset)
        rc = lfs_file_seek(lfs, &state->file, fw->offset,
                           LFS_SEEK_SET) < 0 ? -1 : 0;
    if (rc >= 0) {
        lfs_ssize_t n = lfs_file_write(lfs, &state->file, fw->data.bytes,
                                       fw->data.size);
        if (n != (lfs_ssize_t)fw->data.size) rc = -1;
    }
    bool closed = false;
    if (rc >= 0 && fw->last) {
        closed = true;
        if (lfs_file_close(lfs, &state->file) < 0) rc = -1;
    }
    fatrd_store_unlock();

    if (rc < 0) {
        /* If open failed, state was never published and has no live handle.
         * Otherwise close best-effort before releasing its private cache. */
        if (closed) {
            *write_state = NULL;
            vPortFree(state);
        } else if (*write_state) {
            (void)file_write_close(write_state);
        }
        else vPortFree(state);
        send_error(s, req->id, "write failed");
        return;
    }
    if (fw->last) {
        *write_state = NULL;
        vPortFree(state);
        fatrd_invalidate();
    }
    send_ok(s, req->id, "ok");
}

typedef struct {
    fantasi_proto_session_t *session;
    uint32_t request_id;
} dir_context_t;

static void directory_entry(const char *name, uint32_t size, bool is_dir, void *ctx)
{
    dir_context_t *d = ctx;
    send_dir_entry(d->session, d->request_id, name, size, is_dir);
}

static PROTO_HANDLER void handle_dir_list(fantasi_proto_session_t *s,
                                          CliRequest *req)
{
    dir_context_t d = { s, req->id };
    if (vfs_list(req->payload.dir_list.path, directory_entry, &d) == 0)
        send_ok(s, req->id, "");
    else
        send_error(s, req->id, "not a directory");
}

static PROTO_HANDLER void handle_file_delete(fantasi_proto_session_t *s,
                                             CliRequest *req)
{
    if (vfs_remove(req->payload.file_delete.path) == 0)
        send_ok(s, req->id, "ok");
    else
        send_error(s, req->id, "delete failed");
}

static PROTO_HANDLER void handle_mkdir(fantasi_proto_session_t *s,
                                       CliRequest *req)
{
    if (vfs_mkdir(req->payload.mkdir.path) == 0)
        send_ok(s, req->id, "ok");
    else
        send_error(s, req->id, "mkdir failed");
}

static PROTO_HANDLER void handle_file_rename(fantasi_proto_session_t *s,
                                             CliRequest *req)
{
    int rc = vfs_rename(req->payload.file_rename.src,
                        req->payload.file_rename.dst);
    if (rc == 0) {
#if defined(FANTASI_ENABLE_APPS) && defined(APP_KEEP_SESSION_WORKER)
        /* The rename is the host's publication boundary. Once the requested
         * RAMFS module is visible, this worker has done its job; let it retire
         * so its stack is reclaimed before the app executes that module. */
        if (s_app_session == s && vfs_is_ramfs(req->payload.file_rename.dst)) {
            taskENTER_CRITICAL();
            s_app_worker_hold = false;
            taskEXIT_CRITICAL();
        }
#endif
        send_ok(s, req->id, "ok");
    }
    else if (rc == VFS_ERR_XDEV) send_error(s, req->id, "cross-device");
    else send_error(s, req->id, "rename failed");
}

#ifdef FANTASI_ENABLE_APPS
static void app_output_cb(uint32_t id, const char *data, size_t len)
{
    fantasi_proto_session_t *s = s_app_session;
    if (s && id == s_app_request)
        send_output(s, id, (const uint8_t *)data, len, true);
}

static void app_module_cb(uint32_t id, const char *name)
{
    fantasi_proto_session_t *s = s_app_session;
    if (s && id == s_app_request) {
#ifdef APP_KEEP_SESSION_WORKER
        taskENTER_CRITICAL();
        s_app_worker_hold = true;
        taskEXIT_CRITICAL();
#endif
        send_module_request(s, id, name);
    }
}

static void app_done_cb(uint32_t id, int code)
{
    (void)code;
    fantasi_proto_session_t *s = s_app_session;
    if (!s || id != s_app_request) return;
    send_output(s, id, NULL, 0, false);
    taskENTER_CRITICAL();
    s_app_session = NULL;
    s_app_request = 0;
#ifdef APP_KEEP_SESSION_WORKER
    s_app_worker_hold = false;
#endif
    taskEXIT_CRITICAL();

#ifdef APP_KEEP_SESSION_WORKER
    sessions_take();
    TaskHandle_t worker = s->worker;
    sessions_give();
    if (worker) xTaskNotifyGive(worker);
#endif

    session_put(s);                         /* async-app reference */
}

static const app_session_cb_t s_app_callbacks = {
    app_output_cb, app_module_cb, app_done_cb,
};

static PROTO_HANDLER void handle_app_launch(fantasi_proto_session_t *s,
                                            CliRequest *req)
{
    bool mapped = false;
    taskENTER_CRITICAL();
    if (!s_app_session) {
        s_app_session = s;
        s_app_request = req->id;
#ifdef APP_KEEP_SESSION_WORKER
        s_app_worker_hold = true;
#endif
        mapped = true;
    }
    taskEXIT_CRITICAL();
    if (!mapped) {
        send_error(s, req->id, "an app is already running");
        return;
    }

    sessions_take();
    s->refs++;                              /* held through app_done_cb */
    sessions_give();

    int rc = app_launch_async(req->payload.app_launch, req->id,
                              &s_app_callbacks);
    if (rc < 0) {
        taskENTER_CRITICAL();
        if (s_app_session == s && s_app_request == req->id) {
            s_app_session = NULL;
            s_app_request = 0;
#ifdef APP_KEEP_SESSION_WORKER
            s_app_worker_hold = false;
#endif
        }
        taskEXIT_CRITICAL();
        session_put(s);
        send_error(s, req->id,
                   rc == -1 ? "an app is already running" :
                   rc == -2 ? "not found" : "load failed");
    }
}

static PROTO_HANDLER void handle_app_input(fantasi_proto_session_t *s,
                                           CliRequest *req)
{
    if (s_app_session != s) { send_error(s, req->id, "no app in this session"); return; }
    app_session_feed_input(req->payload.app_input.bytes,
                           req->payload.app_input.size);
    send_ok(s, req->id, "");
}

static PROTO_HANDLER void handle_app_stop(fantasi_proto_session_t *s,
                                          CliRequest *req)
{
    if (s_app_session != s) { send_error(s, req->id, "no app in this session"); return; }
    app_session_stop();
    send_ok(s, req->id, "");
}
#endif

static void dispatch_request(fantasi_proto_session_t *s, CliRequest *req,
                             proto_file_write_t **write_state)
{
    switch (req->which_payload) {
    case CliRequest_command_tag:     handle_command(s, req); break;
    case CliRequest_file_read_tag:   handle_file_read(s, req); break;
    case CliRequest_file_write_tag:  handle_file_write(s, req, write_state); break;
    case CliRequest_dir_list_tag:    handle_dir_list(s, req); break;
    case CliRequest_file_delete_tag: handle_file_delete(s, req); break;
    case CliRequest_mkdir_tag:       handle_mkdir(s, req); break;
    case CliRequest_file_rename_tag: handle_file_rename(s, req); break;
#ifdef FANTASI_ENABLE_APPS
    case CliRequest_app_launch_tag:  handle_app_launch(s, req); break;
    case CliRequest_app_input_tag:   handle_app_input(s, req); break;
    case CliRequest_app_stop_tag:    handle_app_stop(s, req); break;
#endif
    default: send_error(s, req->id, "unknown request"); break;
    }
}

/* ---- Ordered per-session workers --------------------------------------- */

static void session_worker(void *arg)
{
    fantasi_proto_session_t *s = arg;
    proto_file_write_t *write_state = NULL;
    for (;;) {
        sessions_take();
        proto_job_t *job = s->head;
        if (!job) {
            bool keep_worker = write_state && !s->closing;
#if defined(FANTASI_ENABLE_APPS) && defined(APP_KEEP_SESSION_WORKER)
            taskENTER_CRITICAL();
            keep_worker = keep_worker ||
                          (!s->closing && s_app_session == s && s_app_worker_hold);
            taskEXIT_CRITICAL();
#endif
            if (keep_worker) {
                /* The host pipelines chunks, but an ACK can briefly empty the
                 * queue. Keep the active-only handle/cache and sleep until the
                 * next chunk instead of committing metadata every 480 bytes. */
                sessions_give();
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                continue;
            }
            s->worker = NULL;
            bool destroy = (--s->refs == 0);     /* worker reference */
            sessions_give();
            if (write_state) (void)file_write_close(&write_state);
            if (destroy) destroy_session(s);
            vTaskDelete(NULL);
        }
        s->head = job->next;
        if (s->queued) s->queued--;
        sessions_give();

        CliRequest request = CliRequest_init_zero;
        pb_istream_t in = pb_istream_from_buffer(job->message, job->len);
        bool decoded = pb_decode(&in, CliRequest_fields, &request);
        vPortFree(job);

        if (!decoded) {
            if (write_state) (void)file_write_close(&write_state);
            send_error(s, 0, "decode failed");
            continue;
        }

        s->active_id = request.id;
        sessions_take();
        s->touched = xTaskGetTickCount();
        sessions_give();
        if (s->cancel_id == request.id || s->closing) {
            if (write_state) (void)file_write_close(&write_state);
            send_error(s, request.id, "cancelled");
        } else {
            if (request.which_payload != CliRequest_file_write_tag && write_state)
                (void)file_write_close(&write_state);
            dispatch_request(s, &request, &write_state);
        }
        if (s->cancel_id == request.id) s->cancel_id = 0;
        s->active_id = 0;
    }
}

static bool queue_request(fantasi_proto_session_t *s, const uint8_t *message,
                          uint16_t len, uint32_t request_id)
{
    proto_job_t *job = pvPortMalloc(sizeof(*job) + len);
    if (!job) {
        if (s->transport->ingress_can_emit)
            send_error(s, request_id, "out of memory");
        return false;
    }
    job->next = NULL;
    job->len = len;
    memcpy(job->message, message, len);

    bool spawn_failed = false;
    sessions_take();
    if (s->closing) {
        sessions_give();
        vPortFree(job);
        return false;
    }
    if (s->queued >= PROTO_SESSION_QUEUE_MAX) {
        sessions_give();
        vPortFree(job);
        if (s->transport->ingress_can_emit)
            send_error(s, request_id, "session queue full");
        return false;
    }
    /* The bounded queue is short enough that walking it costs less persistent
     * RAM than retaining a tail pointer in every idle session. */
    proto_job_t **slot = &s->head;
    while (*slot) slot = &(*slot)->next;
    *slot = job;
    s->queued++;
    s->touched = xTaskGetTickCount();
    if (!s->worker) {
        s->refs++;                              /* prospective worker reference */
        if (xTaskCreate(session_worker, "session", PROTO_SESSION_STACK, s,
                        tskIDLE_PRIORITY + 1, &s->worker) != pdPASS) {
            s->refs--;
            s->head = NULL;                     /* no older jobs without a worker */
            s->queued = 0;
            spawn_failed = true;
        }
    }
    if (s->worker) xTaskNotifyGive(s->worker);
    sessions_give();

    if (spawn_failed) {
        vPortFree(job);
        if (s->transport->ingress_can_emit)
            send_error(s, request_id, "cannot start session");
        return false;
    }
    return true;
}

/* ---- Ingress, lifecycle, cancellation ---------------------------------- */

static request_meta_t inspect_request(const uint8_t *message, uint16_t len)
{
    request_meta_t meta = {0};
    xSemaphoreTake(s_response_lock, portMAX_DELAY);
    s_probe_request = (CliRequest){0};
    pb_istream_t in = pb_istream_from_buffer(message, len);
    if (pb_decode(&in, CliRequest_fields, &s_probe_request)) {
        meta.valid = true;
        meta.id = s_probe_request.id;
        meta.payload = s_probe_request.which_payload;
        meta.has_session = s_probe_request.has_session;
        meta.session = s_probe_request.session;
        if (meta.payload == CliRequest_cancel_tag)
            meta.cancel_id = s_probe_request.payload.cancel.request_id;
    }
    xSemaphoreGive(s_response_lock);
    return meta;
}

static void cancel_session(fantasi_proto_session_t *s, uint32_t request_id)
{
    if (!request_id) request_id = s->active_id;
    if (request_id) s->cancel_id = request_id;
    TaskHandle_t worker = s->worker;
    if (worker) xTaskNotifyGive(worker);
#ifdef FANTASI_ENABLE_APPS
    if (s_app_session == s && (!request_id || request_id == s_app_request))
        app_session_stop();
#endif
}

static bool submit_to_session(fantasi_proto_session_t *s,
                              const request_meta_t *meta,
                              const uint8_t *message, uint16_t len)
{
    if (meta->payload == CliRequest_cancel_tag) {
        sessions_take();
        s->touched = xTaskGetTickCount();
        sessions_give();
        cancel_session(s, meta->cancel_id);
        return true;                            /* deliberately out-of-band */
    }
    if (meta->payload == CliRequest_session_ping_tag) {
        sessions_take();
        s->touched = xTaskGetTickCount();
        sessions_give();
        return true;                            /* heartbeat has no response */
    }
    if (meta->payload == CliRequest_session_close_tag) {
        if (!s->explicit_session) {
            send_error(s, meta->id, "legacy session cannot close itself");
            return false;
        }
        send_ok(s, meta->id, "");
        fantasi_proto_session_close(s->id, s->transport, s->link);
        return true;
    }
    if (meta->payload == CliRequest_session_open_tag) {
        send_error(s, meta->id, "session already open");
        return false;
    }
    return queue_request(s, message, len, meta->id);
}

uint32_t fantasi_proto_session_open(const fantasi_proto_transport_t *transport,
                                    uint32_t link)
{
    fantasi_proto_session_t *s = session_create(transport, link, true);
    return s ? s->id : 0;
}

bool fantasi_proto_session_touch(uint32_t id,
                                 const fantasi_proto_transport_t *transport,
                                 uint32_t link)
{
    bool found = false;
    sessions_take();
    fantasi_proto_session_t *s = find_session_locked(id, transport, link);
    if (s) {
        s->touched = xTaskGetTickCount();
        found = true;
    }
    sessions_give();
    return found;
}

bool fantasi_proto_session_submit(uint32_t session,
                                  const fantasi_proto_transport_t *transport,
                                  uint32_t link,
                                  const uint8_t *message, uint16_t message_len)
{
    if (!message || message_len == 0 || message_len > CliRequest_size) return false;
    hal_power_activity();
    request_meta_t meta = inspect_request(message, message_len);
    if (!meta.valid || !meta.has_session || meta.session != session) return false;
    /* OPEN never belongs to an existing SID. WebUSB's control endpoint also
     * owns CLOSE: acknowledging it from inside the WRITE transfer would wait
     * for a READ that cannot start until WRITE completes. BLE sessions were
     * opened in-band and may close through their routed fragment envelope. */
    if (meta.payload == CliRequest_session_open_tag ||
        (meta.payload == CliRequest_session_close_tag &&
         !transport->allow_envelope_open))
        return false;
    fantasi_proto_session_t *s = session_get(session, transport, link);
    if (!s) return false;
    bool ok = submit_to_session(s, &meta, message, message_len);
    session_put(s);
    return ok;
}

void fantasi_proto_session_close(uint32_t id,
                                 const fantasi_proto_transport_t *transport,
                                 uint32_t link)
{
    proto_job_t *discard = NULL;
    fantasi_proto_session_t *closed = NULL;
    bool destroy = false;

    sessions_take();
    fantasi_proto_session_t **pp = &s_sessions;
    while (*pp) {
        fantasi_proto_session_t *s = *pp;
        if (s->id == id && route_matches(s, transport, link)) {
            *pp = s->next;
            s->next = NULL;
            s->closing = true;
            s->cancel_id = s->active_id;
            discard = s->head;
            s->head = NULL;
            s->queued = 0;
            if (s->worker) xTaskNotifyGive(s->worker);
            closed = s;
            destroy = (--s->refs == 0);         /* drop registry reference */
            break;
        }
        pp = &(*pp)->next;
    }
    sessions_give();

    while (discard) {
        proto_job_t *next = discard->next;
        vPortFree(discard);
        discard = next;
    }
#ifdef FANTASI_ENABLE_APPS
    if (closed && s_app_session == closed) app_session_stop();
#endif
    if (destroy) destroy_session(closed);
}

void fantasi_proto_endpoint_init(fantasi_proto_endpoint_t *ep,
                                 const fantasi_proto_transport_t *transport,
                                 uint32_t link, uint8_t *frame,
                                 uint16_t frame_cap)
{
    memset(ep, 0, sizeof(*ep));
    ep->transport = transport;
    ep->link = link;
    ep->frame = frame;
    ep->frame_cap = frame_cap;
}

static fantasi_proto_session_t *endpoint_legacy(fantasi_proto_endpoint_t *ep)
{
    if (!ep->legacy)
        ep->legacy = session_create(ep->transport, ep->link, false);
    return ep->legacy;
}

static void endpoint_message(fantasi_proto_endpoint_t *ep,
                             const uint8_t *message, uint16_t len)
{
    request_meta_t meta = inspect_request(message, len);
    if (!meta.valid) {
        fantasi_proto_session_t *legacy = endpoint_legacy(ep);
        if (legacy) send_error(legacy, 0, "decode failed");
        return;
    }

    if (meta.payload == CliRequest_session_open_tag && !meta.has_session) {
        if (!ep->transport->allow_envelope_open) {
            fantasi_proto_session_t *legacy = endpoint_legacy(ep);
            if (legacy) send_error(legacy, meta.id, "transport session open unsupported");
            return;
        }
        fantasi_proto_session_t *opened = session_create(ep->transport, ep->link, true);
        if (!opened) {
            fantasi_proto_session_t *legacy = endpoint_legacy(ep);
            if (legacy) send_error(legacy, meta.id, "out of memory");
            return;
        }
        /* The response carries the allocated SID. If it cannot be delivered,
         * no host can ever name this session, so reclaim it immediately. */
        if (!send_ok(opened, meta.id, ""))
            fantasi_proto_session_close(opened->id, ep->transport, ep->link);
        return;
    }

    fantasi_proto_session_t *s;
    bool borrowed = false;
    if (meta.has_session) {
        s = session_get(meta.session, ep->transport, ep->link);
        borrowed = true;
    } else {
        s = endpoint_legacy(ep);
    }
    if (!s) return;
    submit_to_session(s, &meta, message, len);
    if (borrowed) session_put(s);
}

void fantasi_proto_endpoint_rx(fantasi_proto_endpoint_t *ep,
                               const uint8_t *data, size_t len)
{
    if (!ep || !ep->frame || ep->frame_cap < FANTASI_PROTO_FRAME_MAX) return;
    hal_power_activity();

    while (len) {
        /* Legacy protobuf clients stop a blocking request with a raw ^C. It is
         * recognized only at a frame boundary, so a byte inside a valid frame
         * can never be mistaken for cancellation. */
        if (ep->frame_len == 0 && *data == 0x03) {
            if (ep->legacy) cancel_session(ep->legacy, 0);
            data++; len--;
            continue;
        }

        size_t target = 2;
        if (ep->frame_len >= 2) {
            uint16_t message_len = (uint16_t)ep->frame[0] |
                                   ((uint16_t)ep->frame[1] << 8);
            if (message_len == 0 || message_len > CliRequest_size) {
                ep->frame_len = 0;
                continue;
            }
            target += message_len;
        }

        size_t need = target - ep->frame_len;
        size_t copy = len < need ? len : need;
        memcpy(ep->frame + ep->frame_len, data, copy);
        ep->frame_len += (uint16_t)copy;
        data += copy;
        len -= copy;

        if (ep->frame_len >= 2) {
            uint16_t message_len = (uint16_t)ep->frame[0] |
                                   ((uint16_t)ep->frame[1] << 8);
            if (message_len > CliRequest_size) {
                ep->frame_len = 0;
                continue;
            }
            if (ep->frame_len == (uint16_t)(message_len + 2)) {
                endpoint_message(ep, ep->frame + 2, message_len);
                ep->frame_len = 0;
            }
        }
    }
}

void fantasi_proto_endpoint_down(fantasi_proto_endpoint_t *ep)
{
    if (!ep) return;
    ep->frame_len = 0;
    ep->legacy = NULL;
    for (;;) {
        uint32_t id = 0;
        sessions_take();
        for (fantasi_proto_session_t *s = s_sessions; s; s = s->next) {
            if (route_matches(s, ep->transport, ep->link)) {
                id = s->id;
                break;
            }
        }
        sessions_give();
        if (!id) break;
        fantasi_proto_session_close(id, ep->transport, ep->link);
    }
}

/* ---- Session inventory (`w`) ------------------------------------------- */

typedef struct {
    fantasi_proto_session_t *ref;
    const char *transport;
    uint32_t id;
    uint32_t link;
    uint32_t active;
    uint32_t age_ms;
    uint16_t queued;
    bool explicit_session;
    bool closing;
    bool app;
} session_snapshot_t;

static bool next_snapshot(uint32_t after, session_snapshot_t *out)
{
    fantasi_proto_session_t *best = NULL;
    sessions_take();
    for (fantasi_proto_session_t *s = s_sessions; s; s = s->next)
        if (s->id > after && (!best || s->id < best->id)) best = s;
    if (best) {
        best->refs++;
        TickType_t now = xTaskGetTickCount();
        *out = (session_snapshot_t){
            .ref = best,
            .transport = best->transport->name,
            .id = best->id,
            .link = best->link,
            .active = best->active_id,
            .age_ms = (uint32_t)((now - best->touched) * 1000u / configTICK_RATE_HZ),
            .queued = best->queued,
            .explicit_session = best->explicit_session,
            .closing = best->closing,
#ifdef FANTASI_ENABLE_APPS
            .app = s_app_session == best,
#else
            .app = false,
#endif
        };
    }
    sessions_give();
    return best != NULL;
}

void fantasi_proto_write_sessions(void)
{
    cli_write("  SID  TRANSPORT  LINK  STATE    REQUEST  QUEUED  IDLE\r\n");
    uint32_t cursor = 0;
    unsigned count = 0;
    session_snapshot_t snap;
    while (next_snapshot(cursor, &snap)) {
        const char *state = snap.closing ? "closing" : snap.app ? "app" :
                            snap.active ? "running" : "idle";
        cli_printf("  %4lu  %-9s  %4lu  %-7s  %7lu  %6u  %lums%s\r\n",
                   (unsigned long)snap.id, snap.transport,
                   (unsigned long)snap.link, state,
                   (unsigned long)snap.active, (unsigned)snap.queued,
                   (unsigned long)snap.age_ms,
                   snap.explicit_session ? "" : " legacy");
        cursor = snap.id;
        count++;
        session_put(snap.ref);
    }
    cli_printf("  %u protobuf session%s\r\n", count, count == 1 ? "" : "s");
}

void fantasi_proto_reap(void)
{
    for (;;) {
        uint32_t id = 0;
        const fantasi_proto_transport_t *transport = NULL;
        uint32_t link = 0;
        sessions_take();
        /* Sample the clock under the session lock so it is ordered with
         * updates to `touched`. */
        TickType_t now = xTaskGetTickCount();
        for (fantasi_proto_session_t *s = s_sessions; s; s = s->next) {
            if (!s->explicit_session || !s->transport->lease_ms) continue;
            /* Workers, queued jobs, and transport state keep a session active.
             * Only an idle session may age out. */
            if (s->worker || s->head) continue;
            if (s->transport->has_pending && s->transport->has_pending(s->id, s->link))
                continue;
            TickType_t lease = pdMS_TO_TICKS(s->transport->lease_ms);
            if ((TickType_t)(now - s->touched) >= lease) {
                id = s->id;
                transport = s->transport;
                link = s->link;
                break;
            }
        }
        sessions_give();
        if (!id) break;
        fantasi_proto_session_close(id, transport, link);
    }
}

/* ---- BLE physical endpoint --------------------------------------------- */

#ifdef FANTASI_ENABLE_BLE_CLI
#include "ble_serial.h"
#include "../proto/ble_mux.h"

static SemaphoreHandle_t s_ble_emit_lock;
static volatile uint16_t s_ble_payload = 20;
static uint16_t s_ble_sequence;

static size_t ble_emit(uint32_t session, uint32_t link,
                       const uint8_t *frame, size_t len)
{
    (void)link;
    size_t sent = 0;
    xSemaphoreTake(s_ble_emit_lock, portMAX_DELAY);

    /* A zero SID denotes the backwards-compatible implicit BLE byte stream.
     * Explicit sessions use notification datagrams so duplicate BlueZ delivery
     * from independent StartNotify owners cannot duplicate/corrupt protobuf
     * frames. */
    if (session) {
        uint16_t packet_cap = s_ble_payload;
        uint16_t sequence = ++s_ble_sequence;
        uint8_t packet[FANTASI_BLE_MUX_PACKET_MAX];
        if (packet_cap > sizeof(packet)) packet_cap = sizeof(packet);
        if (packet_cap <= FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE) {
            xSemaphoreGive(s_ble_emit_lock);
            return 0;
        }
        packet[0] = FANTASI_BLE_MUX_RESPONSE_MAGIC_0;
        packet[1] = FANTASI_BLE_MUX_RESPONSE_MAGIC_1;
        packet[2] = FANTASI_BLE_MUX_RESPONSE_MAGIC_2;
        packet[3] = FANTASI_BLE_MUX_RESPONSE_MAGIC_3;
        packet[4] = (uint8_t)session;
        packet[5] = (uint8_t)(session >> 8);
        packet[6] = (uint8_t)(session >> 16);
        packet[7] = (uint8_t)(session >> 24);
        packet[8] = (uint8_t)sequence;
        packet[9] = (uint8_t)(sequence >> 8);
        packet[10] = (uint8_t)len;
        packet[11] = (uint8_t)(len >> 8);

        size_t payload_cap = packet_cap - FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE;
        while (sent < len && ble_serial_connected(NULL)) {
            size_t chunk = len - sent;
            if (chunk > payload_cap) chunk = payload_cap;
            packet[12] = (uint8_t)sent;
            packet[13] = (uint8_t)(sent >> 8);
            memcpy(packet + FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE,
                   frame + sent, chunk);
            size_t packet_len = FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE + chunk;
            if (ble_serial_write(packet, packet_len, NULL) != packet_len) break;
            sent += chunk;
        }
        xSemaphoreGive(s_ble_emit_lock);
        return sent;
    }

    while (sent < len && ble_serial_connected(NULL)) {
        size_t chunk = len - sent;
        if (chunk > s_ble_payload) chunk = s_ble_payload;
        size_t n = ble_serial_write(frame + sent, chunk, NULL);
        if (!n) break;
        sent += n;
    }
    xSemaphoreGive(s_ble_emit_lock);
    return sent;
}

static bool ble_connected(uint32_t session, uint32_t link)
{
    (void)session; (void)link;
    return ble_serial_connected(NULL);
}

static const fantasi_proto_transport_t s_ble_transport = {
    .name = "ble",
    .emit = ble_emit,
    .connected = ble_connected,
    .release = NULL,
    .ingress_can_emit = true,
    .allow_envelope_open = true,
    .lease_ms = 60000,
};

/* Only a request that actually spans ATT writes has an assembly allocation.
 * Idle sessions carry no receive buffer, and a short command consumes exactly
 * its encoded size for only the interval between its first and last fragment. */
typedef struct ble_partial {
    struct ble_partial *next;
    uint32_t session;
    TickType_t touched;
    uint16_t total;
    uint16_t received;
    uint8_t message[];
} ble_partial_t;

_Static_assert(sizeof(ble_partial_t) <= 16,
               "BLE fragment metadata grew beyond its RAM budget");

static ble_partial_t *s_ble_partial;

static uint16_t get_u16le(const uint8_t *in)
{
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

static uint32_t get_u32le(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static ble_partial_t **ble_partial_slot(uint32_t session)
{
    ble_partial_t **pp = &s_ble_partial;
    while (*pp && (*pp)->session != session) pp = &(*pp)->next;
    return pp;
}

static void ble_partial_drop(uint32_t session)
{
    ble_partial_t **slot = ble_partial_slot(session);
    if (!*slot) return;
    ble_partial_t *old = *slot;
    *slot = old->next;
    vPortFree(old);
}

static bool ble_mux_packet(const uint8_t *packet, size_t len)
{
    if (len < 4 || packet[0] != FANTASI_BLE_MUX_MAGIC_0 ||
        packet[1] != FANTASI_BLE_MUX_MAGIC_1 ||
        packet[2] != FANTASI_BLE_MUX_MAGIC_2 ||
        packet[3] != FANTASI_BLE_MUX_MAGIC_3)
        return false;

    /* A recognized but malformed envelope is consumed, never fed into the
     * legacy byte-stream parser where it could desynchronize a valid client. */
    if (len <= FANTASI_BLE_MUX_HEADER_SIZE) return true;
    uint32_t session = get_u32le(packet + 4);
    uint16_t total = get_u16le(packet + 8);
    uint16_t offset = get_u16le(packet + 10);
    size_t payload_len = len - FANTASI_BLE_MUX_HEADER_SIZE;
    if (!session || !total || total > CliRequest_size || offset >= total ||
        payload_len > (size_t)(total - offset)) {
        ble_partial_drop(session);
        return true;
    }

    /* Reject unknown/closed SIDs before allocating attacker-controlled state. */
    fantasi_proto_session_t *live = session_get(session, &s_ble_transport, 0);
    if (!live) {
        ble_partial_drop(session);
        return true;
    }
    session_put(live);

    ble_partial_t **slot = ble_partial_slot(session);
    if (offset == 0) {
        if (*slot) {
            ble_partial_t *old = *slot;
            *slot = old->next;
            vPortFree(old);
        }
        ble_partial_t *part = pvPortMalloc(sizeof(*part) + total);
        if (!part) return true;
        part->next = *slot;
        part->session = session;
        part->touched = xTaskGetTickCount();
        part->total = total;
        part->received = 0;
        *slot = part;
    }

    ble_partial_t *part = *ble_partial_slot(session);
    if (!part || part->total != total || part->received != offset) {
        ble_partial_drop(session);
        return true;
    }
    memcpy(part->message + offset,
           packet + FANTASI_BLE_MUX_HEADER_SIZE, payload_len);
    part->received += (uint16_t)payload_len;
    part->touched = xTaskGetTickCount();

    if (part->received == part->total) {
        ble_partial_t **complete_slot = ble_partial_slot(session);
        *complete_slot = part->next;
        fantasi_proto_session_submit(session, &s_ble_transport, 0,
                                     part->message, part->total);
        vPortFree(part);
    }
    return true;
}

static void ble_partial_reap(bool all)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t lease = pdMS_TO_TICKS(10000);
    ble_partial_t **pp = &s_ble_partial;
    while (*pp) {
        ble_partial_t *part = *pp;
        if (all || (TickType_t)(now - part->touched) >= lease) {
            *pp = part->next;
            vPortFree(part);
        } else {
            pp = &part->next;
        }
    }
}

void proto_set_mtu(uint16_t att_mtu)
{
    uint16_t payload = att_mtu > 3 ? att_mtu - 3 : 20;
    if (payload < 20) payload = 20;
    if (payload > FANTASI_BLE_MUX_PACKET_MAX)
        payload = FANTASI_BLE_MUX_PACKET_MAX;
    s_ble_payload = payload;
}

void proto_task(void *arg)
{
    (void)arg;
    static uint8_t frame[FANTASI_PROTO_FRAME_MAX];
    fantasi_proto_endpoint_t endpoint;
    fantasi_proto_endpoint_init(&endpoint, &s_ble_transport, 0,
                                frame, sizeof(frame));
    bool was_connected = false;

    for (;;) {
        ble_serial_poll();
        bool connected = ble_serial_connected(NULL);
        if (!connected && was_connected) {
            fantasi_proto_endpoint_down(&endpoint);
            ble_partial_reap(true);
        }
        was_connected = connected;

        /* Flipper's RX characteristic permits 486-byte values; Chameleon uses
         * 244. xMessageBufferReceive leaves an oversized message queued, so the
         * destination must cover the larger supported ATT write. */
        uint8_t input[512];
        size_t n = connected ? ble_serial_read(input, sizeof(input), NULL) : 0;
        if (n) {
            if (!ble_mux_packet(input, n))
                fantasi_proto_endpoint_rx(&endpoint, input, n);
        } else {
            ble_serial_wait(100);
        }
        ble_partial_reap(false);
        fantasi_proto_reap();
    }
}
#else
void proto_set_mtu(uint16_t att_mtu) { (void)att_mtu; }
void proto_task(void *arg) { (void)arg; vTaskDelete(NULL); }
#endif

void fantasi_proto_init(void)
{
    if (!s_sessions_lock) s_sessions_lock = xSemaphoreCreateMutex();
    if (!s_response_lock) s_response_lock = xSemaphoreCreateMutex();
#ifdef FANTASI_ENABLE_BLE_CLI
    if (!s_ble_emit_lock) s_ble_emit_lock = xSemaphoreCreateMutex();
#endif
}
