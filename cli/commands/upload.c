#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <unistd.h>

static void cmd_upload(const char *args)
{
    char local[256] = "", remote[256] = "";
    int nargs = args ? sscanf(args, "%255s %255s", local, remote) : 0;
    if (nargs < 1) {
        fprintf(stderr, "usage: upload <local-path> [remote-path]\n");
        return;
    }
    if (nargs < 2) strcpy(remote, ".");   /* cwd + basename, matching protobuf upload */

    FILE *fp = fopen(local, "rb");
    if (!fp) { fprintf(stderr, "cannot open %s: %s\n", local, strerror(errno)); return; }

    if (!fat_mount()) {
        fprintf(stderr, "no filesystem\n");
        fclose(fp); return;
    }

    char rpath[256];
    resolve_path(remote, rpath, sizeof(rpath));
    /* Match normal copy semantics: a directory destination receives the local
     * file's basename. stat covers an existing directory named without a
     * trailing slash; otherwise dir_target preserves syntactic intent lost
     * during path normalization (`.` -> cwd and a trailing slash is removed). */
    struct stat dst_st;
    bool existing_dir = stat(fat_path(rpath), &dst_st) == 0 &&
                        S_ISDIR(dst_st.st_mode);
    dir_target(local, existing_dir ? "." : remote, rpath, sizeof(rpath));

    FILE *out = fopen(fat_path(rpath), "wb");
    if (!out) {
        fprintf(stderr, "cannot create %s\n", rpath);
        fclose(fp); return;
    }

    char buf[4096];
    size_t n, total = 0;
    bool ok = true;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (fwrite(buf, 1, n, out) != n) { ok = false; break; }
        total += n;
        /* Force FAT data + the current directory size to the device in bounded
         * increments. Firmware can then retire the staged prefix instead of
         * retaining one 520-byte staging node per sector for the whole file. */
        if (fflush(out) != 0 || fsync(fileno(out)) != 0 || !fat_sync()) {
            ok = false;
            break;
        }
    }
    fclose(fp);
    if (fclose(out) != 0) ok = false;

    if (!ok) { fprintf(stderr, "write failed\n"); return; }

    if (!fat_sync()) { fprintf(stderr, "upload was not committed\n"); return; }
    printf("%s -> %s (%zu bytes)\n", local, rpath, total);
}

#ifdef HAS_PROTO
/* Pipelined upload of `local_path` to device `remote_path` (already resolved). Returns 0, or -1 on error.
 * Shared by `upload` and `edit`'s WebUSB write-back of an edited temp file. */
int proto_upload(const char *local_path, const char *remote_path)
{
    FILE *f = fopen(local_path, "rb");
    if (!f) { perror(local_path); return -1; }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    /* Pipelined upload: keep up to UPLOAD_WINDOW FileWriteChunks in flight
     * instead of waiting for an ack after each one. The device processes
     * chunks in order and acks each; in-order acks advance the window, so
     * the per-chunk round-trip is hidden. Chunk size stays 480 B - large
     * single chunks are unreliable; the win is pipelining, not bigger
     * messages.
     *
     * Exception: the Proxmark3 (switch-mode, g_switch_mode) has a SAM7S UDP with
     * only two 64-byte ping-pong OUT banks and a slow ARM7 draining them. Under
     * pipelining the host outruns the device, both banks fill, and the DCD's
     * dual-bank handling stalls the endpoint. Pace it to a window of 1 - ack each
     * chunk (i.e. wait until the device has drained + written it) before sending
     * the next, so the banks never overflow. */
    #define UPLOAD_WINDOW 6
    int window = g_switch_mode ? 1 : UPLOAD_WINDOW;
    #define UPLOAD_CHUNK  480
    #define UPLOAD_MAX_RETRANSMIT 30
    uint8_t chunk[UPLOAD_CHUNK];
    uint32_t total = (uint32_t)((fsize + UPLOAD_CHUNK - 1) / UPLOAD_CHUNK);
    uint32_t off = 0, sent = 0, acked = 0;
    int retransmits = 0;
    bool error = false;
    /* FIFO of req ids for chunks sent but not yet acked. Acks MUST be matched
     * to the oldest in-flight chunk: the device writes each chunk at its
     * absolute offset (idempotent), so a LOST WRITE makes it write/ack a later
     * offset, leaving a hole. Blindly counting acks never detects that hole and
     * silently produces a corrupt file. An out-of-order ack id is the signal a
     * write was lost → rewind to the last contiguously-acked chunk and resend. */
    uint32_t inflight[UPLOAD_WINDOW];
    int inf_head = 0, inf_count = 0;

    /* Drain any stale data once, up front; the window loop must not drain
     * again or it would eat the acks it depends on. */
    proto_drain_quiet();

    while (acked < total && !error) {
        while (sent < total && inf_count < window) {
            size_t n = fread(chunk, 1, sizeof(chunk), f);
            if (n == 0) { total = sent; break; }   /* short read / EOF */

            CliRequest req = CliRequest_init_zero;
            req.id = ++proto_req_id;
            req.which_payload = CliRequest_file_write_tag;
            strncpy(req.payload.file_write.path, remote_path,
                    sizeof(req.payload.file_write.path) - 1);
            req.payload.file_write.offset = off;
            memcpy(req.payload.file_write.data.bytes, chunk, n);
            req.payload.file_write.data.size = (pb_size_t)n;
            req.payload.file_write.last = (off + n >= (uint32_t)fsize);
            /* Size hint so the device can pre-allocate (ramfs) instead of
             * growing per chunk; harmless on flash targets that ignore it. */
            req.payload.file_write.has_total = true;
            req.payload.file_write.total = (uint32_t)fsize;

            if (proto_write_req(&req) < 0) {
                fprintf(stderr, "send failed\n");
                error = true;
                break;
            }
            inflight[(inf_head + inf_count) % UPLOAD_WINDOW] = req.id;
            inf_count++;
            off += (uint32_t)n;
            sent++;
        }
        if (error || acked >= total) break;

        CliResponse resp;
        bool rewind = false;
        if (proto_recv(&resp) < 0) {
            rewind = true;                         /* no ack within timeout */
        } else if (resp.which_payload == CliResponse_error_tag) {
            fprintf(stderr, "error: %s\n", resp.payload.error.message);
            error = true;
        } else if (inf_count > 0 && resp.id == inflight[inf_head]) {
            inf_head = (inf_head + 1) % UPLOAD_WINDOW;
            inf_count--;
            acked++;
            retransmits = 0;                       /* progress: reset the budget */
            printf("\r  %u / %ld bytes",
                   (unsigned)(acked >= total ? (uint32_t)fsize : acked * UPLOAD_CHUNK),
                   fsize);
            fflush(stdout);
        } else {
            rewind = true;                         /* out-of-order ack = lost write/hole */
        }

        if (rewind && !error) {
            /* Flash page erase can briefly stall the radio and drop a chunk or
             * its ack; writes are idempotent, so rewind the window to the last
             * acked chunk and resend rather than failing the transfer. */
            if (++retransmits > UPLOAD_MAX_RETRANSMIT) {
                fprintf(stderr, "upload failed (too many retransmits)\n");
                error = true;
                break;
            }
            sent = acked;
            off  = acked * UPLOAD_CHUNK;
            fseek(f, off, SEEK_SET);
            inf_head = inf_count = 0;
            ble_transport_process();           /* discard any stale partial data */
            { char d[256]; while (ble_transport_read(d, sizeof(d)) > 0) {} }
            proto_rx_len = 0;
        }
    }
    fclose(f);
    printf("\n");
    return error ? -1 : 0;
}

/* True if device `path` is a directory. Check the parent listing for an entry
 * named `path`'s basename with is_dir set; this recognizes empty directories
 * without walking the target directory itself. The device reports an error for
 * a missing parent instead of treating it as a successful empty listing. */
static bool proto_path_is_dir(const char *path)
{
    const char *slash = strrchr(path, '/');
    if (!slash) return false;                          /* resolved paths are absolute */
    const char *base = slash + 1;
    if (!*base) return true;                            /* trailing slash: a directory path */

    char parent[256];
    size_t pl = (slash == path) ? 1 : (size_t)(slash - path);   /* root parent stays "/" */
    if (pl >= sizeof parent) return false;
    memcpy(parent, path, pl);
    parent[pl] = '\0';

    CliRequest req = CliRequest_init_zero;
    req.id = ++proto_req_id;
    req.which_payload = CliRequest_dir_list_tag;
    strncpy(req.payload.dir_list.path, parent, sizeof(req.payload.dir_list.path) - 1);
    if (proto_send(&req) < 0) return false;
    CliResponse resp;
    bool is_dir = false;
    do {
        if (proto_recv(&resp) < 0) return false;
        if (resp.which_payload == CliResponse_dir_entry_tag &&
            resp.payload.dir_entry.is_dir &&
            strcmp(resp.payload.dir_entry.name, base) == 0)
            is_dir = true;
    } while (resp.has_next);
    return is_dir;
}

static void proto_cmd_upload(const char *args)
{
    if (!args) { fprintf(stderr, "usage: upload <local> [remote]\n"); return; }
    char local_path[128], remote_arg[64] = "";
    sscanf(args, "%127s %63s", local_path, remote_arg);

    const char *base = strrchr(local_path, '/');
    base = base ? base + 1 : local_path;

    char remote_path[256];
    if (remote_arg[0]) {
        resolve_path(remote_arg, remote_path, sizeof(remote_path));
        // `upload x /dir` -> /dir/x
        if (proto_path_is_dir(remote_path)) {
            size_t l = strlen(remote_path);
            snprintf(remote_path + l, sizeof(remote_path) - l, "%s%s",
                     (l && remote_path[l-1] == '/') ? "" : "/", base);
        }
    } else {
        snprintf(remote_path, sizeof(remote_path), "%s%s%s",
                 cwd, (cwd[strlen(cwd)-1] == '/') ? "" : "/", base);
    }
    proto_upload(local_path, remote_path);
}
#endif

LOCAL_COMMAND_BLE("upload", "copy host file to device", cmd_upload, proto_cmd_upload);
