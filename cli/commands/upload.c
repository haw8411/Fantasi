#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

static void cmd_upload(const char *args)
{
    char local[256] = "", remote[256] = "";
    if (!args || sscanf(args, "%255s %255s", local, remote) < 2) {
        fprintf(stderr, "usage: upload <local-path> <remote-path>\n");
        return;
    }

    FILE *fp = fopen(local, "rb");
    if (!fp) { fprintf(stderr, "cannot open %s: %s\n", local, strerror(errno)); return; }

    if (!fat_mount()) {
        fprintf(stderr, "no filesystem\n");
        fclose(fp); return;
    }

    char rpath[256];
    resolve_path(remote, rpath, sizeof(rpath));

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
    }
    fclose(fp);
    if (fclose(out) != 0) ok = false;

    if (!ok) { fprintf(stderr, "write failed\n"); return; }

    fat_sync();
    printf("%s -> %s (%zu bytes)\n", local, rpath, total);
}

#ifdef HAS_BLE
static void ble_cmd_upload(const char *args)
{
    if (!args) { fprintf(stderr, "usage: upload <local> [remote]\n"); return; }
    char local_path[128], remote_arg[64] = "";
    sscanf(args, "%127s %63s", local_path, remote_arg);

    char remote_path[256];
    if (remote_arg[0]) {
        resolve_path(remote_arg, remote_path, sizeof(remote_path));
    } else {
        const char *base = strrchr(local_path, '/');
        base = base ? base + 1 : local_path;
        snprintf(remote_path, sizeof(remote_path), "%s%s%s",
                 cwd, (cwd[strlen(cwd)-1] == '/') ? "" : "/", base);
    }

    FILE *f = fopen(local_path, "rb");
    if (!f) { perror(local_path); return; }
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
     * dual-bank handling wedges the endpoint. Pace it to a window of 1 - ack each
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
    ble_transport_process();
    { char d[256]; while (ble_transport_read(d, sizeof(d)) > 0) {} }
    ble_rx_len = 0;

    while (acked < total && !error) {
        while (sent < total && inf_count < window) {
            size_t n = fread(chunk, 1, sizeof(chunk), f);
            if (n == 0) { total = sent; break; }   /* short read / EOF */

            CliRequest req = CliRequest_init_zero;
            req.id = ++ble_req_id;
            req.which_payload = CliRequest_file_write_tag;
            strncpy(req.payload.file_write.path, remote_path,
                    sizeof(req.payload.file_write.path) - 1);
            req.payload.file_write.offset = off;
            memcpy(req.payload.file_write.data.bytes, chunk, n);
            req.payload.file_write.data.size = (pb_size_t)n;
            req.payload.file_write.last = (off + n >= (uint32_t)fsize);

            if (ble_write_req(&req) < 0) {
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
        if (ble_recv_proto(&resp) < 0) {
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
            ble_rx_len = 0;
        }
    }
    fclose(f);
    printf("\n");
}
#endif

LOCAL_COMMAND_BLE("upload", "copy host file to device", cmd_upload, ble_cmd_upload);
