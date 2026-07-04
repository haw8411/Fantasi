#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static void cmd_cat(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: cat <file>\n"); return; }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    FILE *f = fopen(fat_path(path), "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return; }

    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        fwrite(buf, 1, n, stdout);
    fclose(f);
}

#ifdef HAS_BLE
/* Resumable, windowed download over BLE: request the file in bounded CAT_WINDOW
 * ranges so a dropped notification only re-requests one window. See the long note
 * in the protocol docs; ble_drain_quiet() clears stale stream bytes before a
 * resume. */
static void ble_cmd_cat(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: cat <path>\n"); return; }
    char path[256];
    resolve_path(arg, path, sizeof(path));

    uint32_t got = 0;          /* contiguous bytes written so far */
    int stalls = 0;            /* consecutive re-requests with no progress */
    bool eof = false;

    while (!eof) {
        CliRequest req = CliRequest_init_zero;
        req.id = ++ble_req_id;
        req.which_payload = CliRequest_file_read_tag;
        strncpy(req.payload.file_read.path, path,
                sizeof(req.payload.file_read.path) - 1);
        req.payload.file_read.offset = got;
        req.payload.file_read.size = CAT_WINDOW;
        if (ble_send_proto(&req) < 0) { fprintf(stderr, "send failed\n"); return; }

        uint32_t before = got;
        bool last_seen = false, err_seen = false;
        CliResponse resp;
        for (;;) {
            if (ble_recv_proto(&resp) < 0) break;          /* desync/timeout → resume window */
            if (resp.id != req.id) continue;               /* stale response */
            if (resp.which_payload == CliResponse_error_tag) {
                err_seen = true; break;
            }
            if (resp.which_payload == CliResponse_file_data_tag) {
                uint32_t off = resp.payload.file_data.offset;
                uint32_t sz  = resp.payload.file_data.data.size;
                const uint8_t *b = resp.payload.file_data.data.bytes;
                if (off > got) break;                      /* gap → resume from `got` */
                if (off + sz > got) {                      /* new (or partially new) bytes */
                    uint32_t skip = got - off;
                    fwrite(b + skip, 1, sz - skip, stdout);
                    got += sz - skip;
                }
                if (resp.payload.file_data.last) { last_seen = true; break; }
            }
            if (!resp.has_next) break;
        }

        uint32_t delivered = got - before;
        if (err_seen) {
            if (got > 0) { eof = true; }                   /* error at/after EOF window */
            else { fprintf(stderr, "error: %s\n", resp.payload.error.message); break; }
        } else if (last_seen && delivered < CAT_WINDOW) {
            eof = true;                                    /* final (short) window */
        } else if (last_seen) {
            stalls = 0;                                    /* full window, more to come */
        } else {
            /* window incomplete (gap/timeout) - resume it from `got` */
            if (delivered > 0) stalls = 0;
            else if (++stalls > 40) {
                fprintf(stderr, "cat: download stalled at %u bytes\n", got);
                break;
            }
            ble_drain_quiet();                             /* clear stale stream before resume */
        }
    }
    fflush(stdout);
}
#endif

LOCAL_COMMAND_BLE("cat", "print file contents", cmd_cat, ble_cmd_cat);
