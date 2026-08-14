#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int)(crc & 1));
    }
    return ~crc;
}

/* crc32 <path> - print "<hex8> <bytes> <path>" for a device file. Lets the
 * flasher and tests compare contents without capturing binary over the link. */
static void cmd_crc32(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: crc32 <file>\n"); return; }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    FILE *f = fopen(fat_path(path), "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return; }

    uint32_t crc = 0;
    size_t total = 0, n;
    uint8_t buf[4096];
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        crc = crc32_update(crc, buf, n);
        total += n;
    }
    fclose(f);
    printf("%08x %zu %s\n", crc, total, path);
}

#ifdef HAS_PROTO
/* crc32 over BLE: read the file in bounded windows (same resume logic as
 * ble cat) and fold each chunk into the running CRC instead of printing. */
static void proto_cmd_crc32(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: crc32 <path>\n"); return; }
    char path[256];
    resolve_path(arg, path, sizeof(path));

    uint32_t got = 0, crc = 0;
    int stalls = 0;
    bool eof = false;

    while (!eof) {
        CliRequest req = CliRequest_init_zero;
        req.id = ++proto_req_id;
        req.which_payload = CliRequest_file_read_tag;
        strncpy(req.payload.file_read.path, path,
                sizeof(req.payload.file_read.path) - 1);
        req.payload.file_read.offset = got;
        req.payload.file_read.size = CAT_WINDOW;
        if (proto_send(&req) < 0) { fprintf(stderr, "send failed\n"); return; }

        uint32_t before = got;
        bool last_seen = false, err_seen = false;
        CliResponse resp;
        for (;;) {
            if (proto_recv(&resp) < 0) break;
            if (resp.id != req.id) continue;
            if (resp.which_payload == CliResponse_error_tag) { err_seen = true; break; }
            if (resp.which_payload == CliResponse_file_data_tag) {
                uint32_t off = resp.payload.file_data.offset;
                uint32_t sz  = resp.payload.file_data.data.size;
                const uint8_t *b = resp.payload.file_data.data.bytes;
                if (off > got) break;
                if (off + sz > got) {
                    uint32_t skip = got - off;
                    crc = crc32_update(crc, b + skip, sz - skip);
                    got += sz - skip;
                }
                if (resp.payload.file_data.last) { last_seen = true; break; }
            }
            if (!resp.has_next) break;
        }

        uint32_t delivered = got - before;
        if (err_seen) {
            if (got > 0) { eof = true; }
            else { fprintf(stderr, "error: %s\n", resp.payload.error.message); return; }
        } else if (last_seen && delivered < CAT_WINDOW) {
            eof = true;
        } else if (last_seen) {
            stalls = 0;
        } else {
            if (delivered > 0) stalls = 0;
            else if (++stalls > 40) {
                fprintf(stderr, "crc32: download stalled at %u bytes\n", got);
                return;
            }
            proto_drain_quiet();
        }
    }
    printf("%08x %u %s\n", crc, got, path);
}
#endif

LOCAL_COMMAND_BLE("crc32", "CRC32 of a device file", cmd_crc32, proto_cmd_crc32);
