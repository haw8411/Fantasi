/* cp - copy a file. Over the protobuf transport (USB/BLE) the device has no copy
 * op, so a device->device copy round-trips through the host: download the source
 * to a temp file, then upload it to the destination. Over the MSC-mounted FAT it
 * is a plain host-side file copy. `cp x /dir/` (trailing slash) copies to
 * /dir/x. */
#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

/* dir_target() (shared with mv) is declared in cli_internal.h. */

#ifdef HAS_PROTO
/* Device->device copy via a host temp file. proto_download / proto_upload print their
 * own diagnostics; returns 0 on success, -1 on failure. */
int proto_copy_dev(const char *devsrc, const char *devdst)
{
    char tmp[] = "/tmp/fantasi_cp_XXXXXX";
    int fd = mkstemp(tmp);
    if (fd < 0) { fprintf(stderr, "cp: mkstemp: %s\n", strerror(errno)); return -1; }
    FILE *f = fdopen(fd, "w+b");
    if (!f) { close(fd); unlink(tmp); fprintf(stderr, "cp: temp file\n"); return -1; }

    int rc = proto_download(devsrc, f);
    fflush(f);
    fclose(f);
    if (rc == 0) rc = proto_upload(tmp, devdst);
    unlink(tmp);
    return rc;
}

static void proto_cmd_cp(const char *args)
{
    char src[256] = "", dst[256] = "";
    if (!args || sscanf(args, "%255s %255s", src, dst) < 2) {
        fprintf(stderr, "usage: cp <src> <dst>\n");
        return;
    }
    char sp[256], dp[256];
    resolve_path(src, sp, sizeof sp);
    resolve_path(dst, dp, sizeof dp);
    dir_target(sp, dst, dp, sizeof dp);
    proto_copy_dev(sp, dp);
}
#endif

/* MSC-FAT variant: OS-level copy over the mounted Fantasi volume. */
static void cmd_cp(const char *args)
{
    char src[256] = "", dst[256] = "";
    if (!args || sscanf(args, "%255s %255s", src, dst) < 2) {
        fprintf(stderr, "usage: cp <src> <dst>\n");
        return;
    }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char sp[256], dp[256];
    resolve_path(src, sp, sizeof sp);
    resolve_path(dst, dp, sizeof dp);
    dir_target(sp, dst, dp, sizeof dp);

    FILE *in = fopen(fat_path(sp), "rb");
    if (!in) { fprintf(stderr, "cp: cannot open %s: %s\n", sp, strerror(errno)); return; }
    FILE *out = fopen(fat_path(dp), "wb");
    if (!out) { fprintf(stderr, "cp: cannot create %s\n", dp); fclose(in); return; }

    char buf[4096]; size_t n;
    while ((n = fread(buf, 1, sizeof buf, in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) { fprintf(stderr, "cp: write error\n"); break; }
    }
    fclose(in);
    fclose(out);
}

LOCAL_COMMAND_BLE("cp", "copy a file", cmd_cp, proto_cmd_cp);
