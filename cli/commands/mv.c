/* mv - move/rename a file. Within one filesystem the device does it in place
 * (LittleFS rename, instant). Across filesystems (e.g. / -> /mnt/ext0) the device
 * reports "cross-device" and the host falls back to copy-then-delete. Over the
 * MSC-mounted FAT it is a plain host-side rename. `mv x /dir/` moves to /dir/x. */
#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>

/* dir_target() (shared with cp) is declared in cli_internal.h. */

#ifdef HAS_PROTO
static void proto_cmd_mv(const char *args)
{
    char src[256] = "", dst[256] = "";
    if (!args || sscanf(args, "%255s %255s", src, dst) < 2) {
        fprintf(stderr, "usage: mv <src> <dst>\n");
        return;
    }
    char sp[256], dp[256];
    resolve_path(src, sp, sizeof sp);
    resolve_path(dst, dp, sizeof dp);
    dir_target(sp, dst, dp, sizeof dp);

    /* Try an in-place device rename first. */
    CliRequest req = CliRequest_init_zero;
    req.id = ++proto_req_id;
    req.which_payload = CliRequest_file_rename_tag;
    strncpy(req.payload.file_rename.src, sp, sizeof(req.payload.file_rename.src) - 1);
    strncpy(req.payload.file_rename.dst, dp, sizeof(req.payload.file_rename.dst) - 1);
    if (proto_send(&req) < 0) return;

    CliResponse resp;
    if (proto_recv(&resp) != 0) { fprintf(stderr, "mv: no response\n"); return; }
    if (resp.which_payload != CliResponse_error_tag) return;   /* renamed in place */

    if (strcmp(resp.payload.error.message, "cross-device") == 0) {
        /* Different backends: copy across, then delete the source. */
        if (proto_copy_dev(sp, dp) == 0)
            proto_cmd_rm(src);
    } else {
        fprintf(stderr, "mv: %s\n", resp.payload.error.message);
    }
}
#endif

/* MSC-FAT variant: OS-level rename over the mounted Fantasi volume. */
static void cmd_mv(const char *args)
{
    char src[256] = "", dst[256] = "";
    if (!args || sscanf(args, "%255s %255s", src, dst) < 2) {
        fprintf(stderr, "usage: mv <src> <dst>\n");
        return;
    }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char sp[256], dp[256];
    resolve_path(src, sp, sizeof sp);
    resolve_path(dst, dp, sizeof dp);
    dir_target(sp, dst, dp, sizeof dp);

    /* fat_path returns a pointer into a shared static buffer, so copy the first. */
    char from[512];
    snprintf(from, sizeof from, "%s", fat_path(sp));
    if (rename(from, fat_path(dp)) != 0)
        fprintf(stderr, "mv: %s\n", strerror(errno));
}

LOCAL_COMMAND_BLE("mv", "move/rename a file", cmd_mv, proto_cmd_mv);
