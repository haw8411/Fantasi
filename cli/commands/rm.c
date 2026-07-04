#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

static void cmd_rm(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: rm <file>\n"); return; }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    if (unlink(fat_path(path)) < 0)
        fprintf(stderr, "cannot remove %s: %s\n", path, strerror(errno));
    else
        fat_sync();
}

#ifdef HAS_BLE
/* Non-static: rmdir reuses this BLE handler (device file_delete removes dirs too). */
void ble_cmd_rm(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: rm <path>\n"); return; }
    char path[256];
    resolve_path(arg, path, sizeof(path));
    CliRequest req = CliRequest_init_zero;
    req.id = ++ble_req_id;
    req.which_payload = CliRequest_file_delete_tag;
    strncpy(req.payload.file_delete.path, path,
            sizeof(req.payload.file_delete.path) - 1);
    if (ble_send_proto(&req) < 0) return;
    CliResponse resp;
    if (ble_recv_proto(&resp) == 0 && resp.which_payload == CliResponse_error_tag)
        fprintf(stderr, "error: %s\n", resp.payload.error.message);
}
#endif

LOCAL_COMMAND_BLE("rm", "delete a file", cmd_rm, ble_cmd_rm);
