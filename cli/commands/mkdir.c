#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

static void cmd_mkdir(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: mkdir <dir>\n"); return; }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    if (mkdir(fat_path(path), 0777) < 0)
        fprintf(stderr, "cannot create %s: %s\n", path, strerror(errno));
    else
        fat_sync();
}

#ifdef HAS_PROTO
static void proto_cmd_mkdir(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: mkdir <path>\n"); return; }
    char path[256];
    resolve_path(arg, path, sizeof(path));
    CliRequest req = CliRequest_init_zero;
    req.id = ++proto_req_id;
    req.which_payload = CliRequest_mkdir_tag;
    strncpy(req.payload.mkdir.path, path,
            sizeof(req.payload.mkdir.path) - 1);
    if (proto_send(&req) < 0) return;
    CliResponse resp;
    if (proto_recv(&resp) == 0 && resp.which_payload == CliResponse_error_tag)
        fprintf(stderr, "error: %s\n", resp.payload.error.message);
}
#endif

LOCAL_COMMAND_BLE("mkdir", "create a directory", cmd_mkdir, proto_cmd_mkdir);
