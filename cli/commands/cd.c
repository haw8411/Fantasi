#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static void cmd_cd(const char *arg)
{
    if (!arg || !arg[0]) { strcpy(cwd, "/"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    struct stat st;
    if (stat(fat_path(path), &st) != 0 || !S_ISDIR(st.st_mode)) {
        fprintf(stderr, "not a directory: %s\n", path);
        return;
    }
    snprintf(cwd, sizeof(cwd), "%s", path);
}

#ifdef HAS_BLE
static void ble_cmd_cd(const char *arg)
{
    if (!arg || !arg[0]) { strcpy(cwd, "/"); return; }
    char path[256];
    resolve_path(arg, path, sizeof(path));

    CliRequest req = CliRequest_init_zero;
    req.id = ++ble_req_id;
    req.which_payload = CliRequest_dir_list_tag;
    strncpy(req.payload.dir_list.path, path,
            sizeof(req.payload.dir_list.path) - 1);
    if (ble_send_proto(&req) < 0) return;

    CliResponse resp;
    bool failed = false;
    do {
        if (ble_recv_proto(&resp) < 0) { failed = true; break; }
        if (resp.which_payload == CliResponse_error_tag) {
            fprintf(stderr, "not a directory: %s\n", path);
            failed = true;
        }
    } while (resp.has_next);
    if (!failed) snprintf(cwd, sizeof(cwd), "%s", path);
}
#endif

LOCAL_COMMAND_BLE("cd", "change directory", cmd_cd, ble_cmd_cd);
