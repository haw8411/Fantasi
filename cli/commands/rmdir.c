#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

static void cmd_rmdir(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: rmdir <dir>\n"); return; }
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    if (rmdir(fat_path(path)) < 0)
        fprintf(stderr, "cannot remove %s: %s\n", path, strerror(errno));
    else
        fat_sync();
}

/* Over BLE, directory removal goes through the same file_delete path as rm. */
LOCAL_COMMAND_BLE("rmdir", "remove empty directory", cmd_rmdir, proto_cmd_rm);
