#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>

static void cmd_ls(const char *arg)
{
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    DIR *d = opendir(fat_path(path));
    if (!d) { fprintf(stderr, "cannot open %s\n", path); return; }

    struct dirent *e;
    while ((e = readdir(d))) {
        if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0)
            continue;
        char full[600];
        snprintf(full, sizeof(full), "%s/%s", fat_path(path), e->d_name);
        struct stat st;
        if (stat(full, &st) == 0 && S_ISDIR(st.st_mode))
            printf("  %s/\n", e->d_name);
        else if (stat(full, &st) == 0)
            printf("  %-20s %lu\n", e->d_name, (unsigned long)st.st_size);
        else
            printf("  %s\n", e->d_name);
    }
    closedir(d);
}

#ifdef HAS_PROTO
static void proto_cmd_ls(const char *arg)
{
    char path[256];
    resolve_path(arg, path, sizeof(path));

    CliRequest req = CliRequest_init_zero;
    req.id = ++proto_req_id;
    req.which_payload = CliRequest_dir_list_tag;
    strncpy(req.payload.dir_list.path, path,
            sizeof(req.payload.dir_list.path) - 1);
    if (proto_send(&req) < 0) { fprintf(stderr, "send failed\n"); return; }

    CliResponse resp;
    do {
        if (proto_recv(&resp) < 0) break;
        if (resp.which_payload == CliResponse_dir_entry_tag) {
            DirEntry *e = &resp.payload.dir_entry;
            if (e->is_dir)
                printf("  %-20s  <dir>\n", e->name);
            else
                printf("  %-20s  %u\n", e->name, (unsigned)e->size);
        } else if (resp.which_payload == CliResponse_error_tag) {
            fprintf(stderr, "error: %s\n", resp.payload.error.message);
        } else if (resp.which_payload == CliResponse_output_tag) {
            /* empty dir */
        }
    } while (resp.has_next);
}
#endif

LOCAL_COMMAND_BLE("ls", "list files", cmd_ls, proto_cmd_ls);
