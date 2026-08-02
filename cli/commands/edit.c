/* `edit <file>` - open a device file in the user's editor.
 *
 *   - MSC available (serial + a mountable FAT): edit the mounted file in place, then fat_sync().
 *   - WebUSB / BLE (no MSC): download the file to a temp path (the cat download), edit that, and on save
 *     upload it back (the upload path) to overwrite the device file. A still-serial session is upgraded to
 *     WebUSB first, since the temp path needs the protobuf file transfer.
 *
 * The editor inherits the terminal and runs to completion; exiting it drops the user back at the prompt. */
#include "cli_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/wait.h>

/* $EDITOR if set, else the first of nano / vim / vi found on PATH. NULL if none. */
static const char *find_editor(void)
{
    const char *e = getenv("EDITOR");
    if (e && *e) return e;

    static const char *cands[] = { "nano", "vim", "vi" };
    const char *path = getenv("PATH");
    if (!path) path = "/usr/bin:/bin";
    for (unsigned c = 0; c < sizeof cands / sizeof cands[0]; c++) {
        char *pd = strdup(path);
        if (!pd) return NULL;
        const char *found = NULL;
        for (char *dir = strtok(pd, ":"); dir; dir = strtok(NULL, ":")) {
            char full[512];
            snprintf(full, sizeof full, "%s/%s", dir, cands[c]);
            if (access(full, X_OK) == 0) { found = cands[c]; break; }
        }
        free(pd);
        if (found) return found;
    }
    return NULL;
}

/* Run `editor path`, inheriting the terminal, and wait. Returns 0 on a clean (exit 0) editor exit. */
static int run_editor(const char *editor, const char *path)
{
    fflush(stdout);
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); return -1; }
    if (pid == 0) {
        execlp(editor, editor, path, (char *)NULL);
        fprintf(stderr, "edit: cannot run %s: %s\n", editor, strerror(errno));
        _exit(127);
    }
    int st;
    while (waitpid(pid, &st, 0) < 0 && errno == EINTR) { }
    return (WIFEXITED(st) && WEXITSTATUS(st) == 0) ? 0 : -1;
}

#ifdef HAS_BLE
/* Download `devpath` to a temp file, edit it, and upload it back only if the editor saved a change. */
static void edit_via_temp(const char *editor, const char *devpath)
{
    char tmp[] = "/tmp/fantasi-edit-XXXXXX";
    int fd = mkstemp(tmp);
    if (fd < 0) { perror("edit: mkstemp"); return; }
    FILE *out = fdopen(fd, "wb");
    if (!out) { perror("edit: fdopen"); close(fd); unlink(tmp); return; }

    if (ble_download(devpath, out) < 0) { fclose(out); unlink(tmp); return; }   /* error already printed */
    fclose(out);

    struct stat before = {0}, after = {0};
    stat(tmp, &before);
    int r = run_editor(editor, tmp);
    stat(tmp, &after);

    bool changed = before.st_size != after.st_size ||
                   before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
                   before.st_mtim.tv_nsec != after.st_mtim.tv_nsec;
    if (r == 0 && changed) {
        printf("saving %s ...\n", devpath);
        ble_upload(tmp, devpath);
    } else if (r == 0) {
        printf("no changes\n");
    }
    unlink(tmp);
}
#endif

static void cmd_edit(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: edit <file>\n"); return; }
    const char *editor = find_editor();
    if (!editor) { fprintf(stderr, "edit: no editor found (set $EDITOR, or install nano/vim/vi)\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));

    if (fat_mount()) {                                      /* MSC available: edit the mounted file directly */
        char host[512];
        snprintf(host, sizeof host, "%s", fat_path(path));
        run_editor(editor, host);
        fat_sync();
        return;
    }

    /* No MSC: use the WebUSB/BLE temp-file path, upgrading a still-serial session to WebUSB for it. */
#if defined(HAS_USB_VENDOR) && defined(HAS_BLE)
    if (use_usb || try_webusb_upgrade(false)) { edit_via_temp(editor, path); return; }
#endif
    fprintf(stderr, "edit: no filesystem available\n");
}

#ifdef HAS_BLE
static void ble_cmd_edit(const char *arg)
{
    if (!arg) { fprintf(stderr, "usage: edit <file>\n"); return; }
    const char *editor = find_editor();
    if (!editor) { fprintf(stderr, "edit: no editor found (set $EDITOR, or install nano/vim/vi)\n"); return; }

    char path[256];
    resolve_path(arg, path, sizeof(path));
    edit_via_temp(editor, path);
}
#endif

LOCAL_COMMAND_BLE("edit", "edit a device file in default editor", cmd_edit, ble_cmd_edit);
