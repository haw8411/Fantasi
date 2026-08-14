#include "../cli.h"
#include "../vfs.h"
#include "../../hal/hal.h"

#include <stdint.h>
#include <stdio.h>

/* Human-readable KiB count into `out` (K/M/G, one decimal for M/G). */
static void humansz(uint32_t kib, char *out, int n)
{
    if (kib >= (1u << 20)) {                  /* >= 1 GiB */
        uint32_t tenths = (uint32_t)(((uint64_t)kib * 10u) >> 20);
        snprintf(out, n, "%lu.%luG", (unsigned long)(tenths / 10u), (unsigned long)(tenths % 10u));
    } else if (kib >= (1u << 10)) {           /* >= 1 MiB */
        uint32_t tenths = (uint32_t)(((uint64_t)kib * 10u) >> 10);
        snprintf(out, n, "%lu.%luM", (unsigned long)(tenths / 10u), (unsigned long)(tenths % 10u));
    } else {
        snprintf(out, n, "%luK", (unsigned long)kib);
    }
}

static int cmd_df(int argc, char **argv)
{
    (void)argc; (void)argv;

    cli_printf("%-12s %8s %8s %8s\r\n", "Filesystem", "Total", "Used", "Free");
    int n = vfs_mount_count();
    for (int i = 0; i < n; i++) {
        const char *path = NULL;
        uint32_t total = 0, freeb = 0;
        bool is_ram = false;
        if (vfs_statfs(i, &path, &total, &freeb, &is_ram) != 0) continue;
        if (is_ram) {
            cli_printf("%-12s %8s %8s %8s\r\n", path, "RAM", "-", "-");
        } else {
            char t[12], u[12], f[12];
            humansz(total, t, sizeof t);
            humansz(total - freeb, u, sizeof u);
            humansz(freeb, f, sizeof f);
            cli_printf("%-12s %8s %8s %8s\r\n", path, t, u, f);
        }
    }

    /* Program (firmware) flash free is a separate metric from filesystem space. */
    int32_t pf = hal_flash_free_bytes();
    if (pf < 0) {
        cli_write("\r\nprogram flash free: unavailable\r\n");
    } else {
        char b[12];
        humansz((uint32_t)pf >> 10, b, sizeof b);   /* bytes -> KiB */
        cli_printf("\r\nprogram flash free: %s\r\n", b);
    }
    return 0;
}

CLI_COMMAND("df", "show filesystem usage per mount", cmd_df);
