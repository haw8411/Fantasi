/* `shortcut` - manage the 8 app launch shortcuts (slots 0-7).
 *
 * Each slot maps to an app path, persisted in settings.cfg as "scN=<path>".
 * The Flipper's Shortcuts menu and the screenless LED+button launchers (CU/PM3)
 * run whatever a slot points at. */
#ifdef FANTASI_ENABLE_APPS

#include "../cli.h"
#include "../app_run.h"
#include "../../hal/hal.h"

#include <string.h>

#define SC_SLOTS  8

/* settings key for slot n: "scN" */
static void sc_key(int n, char key[4])
{
    key[0] = 's'; key[1] = 'c'; key[2] = (char)('0' + n); key[3] = '\0';
}

/* Read slot n's app path into buf; returns length (0 if empty/unset). */
int shortcut_get(int slot, char *buf, int len)
{
    if (slot < 0 || slot >= SC_SLOTS) { if (len) buf[0] = '\0'; return 0; }
    char key[4];
    sc_key(slot, key);
    int r = hal_settings_get(key, buf, len);
    if (r <= 0) { buf[0] = '\0'; return 0; }
    return r;
}

/* Launch slot n's app. Returns the app exit code, or -1 if the slot is empty. */
int shortcut_run(int slot)
{
    char path[64];
    if (shortcut_get(slot, path, sizeof(path)) <= 0) return -1;
    return app_run(path);
}

static int cmd_shortcut(int argc, char **argv)
{
    if (argc < 2) {
        for (int i = 0; i < SC_SLOTS; i++) {
            char path[64];
            int r = shortcut_get(i, path, sizeof(path));
            cli_printf("  %d  %s\r\n", i, r > 0 ? path : "-");
        }
        cli_printf("usage: shortcut <0-7> [<app-path>|clear]\r\n");
        return 0;
    }

    if (argv[1][0] < '0' || argv[1][0] > '7' || argv[1][1] != '\0') {
        cli_printf("shortcut: slot must be 0-7\r\n");
        return 1;
    }
    int slot = argv[1][0] - '0';
    char key[4];
    sc_key(slot, key);

    if (argc == 2) {                       /* run the slot */
        char path[64];
        if (shortcut_get(slot, path, sizeof(path)) <= 0) {
            cli_printf("shortcut %d is empty\r\n", slot);
            return 1;
        }
        return app_run(path);
    }

    if (strcmp(argv[2], "clear") == 0) {   /* clear the slot */
        hal_settings_set(key, "");
        cli_printf("shortcut %d cleared\r\n", slot);
        return 0;
    }

    hal_settings_set(key, argv[2]);        /* assign a path */
    cli_printf("shortcut %d = %s\r\n", slot, argv[2]);
    return 0;
}

CLI_COMMAND("shortcut", "list/set/run app shortcuts (slots 0-7)", cmd_shortcut);

#endif /* FANTASI_ENABLE_APPS */
