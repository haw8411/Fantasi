#include "../cli.h"
#include "../../hal/hal.h"
#include <string.h>

static void settings_emit(const char *line, void *ctx)
{
    (*(int *)ctx)++;
    cli_printf("%s\r\n", line);
}

static void settings_dump(void)
{
    int count = 0;
    hal_settings_foreach(settings_emit, &count);
    if (count == 0)
        cli_write("no settings\r\n");
}

/* settings                 - list all saved settings
 * settings get <key>       - print one value (or "<key>: not set")
 * settings set <key> <val> - write/replace a key (value is a single token)
 * settings unset <key>     - remove a key */
static int cmd_settings(int argc, char **argv)
{
    if (argc >= 3 && strcmp(argv[1], "get") == 0) {
        char v[160];
        int n = hal_settings_get(argv[2], v, sizeof(v));
        if (n < 0) { cli_printf("%s: not set\r\n", argv[2]); return 1; }
        cli_printf("%s\r\n", v);
        return 0;
    }
    if (argc >= 4 && strcmp(argv[1], "set") == 0) {
        if (hal_settings_set(argv[2], argv[3]) != 0) { cli_write("set failed\r\n"); return 1; }
        cli_printf("%s=%s\r\n", argv[2], argv[3]);
        return 0;
    }
    if (argc >= 3 && strcmp(argv[1], "unset") == 0) {
        if (hal_settings_unset(argv[2]) != 0) { cli_write("unset failed\r\n"); return 1; }
        cli_printf("unset %s\r\n", argv[2]);
        return 0;
    }

    settings_dump();
    return 0;
}

CLI_COMMAND("settings", "manage saved settings", cmd_settings);
