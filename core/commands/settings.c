#include "../cli.h"
#include "../../hal/hal.h"

static int cmd_settings(int argc, char **argv)
{
    (void)argc; (void)argv;

    char buf[256];
    int n = hal_settings_dump(buf, sizeof(buf));
    if (n <= 0) {
        cli_write("no settings\r\n");
        return 0;
    }
    char *p = buf;
    while (*p) {
        char *line = p;
        while (*p && *p != '\n') p++;
        if (*p == '\n') *p++ = '\0';
        if (line[0])
            cli_printf("%s\r\n", line);
    }
    return 0;
}

CLI_COMMAND("settings", "show saved settings", cmd_settings);
