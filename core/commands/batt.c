#include "../cli.h"
#include "../../hal/hal.h"

static int cmd_batt(int argc, char **argv)
{
    (void)argc; (void)argv;
    int pct = hal_battery_percent();
    if (pct < 0)
        cli_write("Battery level: Unavailable\r\n");
    else
        cli_printf("Battery level: %d%%\r\n", pct);
    return 0;
}

CLI_COMMAND("batt", "show battery level", cmd_batt);
