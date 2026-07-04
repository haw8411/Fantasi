#include "../cli.h"
#include "../../hal/hal.h"
#include "cmd_util.h"

static int cmd_msc(int argc, char **argv)
{
    (void)argc; (void)argv;
    int ret = hal_enter_msc_mode();
    if (ret == -1) {
        cli_write("MSC storage runs alongside CDC on this platform\r\n");
        return 0;
    }
    if (ret == -2) {
        cli_write("storage not available (S512 required)\r\n");
        return 1;
    }
    cli_write("entering MSC mode - eject to return to CLI\r\n");
    flush_before_reset();
    return 0;
}

CLI_COMMAND("msc", "enter USB mass storage mode", cmd_msc);
