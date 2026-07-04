#include "../cli.h"
#include "../../hal/hal.h"
#include "cmd_util.h"

static int cmd_reboot(int argc, char **argv)
{
    (void)argc; (void)argv;
    cli_write("rebooting...\r\n");
    flush_before_reset();
    hal_reboot();
    return 0;
}

CLI_COMMAND("reboot", "restart the device", cmd_reboot);
