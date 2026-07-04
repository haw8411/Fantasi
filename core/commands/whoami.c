#include "../cli.h"
#include "../../hal/hal.h"

static int cmd_whoami(int argc, char **argv)
{
    (void)argc; (void)argv;
    cli_printf("%s\r\n", hal_device_name());
    return 0;
}

CLI_COMMAND("whoami", "show unique device name", cmd_whoami);
