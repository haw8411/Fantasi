#include "../cli.h"
#include "../../hal/hal.h"

static int cmd_device(int argc, char **argv)
{
    (void)argc; (void)argv;
    cli_printf("%s\r\n", hal_device_id());
    return 0;
}

CLI_COMMAND("device", "print device identifier", cmd_device);
