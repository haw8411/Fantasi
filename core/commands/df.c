#include "../cli.h"
#include "../../hal/hal.h"

#include <stdint.h>

static int cmd_df(int argc, char **argv)
{
    (void)argc; (void)argv;
    int32_t free = hal_flash_free_bytes();
    if (free < 0)
        cli_write("flash free: unavailable\r\n");
    else
        cli_printf("flash free: %ld bytes\r\n", (long)free);
    return 0;
}

CLI_COMMAND("df", "show free flash bytes", cmd_df);
