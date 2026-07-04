#include "../cli.h"
#include "../../hal/hal.h"

static int cmd_radio(int argc, char **argv)
{
    (void)argc; (void)argv;
    hal_radio_info_t ri;
    hal_radio_info(&ri);
    if (!ri.available) {
        cli_write("no radio hardware on this platform\r\n");
        return 0;
    }
    cli_printf("secure flash: %lu KB @ 0x%08lX\r\n",
               (unsigned long)ri.secure_flash_kb,
               (unsigned long)ri.secure_flash_start);
    if (ri.fus_major || ri.fus_minor || ri.fus_sub)
        cli_printf("FUS:   %lu.%lu.%lu\r\n",
                   (unsigned long)ri.fus_major,
                   (unsigned long)ri.fus_minor,
                   (unsigned long)ri.fus_sub);
    if (ri.ws_major || ri.ws_minor || ri.ws_sub)
        cli_printf("stack: %lu.%lu.%lu (type 0x%02lX)\r\n",
                   (unsigned long)ri.ws_major,
                   (unsigned long)ri.ws_minor,
                   (unsigned long)ri.ws_sub,
                   (unsigned long)ri.ws_type);
    cli_printf("BLE:   %s\r\n", hal_ble_is_active() ? "on" : "off");
    return 0;
}

CLI_COMMAND("radio", "show radio stack info", cmd_radio);
