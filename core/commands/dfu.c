#include "../cli.h"
#include "../../hal/hal.h"
#include "cmd_util.h"

#include <string.h>

static int cmd_dfu(int argc, char **argv)
{
    bool fus = (argc >= 2 && strcmp(argv[1], "radio") == 0);
    cli_write(fus ? "activating FUS + DFU...\r\n" : "entering DFU...\r\n");
    flush_before_reset();
    if (fus) {
        /* Set DFU magic BEFORE FUS activation - the second SHCI
         * GET_STATE may reset the device before we get control back. */
        hal_set_dfu_magic();
        hal_ble_activate_fus();
    }
    hal_reboot_dfu();
    return 0;
}

CLI_COMMAND("dfu", "reboot into bootloader/DFU", cmd_dfu);
