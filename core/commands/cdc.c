#include "../cli.h"
#include "../../hal/hal.h"
#include "cmd_util.h"

/* Return the USB personality to the serial CLI (CDC). Meaningful on switch-mode
 * targets (PM3), where it's typically issued over the WebUSB protobuf channel to
 * hand control back to serial. Composite targets never leave CDC, so it's a
 * no-op there. */
static int cmd_cdc(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (hal_enter_cdc_mode() == -1) {
        cli_write("already on serial CDC (composite)\r\n");
        return 0;
    }
    cli_write("returning to serial (CDC)...\r\n");
    flush_before_reset();   /* let the reply drain before the USB task re-enumerates */
    return 0;
}

CLI_COMMAND("cdc", "return USB to serial (CDC) mode", cmd_cdc);
