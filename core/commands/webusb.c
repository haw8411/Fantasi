#include "../cli.h"
#include "../../hal/hal.h"
#include "cmd_util.h"

/* Switch the USB personality to the vendor/WebUSB protobuf interface. On the
 * composite targets (FZ/CU) that interface is always present, so this is a no-op
 * that just says so. On switch-mode targets it tears down CDC and re-enumerates
 * as a vendor-only device; `cdc` switches back. */
static int cmd_webusb(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (hal_enter_webusb_mode() == -1) {
        cli_write("WebUSB protobuf interface always available (composite)\r\n");
        return 0;
    }
    cli_write("entering WebUSB mode - send 'cdc' to return to serial\r\n");
    flush_before_reset();   /* let the reply drain before the USB task re-enumerates */
    return 0;
}

CLI_COMMAND("webusb", "switch USB to WebUSB (protobuf) mode", cmd_webusb);
