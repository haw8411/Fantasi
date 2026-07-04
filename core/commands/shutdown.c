#include "../cli.h"
#include "../../hal/hal.h"
#include "cmd_util.h"

static int cmd_shutdown(int argc, char **argv)
{
    (void)argc; (void)argv;
    flush_before_reset();   /* drain the CDC TX ring before we may cut power */
    /* hal_shutdown() does not return when it powers off; it only returns to
     * report why it couldn't. (A shutdown over the USB CLI necessarily hits
     * the USB-powered case - power off from battery via BLE or the button.) */
    switch (hal_shutdown()) {
    case HAL_SHUTDOWN_USB_POWERED:
        cli_write("can't power off while USB-connected; unplug to shut down\r\n");
        break;
    default:
        cli_write("shutdown not supported on this platform\r\n");
        break;
    }
    return 0;
}

CLI_COMMAND("shutdown", "power the device off", cmd_shutdown);
