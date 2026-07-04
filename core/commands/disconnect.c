#include "../cli.h"
#include "../../hal/hal.h"

#include <stdint.h>
#include <stdlib.h>

static int cmd_disconnect(int argc, char **argv)
{
    hal_ble_conn_info_t conns[2];
    int n = hal_ble_connections(conns, 2);
    if (n == 0) {
        cli_write("no active connections\r\n");
        return 0;
    }

    if (argc >= 2) {
        uint16_t h = (uint16_t)strtoul(argv[1], NULL, 16);
        if (hal_ble_disconnect(h) == 0)
            cli_printf("disconnected 0x%04X\r\n", h);
        else
            cli_write("disconnect failed\r\n");
    } else {
        for (int i = 0; i < n; i++) {
            hal_ble_disconnect(conns[i].handle);
            cli_printf("disconnected 0x%04X\r\n", conns[i].handle);
        }
    }
    return 0;
}

CLI_COMMAND("disconnect", "disconnect BLE peer(s)", cmd_disconnect);
