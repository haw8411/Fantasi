#include "../cli.h"
#include "../../hal/hal.h"
#include "ble_common.h"

#include <stdint.h>

static int cmd_connect(int argc, char **argv)
{
    if (argc < 2) {
        cli_write("usage: connect AA:BB:CC:DD:EE:FF\r\n");
        return 1;
    }

    uint8_t addr[6];
    if (parse_mac(argv[1], addr) != 0) {
        cli_write("usage: connect AA:BB:CC:DD:EE:FF\r\n");
        return 1;
    }

    uint8_t addr_type = 0x00;
    if (argc >= 3 && argv[2][0] == 'r')
        addr_type = 0x01;

    if (hal_ble_pair_setup(HAL_BLE_IO_DISPLAY_ONLY) != 0) {
        cli_write("BLE not available\r\n");
        return 1;
    }

    cli_printf("connecting to %02X:%02X:%02X:%02X:%02X:%02X...\r\n",
               addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    cli_flush();

    if (hal_ble_pair_connect(addr, addr_type) != 0) {
        cli_write("connect failed\r\n");
        return 1;
    }

    hal_ble_evt_t evt;
    if (hal_ble_pair_wait(&evt, 15000) != 0 ||
        evt.type != HAL_BLE_EVT_CONNECTED) {
        cli_write("connection timeout\r\n");
        return 1;
    }

    if (evt.status != 0) {
        cli_printf("connection failed (status 0x%02X)\r\n", evt.status);
        return 1;
    }

    cli_printf("connected (handle 0x%04X)\r\n", evt.conn_handle);
    return 0;
}

CLI_COMMAND("connect", "connect to a BLE peer", cmd_connect);
