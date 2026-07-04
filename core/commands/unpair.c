#include "../cli.h"
#include "../../hal/hal.h"
#include "ble_common.h"

#include <stdint.h>

static int cmd_unpair(int argc, char **argv)
{
    if (argc >= 2) {
        uint8_t addr[6];
        if (parse_mac(argv[1], addr) != 0) {
            cli_write("usage: unpair [AA:BB:CC:DD:EE:FF]\r\n");
            return 1;
        }
        if (hal_ble_remove_bond(addr, 0x00) == 0) {
            cli_write("bond removed\r\n");
        } else {
            hal_ble_clear_bonds();
            cli_write("selective removal unsupported, all bonds cleared\r\n");
        }
    } else {
        hal_ble_clear_bonds();
        cli_write("all bonds cleared\r\n");
    }
    return 0;
}

CLI_COMMAND("unpair", "remove BLE bond(s)", cmd_unpair);
