#include "../cli.h"
#include "../../hal/hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

static bool addr_eq(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 6; i++)
        if (a[i] != b[i]) return false;
    return true;
}

static void ble_list(void)
{
    if (!hal_ble_is_active()) {
        cli_write("ble is off\r\n");
        return;
    }

    hal_ble_bonded_t bonded[8];
    int nb = hal_ble_get_bonded(bonded, 8);

    hal_ble_conn_info_t conns[2];
    int nc = hal_ble_connections(conns, 2);

    if (nb == 0 && nc == 0) {
        cli_write("no paired or connected devices\r\n");
        return;
    }

    for (int i = 0; i < nb; i++) {
        const uint8_t *a = bonded[i].addr;
        cli_printf("  %02X:%02X:%02X:%02X:%02X:%02X  paired",
                   a[5], a[4], a[3], a[2], a[1], a[0]);
        for (int j = 0; j < nc; j++) {
            if (addr_eq(bonded[i].addr, conns[j].addr)) {
                cli_printf(", connected (0x%04X)", conns[j].handle);
                break;
            }
        }
        cli_write("\r\n");
    }

    for (int j = 0; j < nc; j++) {
        bool found = false;
        for (int i = 0; i < nb; i++) {
            if (addr_eq(conns[j].addr, bonded[i].addr)) {
                found = true;
                break;
            }
        }
        if (!found) {
            const uint8_t *a = conns[j].addr;
            cli_printf("  %02X:%02X:%02X:%02X:%02X:%02X  connected (0x%04X)\r\n",
                       a[5], a[4], a[3], a[2], a[1], a[0], conns[j].handle);
        }
    }
}

static void ble_set_enabled(bool on)
{
    if (on) {
        if (!hal_ble_is_active()) {
            if (hal_ble_pair_setup(HAL_BLE_IO_DISPLAY_ONLY) != 0) {
                cli_write("ble on failed\r\n");
                return;
            }
            cli_write("ble on\r\n");
        } else {
            cli_write("ble already on\r\n");
        }
    } else {
        if (hal_ble_is_active())
            hal_ble_shutdown();
        cli_write("ble off\r\n");
    }

    if (hal_settings_set("ble", on ? "1" : "0") != 0)
        cli_write("warning: config save failed\r\n");
}

static int cmd_ble(int argc, char **argv)
{
    if (argc >= 2) {
        if (strcmp(argv[1], "on") == 0) {
            ble_set_enabled(true);
        } else if (strcmp(argv[1], "off") == 0) {
            ble_set_enabled(false);
        } else if (strcmp(argv[1], "list") == 0) {
            ble_list();
        } else {
            cli_write("usage: ble [list|on|off]\r\n");
        }
    } else {
        ble_list();
    }
    return 0;
}

CLI_COMMAND("ble", "BLE status [list|on|off]", cmd_ble);
