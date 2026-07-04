#include "../cli.h"
#include "../../hal/hal.h"

#include <stdint.h>
#include <stdlib.h>

static void scan_cb(const uint8_t *addr, uint8_t addr_type, int8_t rssi,
                    const char *name)
{
    (void)addr_type;
    cli_printf("%02X:%02X:%02X:%02X:%02X:%02X  %d dBm",
               addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
               (int)rssi);
    if (name && name[0])
        cli_printf("  \"%s\"", name);
    cli_write("\r\n");
}

static int cmd_scan(int argc, char **argv)
{
    uint32_t dur = 5000;
    if (argc >= 2) {
        unsigned long v = strtoul(argv[1], NULL, 10);
        if (v >= 1 && v <= 60) dur = (uint32_t)v * 1000;
    }

    cli_printf("scanning for %lu s...\r\n", (unsigned long)(dur / 1000));

    int n = hal_ble_scan(scan_cb, dur);
    if (n < 0) {
        cli_write("BLE not available on this platform\r\n");
        return 1;
    }

    cli_printf("scan complete: %d device%s\r\n", n, n == 1 ? "" : "s");
    return 0;
}

CLI_COMMAND("scan", "scan for BLE devices", cmd_scan);
