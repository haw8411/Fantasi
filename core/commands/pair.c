#include "../cli.h"
#include "../../hal/hal.h"
#include "ble_common.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stdlib.h>

static uint32_t read_passkey_from_serial(void)
{
    char buf[8];
    int pos = 0;

    cli_write("enter passkey: ");

    cli_ctx_t *ctx = cli_current_ctx();
    for (;;) {
        uint8_t ch;
        if (!ctx || ctx->transport.read(&ch, 1, ctx->transport.ctx) == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            cli_write("\r\n");
            break;
        }
        if (ch == 0x7F || ch == '\b') {
            if (pos > 0) {
                pos--;
                cli_write("\b \b");
            }
            continue;
        }
        if (ch >= '0' && ch <= '9' && pos < 6) {
            buf[pos++] = (char)ch;
            char echo[2] = {(char)ch, 0};
            cli_write(echo);
        }
    }
    buf[pos] = '\0';
    return (uint32_t)strtoul(buf, NULL, 10);
}

static int cmd_pair(int argc, char **argv)
{
    int rc = 1;

    if (argc >= 2) {
        /* Central mode: pair <MAC> */
        uint8_t addr[6];
        if (parse_mac(argv[1], addr) != 0) {
            cli_write("usage: pair AA:BB:CC:DD:EE:FF\r\n");
            return 1;
        }

        uint8_t addr_type = 0x00;
        if (argc >= 3 && argv[2][0] == 'r')
            addr_type = 0x01;

        if (hal_ble_pair_setup(HAL_BLE_IO_KEYBOARD_ONLY) != 0) {
            cli_write("BLE not available\r\n");
            return 1;
        }
        hal_ble_pair_begin();

        cli_printf("connecting to %02X:%02X:%02X:%02X:%02X:%02X...\r\n",
                   addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

        if (hal_ble_pair_connect(addr, addr_type) != 0) {
            cli_write("connect failed\r\n");
            goto done;
        }

        hal_ble_evt_t evt;
        if (hal_ble_pair_wait(&evt, 15000) != 0 ||
            evt.type != HAL_BLE_EVT_CONNECTED) {
            cli_write("connection timeout\r\n");
            goto done;
        }

        if (evt.status != 0) {
            cli_printf("connection failed (status 0x%02X)\r\n", evt.status);
            goto done;
        }

        uint16_t handle = evt.conn_handle;
        cli_printf("connected (handle 0x%04X)\r\n", handle);

        cli_write("initiating pairing...\r\n");
        if (hal_ble_pair_initiate(handle) != 0) {
            cli_write("pairing request failed\r\n");
            hal_ble_disconnect(handle);
            goto done;
        }

        for (;;) {
            if (hal_ble_pair_wait(&evt, 30000) != 0) {
                cli_write("pairing timeout\r\n");
                hal_ble_disconnect(handle);
                goto done;
            }

            if (evt.type == HAL_BLE_EVT_PASSKEY_REQUEST) {
                uint32_t pin = read_passkey_from_serial();
                hal_ble_pair_passkey(handle, pin);
                continue;
            }

            if (evt.type == HAL_BLE_EVT_PASSKEY_DISPLAY) {
                cli_printf("confirm: %06lu\r\n", (unsigned long)evt.passkey);
                hal_ble_pair_confirm(handle, true);
                continue;
            }

            if (evt.type == HAL_BLE_EVT_PAIR_COMPLETE) {
                if (evt.status == 0) {
                    cli_write("paired - connection active\r\n");
                    rc = 0;
                } else {
                    cli_printf("pairing failed (status 0x%02X reason 0x%02X)\r\n",
                               evt.status, evt.reason);
                    hal_ble_disconnect(handle);
                }
                goto done;
            }

            if (evt.type == HAL_BLE_EVT_DISCONNECTED) {
                cli_write("disconnected during pairing\r\n");
                goto done;
            }
        }

    } else {
        /* Peripheral mode: wait for an incoming pairing request.
         * Background advertising is already active from ble_init(). */
        if (hal_ble_pair_setup(HAL_BLE_IO_KEYBOARD_ONLY) != 0) {
            cli_write("BLE not available\r\n");
            return 1;
        }
        hal_ble_pair_begin();

        hal_ble_evt_t evt;
        cli_write("waiting for pairing request (60s)...\r\n");

        if (hal_ble_pair_wait(&evt, 60000) != 0 ||
            evt.type != HAL_BLE_EVT_CONNECTED) {
            cli_write("no connection received\r\n");
            goto done;
        }

        if (evt.status != 0) {
            cli_printf("connection failed (status 0x%02X)\r\n", evt.status);
            goto done;
        }

        uint16_t handle = evt.conn_handle;
        cli_printf("connected (handle 0x%04X)\r\n", handle);

        for (;;) {
            if (hal_ble_pair_wait(&evt, 30000) != 0) {
                cli_write("pairing timeout\r\n");
                hal_ble_disconnect(handle);
                goto done;
            }

            if (evt.type == HAL_BLE_EVT_PASSKEY_REQUEST) {
                uint32_t pin = read_passkey_from_serial();
                hal_ble_pair_passkey(handle, pin);
                continue;
            }

            if (evt.type == HAL_BLE_EVT_PASSKEY_DISPLAY) {
                cli_printf("confirm: %06lu\r\n", (unsigned long)evt.passkey);
                hal_ble_pair_confirm(handle, true);
                continue;
            }

            if (evt.type == HAL_BLE_EVT_PAIR_COMPLETE) {
                if (evt.status == 0) {
                    cli_write("paired - connection active\r\n");
                    rc = 0;
                } else {
                    cli_printf("pairing failed (status 0x%02X reason 0x%02X)\r\n",
                               evt.status, evt.reason);
                    hal_ble_disconnect(handle);
                }
                goto done;
            }

            if (evt.type == HAL_BLE_EVT_DISCONNECTED) {
                cli_write("disconnected during pairing\r\n");
                goto done;
            }
        }
    }

done:
    hal_ble_pair_end();
    return rc;
}

CLI_COMMAND("pair", "pair with a BLE device", cmd_pair);
