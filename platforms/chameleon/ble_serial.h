#ifndef FANTASI_BLE_SERIAL_H
#define FANTASI_BLE_SERIAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "../../hal/hal.h"

int    ble_serial_init(void);
size_t ble_serial_write(const uint8_t *buf, size_t len, void *ctx);
size_t ble_serial_read(uint8_t *buf, size_t len, void *ctx);
bool   ble_serial_connected(void *ctx);
void   ble_serial_poll(void);

/* Serial service (advertising) on/off - backs `ble on` / `ble off`. */
int    ble_serial_resume(void);
void   ble_serial_stop(void);
bool   ble_serial_is_active(void);

/* Central-mode pairing */
int  ble_pair_setup_security(uint8_t io_cap);
void ble_pair_set_manual(bool on);
int  ble_pair_connect(const uint8_t *addr, uint8_t addr_type);
int  ble_pair_initiate(uint16_t conn_handle);
int  ble_pair_send_passkey(uint16_t conn_handle, uint32_t passkey);
int  ble_pair_numeric_confirm(uint16_t conn_handle, bool accept);
int  ble_pair_wait_event(hal_ble_evt_t *evt, uint32_t timeout_ms);
int  ble_pair_disconnect(uint16_t conn_handle);

#endif
