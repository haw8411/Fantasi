#ifndef FANTASI_BLE_SERIAL_H
#define FANTASI_BLE_SERIAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

int  ble_serial_init(void);

size_t ble_serial_write(const uint8_t *buf, size_t len, void *ctx);
size_t ble_serial_read(uint8_t *buf, size_t len, void *ctx);
bool   ble_serial_connected(void *ctx);
void   ble_serial_poll(void);

void ble_serial_on_attr_modified(uint16_t conn_handle, uint16_t handle,
                                 const uint8_t *data, uint16_t len);
void ble_serial_on_tx_complete(void);
void ble_serial_on_connect(uint16_t conn_handle);
void ble_serial_on_disconnect(uint16_t conn_handle);
void ble_serial_on_notification(const uint8_t *data, uint16_t len);
void ble_serial_set_pair_pending(uint16_t conn_handle, uint32_t passkey);

#endif
