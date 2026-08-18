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

/* cli_transport_t.wait - blocks until the IPCC RX ISR signals a BLE event
 * (ble_serial_wake_from_isr) or timeout_ms elapses. May return early. */
void   ble_serial_wait(uint32_t timeout_ms);

/* Called from ble.c's IPCC RX handler (ISR context) after event processing. */
#ifndef __ASSEMBLER__
#include "FreeRTOS.h"
void   ble_serial_wake_from_isr(BaseType_t *woken);
#endif

void ble_serial_on_attr_modified(uint16_t conn_handle, uint16_t handle,
                                 const uint8_t *data, uint16_t len);
void ble_serial_on_tx_complete(void);
void ble_serial_on_connect(uint16_t conn_handle);
void ble_serial_on_disconnect(uint16_t conn_handle);
void ble_serial_on_notification(const uint8_t *data, uint16_t len);
void ble_serial_set_pair_pending(uint16_t conn_handle, uint32_t passkey);

#endif
