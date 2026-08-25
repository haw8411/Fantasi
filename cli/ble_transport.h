#ifndef FANTASI_BLE_TRANSPORT_H
#define FANTASI_BLE_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

void    ble_transport_set_addr(const char *addr);

/* Restrict device discovery to the Fantasi device with this name (the bare
 * hal_device_name() token). Advertised as "Fantasi <name>", so the
 * "Fantasi " prefix is matched automatically. NULL/empty = any. */
void    ble_transport_set_name(const char *name);
int     ble_transport_open(void);
bool    ble_transport_reconnect(void);
void    ble_transport_close(void);
bool    ble_transport_connected(void);

ssize_t ble_transport_read(void *buf, size_t len);
ssize_t ble_transport_write(const void *buf, size_t len);
/* Send one protobuf message through the per-session BLE fragment envelope.
 * Each fragment is one GATT write, so BlueZ may serialize writes from unrelated
 * processes in any order without combining their protobuf streams. */
ssize_t ble_transport_write_session(uint32_t session,
                                    const void *message, size_t len);
/* Fast path for idempotent requests (currently absolute-offset file chunks).
 * ATT Write Commands avoid paying one radio round-trip per mux fragment; the
 * request's protobuf response remains the end-to-end acknowledgement. */
ssize_t ble_transport_write_session_command(uint32_t session,
                                            const void *message, size_t len);
/* After an in-band OPEN completes, retain only this logical session's response
 * envelopes. Zero accepts every session while OPEN is still being negotiated
 * (and preserves the un-enveloped legacy byte stream). */
void    ble_transport_set_response_session(uint32_t session);

int     ble_transport_fd(void);
void    ble_transport_process(void);

#endif
