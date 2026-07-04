#ifndef FANTASI_BLE_TRANSPORT_H
#define FANTASI_BLE_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

void    ble_transport_set_addr(const char *addr);
int     ble_transport_open(void);
bool    ble_transport_reconnect(void);
void    ble_transport_close(void);
bool    ble_transport_connected(void);

ssize_t ble_transport_read(void *buf, size_t len);
ssize_t ble_transport_write(const void *buf, size_t len);

int     ble_transport_fd(void);
void    ble_transport_process(void);

#endif
