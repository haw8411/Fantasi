#ifndef FANTASI_BLE_PIPE_H
#define FANTASI_BLE_PIPE_H

#include <stdint.h>
#include <stddef.h>

typedef size_t (*ble_pipe_write_fn)(const uint8_t *buf, size_t len, void *ctx);
typedef void   (*ble_pipe_poll_fn)(void);

#define BLE_PIPE_BUF_SIZE 512

typedef struct {
    uint8_t          buf[BLE_PIPE_BUF_SIZE];
    uint16_t         len;
    uint16_t         mtu_payload;
    ble_pipe_write_fn write_fn;
    ble_pipe_poll_fn  poll_fn;
    void             *write_ctx;
} ble_pipe_t;

void   ble_pipe_init(ble_pipe_t *p, ble_pipe_write_fn wfn,
                     ble_pipe_poll_fn pfn, void *ctx, uint16_t mtu_payload);
void   ble_pipe_set_mtu(ble_pipe_t *p, uint16_t att_mtu);
size_t ble_pipe_write(const uint8_t *buf, size_t len, void *pipe_ptr);
void   ble_pipe_flush(ble_pipe_t *p);

#endif
