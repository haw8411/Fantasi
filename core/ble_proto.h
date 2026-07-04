#ifndef FANTASI_BLE_PROTO_H
#define FANTASI_BLE_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include "cli.h"

void ble_proto_task(void *arg);
void ble_proto_set_mtu(uint16_t att_mtu);

/* ---- Transport-agnostic protobuf engine ----
 * The frame-parse + dispatch + handlers are shared by every protobuf transport
 * (BLE, USB vendor/WebUSB). Each transport supplies its own RX accumulator and a
 * framed-response `emit` sink; a mutex inside serialises the shared dispatch
 * state across transports. Call fantasi_proto_init() once before any proto task. */
void fantasi_proto_init(void);
void fantasi_proto_rx(cli_ctx_t *ctx, uint8_t *accum, size_t cap, size_t *accum_len,
                      size_t (*emit)(const uint8_t *buf, size_t len),
                      const uint8_t *in, size_t n);

#endif
