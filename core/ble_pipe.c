#include "ble_pipe.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

void ble_pipe_init(ble_pipe_t *p, ble_pipe_write_fn wfn,
                   ble_pipe_poll_fn pfn, void *ctx, uint16_t mtu_payload)
{
    memset(p, 0, sizeof(*p));
    p->write_fn    = wfn;
    p->poll_fn     = pfn;
    p->write_ctx   = ctx;
    p->mtu_payload = mtu_payload ? mtu_payload : 20;
}

void ble_pipe_set_mtu(ble_pipe_t *p, uint16_t att_mtu)
{
    uint16_t pl = att_mtu - 3;
    if (pl < 20) pl = 20;
    if (pl > 244) pl = 244;
    p->mtu_payload = pl;
}

static void drain_one(ble_pipe_t *p, const uint8_t *data, uint16_t len)
{
    for (int retries = 0; retries < 200; retries++) {
        size_t w = p->write_fn(data, len, p->write_ctx);
        if (w > 0) return;
        if (p->poll_fn) p->poll_fn();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void drain_full(ble_pipe_t *p)
{
    while (p->len >= p->mtu_payload) {
        drain_one(p, p->buf, p->mtu_payload);
        p->len -= p->mtu_payload;
        if (p->len > 0)
            memmove(p->buf, &p->buf[p->mtu_payload], p->len);
    }
}

size_t ble_pipe_write(const uint8_t *buf, size_t len, void *pipe_ptr)
{
    ble_pipe_t *p = (ble_pipe_t *)pipe_ptr;
    const uint8_t *src = buf;
    size_t remaining = len;

    while (remaining > 0) {
        uint16_t space = BLE_PIPE_BUF_SIZE - p->len;
        uint16_t copy = (remaining > space) ? space : (uint16_t)remaining;
        memcpy(&p->buf[p->len], src, copy);
        p->len += copy;
        src += copy;
        remaining -= copy;
        drain_full(p);
    }
    return len;
}

void ble_pipe_flush(ble_pipe_t *p)
{
    if (p->len == 0) return;
    if (p->len < p->mtu_payload) {
        memset(&p->buf[p->len], 0, p->mtu_payload - p->len);
        p->len = p->mtu_payload;
    }
    drain_full(p);
    if (p->len > 0)
        drain_one(p, p->buf, p->len);
    p->len = 0;
}
