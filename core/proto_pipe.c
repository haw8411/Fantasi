#include "proto_pipe.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

void proto_pipe_init(proto_pipe_t *p, proto_pipe_write_fn wfn,
                   proto_pipe_poll_fn pfn, void *ctx, uint16_t mtu_payload)
{
    memset(p, 0, sizeof(*p));
    p->write_fn    = wfn;
    p->poll_fn     = pfn;
    p->write_ctx   = ctx;
    p->mtu_payload = mtu_payload ? mtu_payload : 20;
}

void proto_pipe_set_mtu(proto_pipe_t *p, uint16_t att_mtu)
{
    uint16_t pl = att_mtu - 3;
    if (pl < 20) pl = 20;
    if (pl > 244) pl = 244;
    p->mtu_payload = pl;
}

static void drain_one(proto_pipe_t *p, const uint8_t *data, uint16_t len)
{
    /* ~200 ms tick-deadline. */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(200);
    for (;;) {
        size_t w = p->write_fn(data, len, p->write_ctx);
        if (w > 0) return;
        if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) return;
        if (p->poll_fn) p->poll_fn();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void drain_full(proto_pipe_t *p)
{
    while (p->len >= p->mtu_payload) {
        drain_one(p, p->buf, p->mtu_payload);
        p->len -= p->mtu_payload;
        if (p->len > 0)
            memmove(p->buf, &p->buf[p->mtu_payload], p->len);
    }
}

size_t proto_pipe_write(const uint8_t *buf, size_t len, void *pipe_ptr)
{
    proto_pipe_t *p = (proto_pipe_t *)pipe_ptr;
    const uint8_t *src = buf;
    size_t remaining = len;

    while (remaining > 0) {
        uint16_t space = PROTO_PIPE_BUF_SIZE - p->len;
        uint16_t copy = (remaining > space) ? space : (uint16_t)remaining;
        memcpy(&p->buf[p->len], src, copy);
        p->len += copy;
        src += copy;
        remaining -= copy;
        drain_full(p);
    }
    return len;
}

void proto_pipe_flush(proto_pipe_t *p)
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
