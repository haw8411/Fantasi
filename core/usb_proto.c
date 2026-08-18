/* USB vendor-interface (WebUSB) protobuf transport. Carries the same protobuf
 * request/response protocol as BLE - the whole CLI plus file ops - over a
 * dedicated bulk pipe, so structured/bulk data never pollutes the text CLI.
 * Drives the shared engine in proto.c (fantasi_proto_rx). */
#include "usb_proto.h"
#include "cli.h"
#include "proto.h"   /* fantasi_proto_rx / fantasi_proto_init */

#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"

#define VPROTO_ACCUM  1024

/* ---- cli_transport over the TinyUSB vendor class ----
 * tud_task() (in platform_usb_task) services the endpoints; this transport just
 * moves bytes to/from the vendor FIFOs, so poll is NULL. */

static size_t vendor_read(uint8_t *buf, size_t len, void *c)
{
    (void)c;
    if (!tud_vendor_mounted() || !tud_vendor_available()) return 0;
    return (size_t)tud_vendor_read(buf, len);
}

static size_t vendor_write(const uint8_t *buf, size_t len, void *c)
{
    (void)c;
    if (!tud_vendor_mounted()) return 0;
    uint32_t w = tud_vendor_write(buf, len);
    tud_vendor_write_flush();
    return (size_t)w;
}

static bool vendor_connected(void *c) { (void)c; return tud_vendor_mounted(); }

static void vendor_flush(void) { tud_vendor_write_flush(); }

/* RX-event wait: block until the vendor RX callback signals new data (or
 * timeout), so the task - and the CPU - can idle between requests. Single
 * waiter: usb_proto_task. */
static volatile TaskHandle_t s_vendor_waiter;

static void vendor_wait(uint32_t timeout_ms)
{
    s_vendor_waiter = xTaskGetCurrentTaskHandle();
    if (!tud_vendor_available())   /* re-check: data may have raced in */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    s_vendor_waiter = NULL;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf; (void)buffer; (void)bufsize;
    TaskHandle_t t = s_vendor_waiter;
    if (t) xTaskNotifyGive(t);
}

/* Framed-response sink: push the whole buffer through the bulk FIFO, draining as
 * needed so a response larger than the FIFO can't be truncated. */
static size_t vendor_emit(const uint8_t *buf, size_t len)
{
    size_t sent = 0;
    /* Tick-deadline, not a retry counter: tickless idle can make one
     * vTaskDelay span many ticks. ~250 ms of no progress = peer gone. */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(250);
    while (sent < len) {
        if (!tud_vendor_mounted()) break;
        uint32_t w = tud_vendor_write(buf + sent, len - sent);
        if (w == 0) {
            tud_vendor_write_flush();
            if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) break;   /* peer gone: drop rest rather than hang */
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        deadline = xTaskGetTickCount() + pdMS_TO_TICKS(250);
        sent += w;
    }
    tud_vendor_write_flush();
    return sent;
}

void usb_proto_task(void *arg)
{
    (void)arg;
    fantasi_proto_init();

    static cli_ctx_t ctx;
    ctx.transport.write     = vendor_write;
    ctx.transport.read      = vendor_read;
    ctx.transport.connected = vendor_connected;
    ctx.transport.poll      = NULL;          /* tud_task() runs in platform_usb_task */
    ctx.transport.flush     = vendor_flush;
    ctx.transport.wait      = vendor_wait;
    ctx.transport.ctx       = NULL;

    static uint8_t accum[VPROTO_ACCUM];
    static size_t  accum_len;
    accum_len = 0;

    for (;;) {
        uint8_t tmp[256];
        size_t n = vendor_read(tmp, sizeof(tmp), NULL);
        if (n == 0) {
            /* Block until the vendor RX callback signals data (timeout only
             * bounds a missed wakeup). */
            vendor_wait(100);
            continue;
        }
        fantasi_proto_rx(&ctx, accum, VPROTO_ACCUM, &accum_len, vendor_emit, tmp, n);
    }
}
