/* Generic HAL serial implementation on top of TinyUSB's CDC class.
 * Used by both Cortex-M platforms (Flipper, Chameleon). The platform
 * still owns hal_init() (clocks, USB PHY, tud_init) and the USB IRQ
 * handler; this file only provides read/write/connected + heap helpers. */

#include "../../hal/hal.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"

size_t hal_serial_write(const uint8_t *buf, size_t len)
{
    if (!tud_cdc_connected()) return 0;
    uint32_t wrote = tud_cdc_write(buf, len);
    /* Flush every call: we're driving a CLI, not bulk throughput. */
    tud_cdc_write_flush();
    return (size_t)wrote;
}

size_t hal_serial_read(uint8_t *buf, size_t len)
{
    if (!tud_cdc_available()) return 0;
    return (size_t)tud_cdc_read(buf, len);
}

/* ---- RX-event wait (overrides the poll-fallback weak in core/cli.c) ----
 * The CLI task blocks here between commands instead of polling every 5 ms;
 * tud_cdc_rx_cb (below, USB-task context) wakes it. Single waiter: the USB
 * CDC serial has exactly one reader, the CLI task. */
static volatile TaskHandle_t s_cdc_waiter;

void hal_serial_wait(uint32_t timeout_ms)
{
    s_cdc_waiter = xTaskGetCurrentTaskHandle();
    /* Re-check after registering: data that raced in between the caller's
     * read() and here would otherwise sleep a full timeout. */
    if (!tud_cdc_available())
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    s_cdc_waiter = NULL;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    TaskHandle_t t = s_cdc_waiter;
    if (t) xTaskNotifyGive(t);
}

bool hal_serial_connected(void)
{
    return tud_cdc_connected();
}

size_t hal_free_heap_bytes(void)
{
    return (size_t)xPortGetFreeHeapSize();
}

size_t hal_min_ever_free_heap_bytes(void)
{
    return (size_t)xPortGetMinimumEverFreeHeapSize();
}

/* Set by hal_usb_reenumerate() when a descriptor change (switch-mode HID arming,
 * or an hid persistent/switch mode change) needs the host to re-read the config.
 * The USB task performs the actual bus bounce. */
static volatile bool s_reenum_req;

void hal_usb_reenumerate(void)
{
    s_reenum_req = true;
}

/* The USB device task that core/main.c creates via the weak symbol. */
void platform_usb_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* Block on TinyUSB's internal event queue (CFG_TUSB_OS=OPT_OS_FREERTOS;
         * tud_int_handler wakes it) so the CPU idles between events. The 100 ms
         * timeout only bounds the reenum-flag fallback below. */
        tud_task_ext(100, false);
        if (s_reenum_req) {
            s_reenum_req = false;
            tud_disconnect();
            vTaskDelay(pdMS_TO_TICKS(300));
            tud_connect();
        }
    }
}
