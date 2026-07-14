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
        tud_task();
        if (s_reenum_req) {
            s_reenum_req = false;
            tud_disconnect();
            vTaskDelay(pdMS_TO_TICKS(300));
            tud_connect();
        }
        /* Yield so lower-priority tasks (the CLI) can run. tud_task()
         * is already event-driven internally. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
