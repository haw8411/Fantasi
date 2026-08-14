#include "cli.h"
#include "log.h"
#include "../hal/hal.h"

#ifdef FANTASI_ENABLE_BLE_CLI
#include "ble_serial.h"
#include "proto.h"
#endif

#ifdef FANTASI_ENABLE_WEBUSB
#include "proto.h"   /* fantasi_proto_init (shared protobuf engine) */
#include "usb_proto.h"
#endif

#include "../hal/storage/fat_ramdisk.h"   /* fatrd_store_init (storage lock) */

#include "FreeRTOS.h"
#include "task.h"

/* Platforms that need a dedicated USB device task (TinyUSB) define this to
 * point at their task entry. PM3 uses interrupts; it can leave this NULL. */
__attribute__((weak)) void platform_usb_task(void *arg) { (void)arg; vTaskSuspend(NULL); }

#ifndef CLI_TASK_STACK
#define CLI_TASK_STACK     (configMINIMAL_STACK_SIZE * 8)
#endif
/* TinyUSB's tud_task drains events and runs class drivers in the same task -
 * descriptor callbacks, cdc_device, usbd_control. Its real peak is tiny
 * (~72 words measured on PM3 via uxTaskGetSystemState). Default 8× for
 * platforms with roomy heaps; small-heap targets (PM3) override
 * USB_TASK_STACK down to their measured need + margin. */
#ifndef USB_TASK_STACK
#define USB_TASK_STACK     (configMINIMAL_STACK_SIZE * 8)
#endif

#ifdef FANTASI_ENABLE_BLE_CLI
static cli_ctx_t ble_cli_ctx;
#endif

#ifdef FANTASI_ENABLE_PWR_BUTTON
/* Long-hold power button. Platforms with a physical power button (CU: button
 * B; FZ: back button) implement hal_shutdown_button_held(); a continuous hold
 * of PWR_BUTTON_HOLD_MS powers the device off. PM3 has no such button and
 * leaves FANTASI_ENABLE_PWR_BUTTON unset, so none of this is built there. */
#ifndef PWR_BUTTON_HOLD_MS
#define PWR_BUTTON_HOLD_MS 1500
#endif
#define PWR_BUTTON_POLL_MS 50

static void pwr_button_task(void *arg)
{
    (void)arg;
    uint32_t held_ms = 0;
    bool fired = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PWR_BUTTON_POLL_MS));
        if (hal_shutdown_button_held()) {
            held_ms += PWR_BUTTON_POLL_MS;
            /* Fire once per hold. hal_shutdown() doesn't return when it powers
             * off; if it does return (e.g. USB-powered on FZ) the press is a
             * no-op - don't retry until the button is released and held again. */
            if (held_ms >= PWR_BUTTON_HOLD_MS && !fired) {
                fired = true;
                hal_shutdown();
            }
        } else {
            held_ms = 0;
            fired = false;
        }
    }
}
#endif

int main(void)
{
    hal_init();
    fantasi_log_init();
    fantasi_log(LOG_INFO, "boot");

    fatrd_store_init();   /* create the storage lock before the usb/cli/proto tasks touch LittleFS */

    xTaskCreate(cli_task, "cli", CLI_TASK_STACK, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(platform_usb_task, "usb", USB_TASK_STACK, NULL, tskIDLE_PRIORITY + 2, NULL);

#ifdef FANTASI_ENABLE_PWR_BUTTON
    xTaskCreate(pwr_button_task, "pwrbtn", configMINIMAL_STACK_SIZE,   /* shallow: poll + hal_shutdown (I2C PMIC regs only, no lfs/display) */
                NULL, tskIDLE_PRIORITY + 1, NULL);
#endif

#if defined(FANTASI_ENABLE_BLE_CLI) || defined(FANTASI_ENABLE_WEBUSB)
    fantasi_proto_init();   /* create the shared proto lock before any proto task runs */
#endif

#ifdef FANTASI_ENABLE_WEBUSB
    /* WebUSB protobuf transport. This and the BLE proto task below both default
     * to x4 (16 KB), sized for the Chameleon: there the LittleFS write path nests
     * SoftDevice flash SVCs (plus BLE event pumping, for blecli) and x2 tripped
     * the stack-overflow hook during uploads. Platforms without a SoftDevice run
     * that path shallow and sequential (the Flipper measured usbproto at 1312 B
     * and blecli at 1368 B under full transfers), so their Makefiles override these
     * stacks down e.g. with -DWEBUSB_PROTO_STACK=1024 and -DPROTO_STACK=1024. */
#ifndef WEBUSB_PROTO_STACK
#define WEBUSB_PROTO_STACK (CLI_TASK_STACK * 4)
#endif
    xTaskCreate(usb_proto_task, "usbproto", WEBUSB_PROTO_STACK,
                NULL, tskIDLE_PRIORITY + 1, NULL);
#endif

#ifdef FANTASI_ENABLE_BLE_CLI
    ble_cli_ctx.transport.write     = ble_serial_write;
    ble_cli_ctx.transport.read      = ble_serial_read;
    ble_cli_ctx.transport.connected = ble_serial_connected;
    ble_cli_ctx.transport.poll      = ble_serial_poll;
    ble_cli_ctx.transport.ctx       = NULL;
    /* Defaults to x4; see WEBUSB_PROTO_STACK above for the shared sizing rationale. */
#ifndef PROTO_STACK
#define PROTO_STACK (CLI_TASK_STACK * 4)
#endif
    xTaskCreate(proto_task, "blecli", PROTO_STACK,
                &ble_cli_ctx, tskIDLE_PRIORITY + 1, NULL);
#endif

    vTaskStartScheduler();

    /* Scheduler should never return. If it does, the heap was too small
     * or the idle/timer task couldn't be created. Halt rather than
     * continue with undefined state. */
    for (;;);
}

/* FreeRTOS hooks referenced by configASSERT / stack-check / heap-fail.
 * Reset on failure so the device self-recovers instead of hard-stalling
 * (a spin here needs a physical power-cycle). Platforms provide
 * fantasi_reset(); the weak fallback spins for platforms that don't. */
__attribute__((weak)) void fantasi_reset(void) { for (;;); }

/* A failed heap allocation is RECOVERABLE and must not brick the device: every
 * runtime allocator here checks the result (a ramfs/file upload that outgrows
 * the heap fails the write and reports an error to the host). Resetting here
 * would be worse than the OOM: on STM32WB a warm reset hangs CPU2 and drops USB.
 * Return instead; pvPortMalloc hands the caller NULL to handle. */
void vApplicationMallocFailedHook(void) { }

/* A stack overflow, unlike OOM, is unrecoverable memory corruption - reset. */
void vApplicationStackOverflowHook(TaskHandle_t t, char *n) { (void)t; (void)n; fantasi_reset(); }

