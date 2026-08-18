/* nRF52840 internal flash driver for the LittleFS storage region.
 *
 * The 256 KB storage region is placed at the top of available app
 * flash, just below the DFU bootloader. The bootloader address is
 * read from UICR.NRFFW[0] at runtime.
 *
 * Layout (typical CU with bootloader at 0xF3000):
 *   0x00000000  MBR + SoftDevice (156 KB)
 *   0x00027000  firmware  (~24 KB, can grow to ~560 KB)
 *   0x000B3000  LittleFS  (256 KB = 64 × 4 KB pages)
 *   0x000F3000  DFU bootloader
 *
 * nRF52840 flash: 4 KB pages, 32-bit word programming via NVMC. */

#include "flash_storage.h"
#include "hal_storage.h"
#include "ble.h"
#include "power.h"
#include "nrf.h"
#include "../../hal/hal_power.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define UICR_BOOTLOADER_ADDR (*(volatile uint32_t *)0x10001014UL)

/* SoftDevice flash SVCs - async when SD is active, sync when not. */
#define SVC_SD_FLASH_PAGE_ERASE 0x28
#define SVC_SD_FLASH_WRITE      0x29
#define SVC_SD_EVT_GET          0x4B

SVCALL(SVC_SD_FLASH_PAGE_ERASE, uint32_t, svc_flash_page_erase(uint32_t page_number))
SVCALL(SVC_SD_FLASH_WRITE,      uint32_t, svc_flash_write(uint32_t *p_dst, const uint32_t *p_src, uint32_t size))
SVCALL(SVC_SD_EVT_GET,          uint32_t, svc_sd_evt_get(uint32_t *p_evt_id))

#define NRF_EVT_FLASH_OPERATION_SUCCESS 2
#define NRF_EVT_FLASH_OPERATION_ERROR   3

/* USB VBUS SoC events. Once the SoftDevice is enabled it owns the POWER
 * peripheral, so the app's POWER_CLOCK ISR no longer sees cable attach/detach -
 * those arrive here as SoC events instead (enabled via sd_power_usb*_enable in
 * cu_ble_sd_init). */
#define NRF_EVT_POWER_USB_POWER_READY   9
#define NRF_EVT_POWER_USB_DETECTED      10
#define NRF_EVT_POWER_USB_REMOVED       11

/* TinyUSB's nRF power-event hook (portable/nordic/nrf5x/dcd_nrf5x.c). Event
 * codes are fixed by the TinyUSB API: 0=detected, 1=removed, 2=ready. */
extern void tusb_hal_nrf_power_event(uint32_t event);
#define NRF_ERROR_BUSY_VAL              17   /* NRF_ERROR_BASE + 17 */

/* Keep draining BLE events while we block on a flash op: if the SoftDevice's
 * event queue fills (because the proto task is stuck here and not polling),
 * connection processing stalls and the radio never yields idle time for the
 * flash operation - a deadlock where the completion event never arrives.
 *
 * But this re-pump is only safe when we are already on the BLE proto task (a
 * flash op it started, e.g. a bond save - it re-enters its own poll). When a
 * flash op is started from another task - an MSC/settings write on the USB task
 * - pumping ble_serial_poll() here would run it concurrently with the
 * proto task; it is not reentrant across tasks, and the result hangs the flash
 * wait and stalls USB (the device drops off the bus). In that case we just wait
 * for the completion event: the proto task is already draining BLE events. */
extern void ble_serial_poll(void);
extern bool ble_serial_on_poll_task(void);

/* Forward a non-flash SoC event to TinyUSB. USB VBUS events arrive here once the
 * SoftDevice owns POWER (it does, when BLE is up) - without this the stack never
 * hears a cable attach that happens after boot. Codes: 0=detected,1=removed,
 * 2=ready. */
static void soc_dispatch_usb(uint32_t evt_id)
{
    switch (evt_id) {
    case NRF_EVT_POWER_USB_DETECTED:    cu_power_vbus(true);
                                        tusb_hal_nrf_power_event(0); break;
    case NRF_EVT_POWER_USB_REMOVED:     cu_power_vbus(false);
                                        tusb_hal_nrf_power_event(1); break;
    case NRF_EVT_POWER_USB_POWER_READY: tusb_hal_nrf_power_event(2); break;
    default: break;
    }
}

/* True while wait_flash_event is actively draining the SoC queue. sd_evt_get is
 * a destructive read, so the idle poll-loop drain (cu_soc_drain) must stand down
 * while a flash op owns the queue, or it could swallow the completion event. */
static volatile bool s_flash_waiting;

/* Idle SoC drain for USB VBUS events, called from ble_serial_poll. Defers to
 * wait_flash_event when a flash op is in flight (it dispatches USB events
 * itself meanwhile, so nothing is lost). */
void cu_soc_drain(void)
{
    if (s_flash_waiting) return;
    uint32_t evt_id;
    while (svc_sd_evt_get(&evt_id) == NRF_SUCCESS)
        soc_dispatch_usb(evt_id);   /* flash events shouldn't occur here */
}

/* Wait for the SoftDevice's async flash-completion SoC event. Yields the CPU
 * between polls so the CLI/proto and idle tasks keep running. Bounded (~3s) so
 * a missed event fails the op instead of hanging. USB VBUS events that land
 * mid-wait are dispatched rather than discarded. */
static int wait_flash_event(void)
{
    bool may_pump = ble_serial_on_poll_task();
    s_flash_waiting = true;
    int ret = -1;
    for (int i = 0; i < 3000; i++) {
        uint32_t evt_id;
        uint32_t rc = svc_sd_evt_get(&evt_id);
        if (rc == NRF_SUCCESS) {
            if (evt_id == NRF_EVT_FLASH_OPERATION_SUCCESS) { ret =  0; break; }
            if (evt_id == NRF_EVT_FLASH_OPERATION_ERROR)   { ret = -1; break; }
            soc_dispatch_usb(evt_id);   /* unrelated SoC event - handle USB, keep polling */
            continue;
        }
        if (may_pump) ble_serial_poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    s_flash_waiting = false;
    return ret;
}

/* sd_flash_write/erase return NRF_ERROR_BUSY when the SoftDevice can't take
 * the request yet (radio active). Retry with a yield instead of failing. */
static int flash_sd_write(uint32_t *dst, const uint32_t *src, uint32_t words)
{
    for (int attempt = 0; attempt < 2000; attempt++) {
        uint32_t rc = svc_flash_write(dst, src, words);
        if (rc == NRF_SUCCESS) return wait_flash_event();
        if (rc != NRF_ERROR_BUSY_VAL) return -1;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return -1;
}

static int flash_sd_erase(uint32_t page_number)
{
    for (int attempt = 0; attempt < 2000; attempt++) {
        uint32_t rc = svc_flash_page_erase(page_number);
        if (rc == NRF_SUCCESS) return wait_flash_event();
        if (rc != NRF_ERROR_BUSY_VAL) return -1;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return -1;
}

static uint32_t s_base;

int storage_flash_init(void)
{
    uint32_t bl_addr = UICR_BOOTLOADER_ADDR;
    if (bl_addr == 0xFFFFFFFFUL || bl_addr > 0x100000UL)
        bl_addr = 0xF3000UL;

    if (bl_addr < STORAGE_SIZE)
        return -1;

    s_base = (bl_addr - STORAGE_SIZE) & ~(STORAGE_PAGE_SIZE - 1);

    extern uint8_t _eflash;
    if ((uint32_t)&_eflash > s_base)
        return -1;

    return 0;
}

uint32_t storage_flash_base(void)
{
    return s_base;
}

/* ---- NVMC helpers ---- */

static void nvmc_wait(void)
{
    while (!NRF_NVMC->READY) {}
}

/* ---- public API ---- */

int storage_flash_read(uint32_t offset, void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    memcpy(buf, (const void *)(s_base + offset), len);
    return 0;
}

int storage_flash_erase(uint32_t page_index)
{
    if (page_index >= STORAGE_PAGE_COUNT) return -1;

    uint32_t page_addr = s_base + page_index * STORAGE_PAGE_SIZE;

    if (cu_ble_sd_is_active()) {
        /* Vote out of deep sleep for the whole async op (request + completion
         * event wait) so tickless can't lengthen the completion latency. */
        pwr_inhibit_enter(PWR_CLIENT_FLASH);
        int rc = flash_sd_erase(page_addr / STORAGE_PAGE_SIZE);
        pwr_inhibit_exit(PWR_CLIENT_FLASH);
        return rc;
    }

    __disable_irq();
    nvmc_wait();
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    nvmc_wait();

    NRF_NVMC->ERASEPAGE = page_addr;
    nvmc_wait();

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    nvmc_wait();
    __enable_irq();

    return 0;
}

int storage_flash_program(uint32_t offset, const void *buf, size_t len)
{
    if (offset + len > STORAGE_SIZE) return -1;
    if ((offset & 3) || (len & 3)) return -1;

    uint32_t *dst = (uint32_t *)(s_base + offset);
    const uint32_t *src = (const uint32_t *)buf;
    size_t words = len / 4;

    if (cu_ble_sd_is_active()) {
        pwr_inhibit_enter(PWR_CLIENT_FLASH);
        int rc = 0;
        /* sd_flash_write requires a word-aligned source. Callers (lfs) align the
         * destination offset/length but not always the source pointer, so bounce
         * an unaligned source through an aligned buffer. (The direct NVMC path
         * below tolerates an unaligned source via Cortex-M4 unaligned reads.) */
        if ((uintptr_t)buf & 3) {
            uint32_t bounce[STORAGE_CACHE_SIZE / 4];
            const uint8_t *s = (const uint8_t *)buf;
            uint32_t off = offset, rem = (uint32_t)len;
            while (rem) {
                uint32_t chunk = rem > sizeof(bounce) ? sizeof(bounce) : rem;
                memcpy(bounce, s, chunk);
                if (flash_sd_write((uint32_t *)(s_base + off), bounce, chunk / 4) != 0) {
                    rc = -1;
                    break;
                }
                s += chunk; off += chunk; rem -= chunk;
            }
        } else {
            rc = flash_sd_write(dst, src, words);
        }
        pwr_inhibit_exit(PWR_CLIENT_FLASH);
        return rc;
    }

    volatile uint32_t *vdst = (volatile uint32_t *)dst;

    __disable_irq();
    nvmc_wait();
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    nvmc_wait();

    for (size_t i = 0; i < words; i++) {
        vdst[i] = src[i];
        nvmc_wait();
    }

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    nvmc_wait();
    __enable_irq();

    return 0;
}
