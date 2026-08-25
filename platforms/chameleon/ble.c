/* Fantasi / Chameleon Ultra - BLE via SoftDevice S140.
 *
 * The nRF52840 ships with Nordic SoftDevice S140 v7.2.0 in flash at
 * 0x1000-0x27000. The SoftDevice is activated once at boot and stays
 * active permanently for BLE serial + scanning.
 *
 * The SoftDevice API is SVC-based: each sd_*() call is an SVC
 * instruction that the MBR dispatches to the SD binary. FreeRTOS
 * only uses SVC #0 at scheduler start and PendSV/SysTick afterward,
 * so there is no conflict with the SD's SVC range (0x10+). */

#include "ble.h"
#include "power.h"
#include "nrf.h"
#include "../../core/cli.h"
#include "../../core/log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stddef.h>

/* ---- SVC numbers (S140 v7.2.0) ---- */

#define SVC_SD_ENABLE          0x10
#define SVC_SD_DISABLE         0x11
#define SVC_SD_VECTOR_TABLE_BASE_SET 0x13
#define SVC_BLE_ENABLE         0x60
#define SVC_BLE_EVT_GET        0x61
#define SVC_BLE_CFG_SET        0x69
#define SVC_BLE_GAP_SCAN_START 0x8A
#define SVC_BLE_GAP_SCAN_STOP  0x8B
#define SVC_SD_POWER_USBPWRRDY_ENABLE   0x4D
#define SVC_SD_POWER_USBDETECTED_ENABLE 0x4E
#define SVC_SD_POWER_USBREMOVED_ENABLE  0x4F
#define SVC_SD_POWER_DCDC_MODE_SET      0x3F   /* SOC_SVC_BASE_NOT_AVAILABLE(0x2C) + 19 */
#define SVC_SD_CLOCK_HFCLK_REQUEST      0x42   /* SOC_SVC_BASE_NOT_AVAILABLE + 22 */
#define SVC_SD_CLOCK_HFCLK_RELEASE      0x43
#define SVC_SD_CLOCK_HFCLK_IS_RUNNING   0x44

/* ---- BLE constants ---- */

#define BLE_GAP_EVT_TIMEOUT     0x1B
#define BLE_GAP_EVT_ADV_REPORT  0x1D
#define BLE_GAP_SCAN_BUFFER_MIN 31
#define BLE_GAP_PHY_1MBPS       0x01
#define BLE_GAP_TIMEOUT_SRC_SCAN 0x01

#define BLE_COMMON_CFG_VS_UUID      0x01
#define BLE_GAP_CFG_ROLE_COUNT      0x40
#define BLE_GATTS_CFG_ATTR_TAB_SIZE 0xA1
#define BLE_CONN_CFG_GAP            0x20
#define BLE_CONN_CFG_GATTS          0x22
#define BLE_CONN_CFG_GATT           0x23
#define BLE_GATTS_ATTR_TAB_SIZE_MIN 248

#define AD_TYPE_SHORT_NAME  0x08
#define AD_TYPE_FULL_NAME   0x09

/* ---- Scan-specific structs ---- */

/* sd_scan_params_t defined in ble.h */

typedef struct {
    uint16_t connectable   : 1;
    uint16_t scannable     : 1;
    uint16_t directed      : 1;
    uint16_t scan_response : 1;
    uint16_t extended_pdu  : 1;
    uint16_t status        : 2;
    uint16_t reserved      : 9;
} sd_adv_type_t;

typedef struct {
    uint16_t aux_offset;
    uint8_t  aux_phy;
} sd_aux_ptr_t;

typedef struct {
    sd_adv_type_t type;
    sd_addr_t     peer_addr;
    sd_addr_t     direct_addr;
    uint8_t       primary_phy;
    uint8_t       secondary_phy;
    int8_t        tx_power;
    int8_t        rssi;
    uint8_t       ch_index;
    uint8_t       set_id;
    uint16_t      data_id;
    sd_data_t     data;
    sd_aux_ptr_t  aux_pointer;
} sd_adv_report_t;

_Static_assert(sizeof(sd_lfclk_cfg_t) == 4, "lfclk_cfg layout mismatch");
_Static_assert(sizeof(sd_data_t) == 8, "data_t layout mismatch");
_Static_assert(sizeof(sd_addr_t) == 7, "addr_t layout mismatch");
_Static_assert(sizeof(sd_scan_params_t) == 14, "scan_params layout mismatch");
_Static_assert(sizeof(sd_adv_type_t) == 2, "adv_type layout mismatch");
_Static_assert(sizeof(sd_aux_ptr_t) == 4, "aux_ptr layout mismatch");
_Static_assert(sizeof(sd_adv_report_t) == 36, "adv_report layout mismatch");
_Static_assert(offsetof(sd_adv_report_t, peer_addr) == 2, "");
_Static_assert(offsetof(sd_adv_report_t, rssi) == 19, "");
_Static_assert(offsetof(sd_adv_report_t, data) == 24, "");

/* ---- SVC stubs ---- */

SVCALL(SVC_SD_ENABLE,          uint32_t, svc_sd_enable(const sd_lfclk_cfg_t *cfg, sd_fault_handler_t h))
SVCALL(SVC_SD_DISABLE,         uint32_t, svc_sd_disable(void))
SVCALL(SVC_SD_VECTOR_TABLE_BASE_SET, uint32_t, svc_sd_vector_table_base_set(uint32_t addr))
SVCALL(SVC_BLE_ENABLE,         uint32_t, svc_ble_enable(uint32_t *p_ram))
SVCALL(SVC_BLE_CFG_SET,        uint32_t, svc_ble_cfg_set(uint32_t id, const void *cfg, uint32_t ram))
SVCALL(SVC_BLE_EVT_GET,        uint32_t, svc_ble_evt_get(uint8_t *buf, uint16_t *len))
SVCALL(SVC_BLE_GAP_SCAN_START, uint32_t, svc_scan_start(const sd_scan_params_t *p, const sd_data_t *buf))
SVCALL(SVC_BLE_GAP_SCAN_STOP,  uint32_t, svc_scan_stop(void))
SVCALL(SVC_SD_POWER_USBPWRRDY_ENABLE,   uint32_t, svc_power_usbpwrrdy_enable(uint8_t en))
SVCALL(SVC_SD_POWER_USBDETECTED_ENABLE, uint32_t, svc_power_usbdetected_enable(uint8_t en))
SVCALL(SVC_SD_POWER_USBREMOVED_ENABLE,  uint32_t, svc_power_usbremoved_enable(uint8_t en))
SVCALL(SVC_SD_POWER_DCDC_MODE_SET,      uint32_t, svc_power_dcdc_mode_set(uint8_t mode))
SVCALL(SVC_SD_CLOCK_HFCLK_REQUEST,      uint32_t, svc_clock_hfclk_request(void))
SVCALL(SVC_SD_CLOCK_HFCLK_RELEASE,      uint32_t, svc_clock_hfclk_release(void))
SVCALL(SVC_SD_CLOCK_HFCLK_IS_RUNNING,   uint32_t, svc_clock_hfclk_is_running(uint32_t *running))
SVCALL(0x74,                   uint32_t, svc_gap_adv_stop(uint8_t adv_handle))
SVCALL(0x73,                   uint32_t, svc_gap_adv_start_scan(uint8_t adv_handle, uint8_t tag))

/* ---- VTOR management ---- */

extern void (* const g_vector_table[])(void);
static uint32_t saved_vtor;

/* ---- State ---- */

static volatile bool sd_active;

/* ---- SoftDevice fault handler ---- */

static void sd_fault(uint32_t id, uint32_t pc, uint32_t info)
{
    (void)id; (void)pc; (void)info;
    /* Fingerprint the fault (retained register; the SD is broken anyway, so
     * the direct write is fine) - the next boot logs it. */
    NRF_POWER->GPREGRET2 = 0xA2;   /* HAL_CRASH_RADIO_FAULT */
    NVIC_SystemReset();
}

/* ---- Persistent SoftDevice init ---- */

bool cu_ble_sd_init(void)
{
    if (sd_active) return true;

    NVIC_DisableIRQ(POWER_CLOCK_IRQn);
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);

    /* If the app started the LFXO by hand (ble=0 boot, `ble on` later), hand
     * the clock back before sd_enable - the SD insists on owning LFCLK and
     * enabling it over an app-started clock can stall inside the SVC. */
    if (!cu_power_release_lfclk_for_sd()) {
        cli_write("ble: LFCLK handoff timed out\r\n");
        NVIC_EnableIRQ(POWER_CLOCK_IRQn);
        return false;
    }

    /* While VTOR points at the MBR below (until the SD learns our app vector
     * base), an app peripheral IRQ would vector through the MBR with no
     * forwarding target and fault. POWER_CLOCK is masked above; the other app
     * IRQs live at this point - USBD (busy during enumeration), GPIOTE
     * (button wake), RTC1 (tickless wake timer) - are masked across the
     * window too. Pending is not cleared, so an event latched here still
     * fires (correctly vectored) once re-enabled. This makes bring-up
     * deterministic instead of relying on a settle delay. */
    NVIC_DisableIRQ(USBD_IRQn);
    NVIC_DisableIRQ(GPIOTE_IRQn);
    NVIC_DisableIRQ(RTC1_IRQn);

    saved_vtor = SCB->VTOR;
    SCB->VTOR = 0;
    __DSB();
    __ISB();

    /* LFCLK source: SD default (NULL = LFRC + SD-managed calibration), the
     * known-good bring-up config. RTC1 tickless runs fine off the LFRC
     * (+-250 ppm). Moving to the LFXO (stock uses XTAL, 20 ppm) makes
     * sd_softdevice_enable block ~0.3 s for crystal startup inside the
     * masked-USBD window below - retry that only with USB-safe sequencing. */
    uint32_t rc = svc_sd_enable(NULL, sd_fault);
    if (rc != NRF_SUCCESS) {
        cli_printf("ble: sd_enable 0x%04lx\r\n", (unsigned long)rc);
        SCB->VTOR = saved_vtor;
        __DSB();
        NVIC_EnableIRQ(POWER_CLOCK_IRQn);
        NVIC_EnableIRQ(USBD_IRQn);
        NVIC_EnableIRQ(GPIOTE_IRQn);
        NVIC_EnableIRQ(RTC1_IRQn);
        if (!cu_power_reclaim_lfclk_after_sd_failure())
            cli_write("ble: LFCLK reclaim timed out\r\n");
        return false;
    }

    /* Tell the SD where our app vector table lives so it forwards
     * SysTick, PendSV, and other exceptions to FreeRTOS. */
    svc_sd_vector_table_base_set((uint32_t)g_vector_table);

    /* App vectors are registered - the window is closed, so USB can resume.
     * (POWER_CLOCK stays masked: the SoftDevice owns it from here.) */
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(RTC1_IRQn);

    __attribute__((aligned(4))) uint8_t cfg[16];

    /* VS UUIDs: need 1 for our serial service */
    memset(cfg, 0, sizeof(cfg));
    cfg[0] = 1;
    svc_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, cfg, APP_RAM_BASE);

    /* Role count: 1 peripheral, no central */
    memset(cfg, 0, sizeof(cfg));
    cfg[0] = 1;   /* adv_set_count */
    cfg[1] = 1;   /* periph_role_count */
    cfg[2] = 1;   /* central_role_count */
    cfg[3] = 1;   /* central_sec_count */
    svc_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, cfg, APP_RAM_BASE);

    /* GATTS attribute table: service + 2 chars + CCCD + values */
    memset(cfg, 0, sizeof(cfg));
    *(uint32_t *)cfg = 1408;
    svc_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, cfg, APP_RAM_BASE);

    /* conn_cfg_tag 1: MTU 247 + required GAP/GATTS settings.
     * All three must be configured for a custom tag.
     * conn_count is 1: only the peripheral (serial) connection uses tag 1;
     * the central role (sd_ble_gap_connect) uses the default tag 0. A
     * conn_count of 2 here over-reserves links and makes adv_start(tag 1)
     * fail (NRF_ERROR_CONN_COUNT), which silently kills advertising. */
    memset(cfg, 0, sizeof(cfg));
    cfg[0] = 1;                            /* conn_cfg_tag */
    cfg[2] = 1;                            /* conn_count (peripheral only) */
    /* event_length 10ms (8 x 1.25ms): with a 15ms interval this lets the
     * SoftDevice pack many notifications per connection event (download hit
     * ~215 kbps vs ~39 kbps at 5ms) while still leaving ~5ms of radio-idle
     * per interval for the flash scheduler - sd_flash_write only runs when
     * the radio is idle, and over-long events starve it. */
    *(uint16_t *)&cfg[4] = 8;
    svc_ble_cfg_set(BLE_CONN_CFG_GAP, cfg, APP_RAM_BASE);

    memset(cfg, 0, sizeof(cfg));
    cfg[0] = 1;                            /* conn_cfg_tag */
    cfg[2] = 4;                            /* hvn_tx_queue_size (match tx_credits) */
    svc_ble_cfg_set(BLE_CONN_CFG_GATTS, cfg, APP_RAM_BASE);

    memset(cfg, 0, sizeof(cfg));
    cfg[0] = 1;                            /* conn_cfg_tag */
    *(uint16_t *)&cfg[2] = 247;            /* att_mtu */
    svc_ble_cfg_set(BLE_CONN_CFG_GATT, cfg, APP_RAM_BASE);

    uint32_t ram_start = APP_RAM_BASE;
    rc = svc_ble_enable(&ram_start);
    if (rc != NRF_SUCCESS) {
        fantasi_log(LOG_ERROR, "ble_enable 0x%04lx need 0x%08lx",
                    (unsigned long)rc, (unsigned long)ram_start);
        uint32_t disable_rc = svc_sd_disable();
        if (disable_rc != NRF_SUCCESS) {
            /* A half-enabled SoftDevice cannot safely be returned to code
             * that believes CLOCK and POWER are app-owned.  Reboot into a
             * known state instead of risking direct-register corruption. */
            cli_printf("ble: sd_disable 0x%04lx\r\n",
                       (unsigned long)disable_rc);
            NVIC_SystemReset();
        }
        SCB->VTOR = saved_vtor;
        __DSB();
        NVIC_EnableIRQ(POWER_CLOCK_IRQn);
        if (!cu_power_reclaim_lfclk_after_sd_failure())
            cli_write("ble: LFCLK reclaim timed out\r\n");
        return false;
    }

    sd_active = true;

    /* The SoftDevice now owns POWER_CLOCK, so the app's POWER ISR no longer sees
     * VBUS attach/detach. Enable the USB power events as SoC events instead;
     * cu_soc_drain (in ble_serial_poll) forwards them to TinyUSB. Without this,
     * plugging the cable in after the SoftDevice came up never enumerates USB. */
    svc_power_usbdetected_enable(1);
    svc_power_usbpwrrdy_enable(1);
    svc_power_usbremoved_enable(1);

    return true;
}

bool cu_ble_sd_is_active(void)
{
    return sd_active;
}

bool cu_hfclk_request(void)
{
    if (!sd_active) {
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        for (uint32_t i = 0; i < 200000; i++)
            if (NRF_CLOCK->EVENTS_HFCLKSTARTED) return true;
        return false;
    }

    if (svc_clock_hfclk_request() != NRF_SUCCESS) return false;
    for (uint32_t i = 0; i < 200; i++) {
        uint32_t running = 0;
        if (svc_clock_hfclk_is_running(&running) != NRF_SUCCESS) break;
        if (running) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    svc_clock_hfclk_release();
    return false;
}

void cu_hfclk_release(void)
{
    if (sd_active) svc_clock_hfclk_release();
}

/* Strong override of the tinyusb nRF driver's weak hook. While the SoftDevice is
 * active it owns the HFCLK, so the USB driver must not write NRF_CLOCK directly
 * (the SD traps it as APP_MEMACC and resets - the 0xA2 crash on USB disconnect,
 * where the driver's hfclk_disable() would otherwise stop the HFXO). The HFXO
 * runs regardless: started at boot for USB and kept up by the SD. */
bool tud_nrf_hfclk_external(void)
{
    return sd_active;
}

/* ---- Advertising-data parser ---- */

static void parse_ad_name(const uint8_t *ad, uint8_t ad_len,
                          char *name, uint8_t name_sz)
{
    name[0] = '\0';
    const uint8_t *p = ad;
    const uint8_t *end = ad + ad_len;

    while (p < end) {
        uint8_t len  = p[0];
        if (len == 0 || p + 1 + len > end) break;
        uint8_t type = p[1];

        if (type == AD_TYPE_FULL_NAME || type == AD_TYPE_SHORT_NAME) {
            uint8_t nlen = len - 1;
            if (nlen > name_sz - 1)
                nlen = name_sz - 1;
            memcpy(name, p + 2, nlen);
            name[nlen] = '\0';
            if (type == AD_TYPE_FULL_NAME) return;
        }
        p += 1 + len;
    }
}

/* ---- BLE scan ---- */

#define ADV_REPORT_OFFSET 8

int cu_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms)
{
    if (!cu_ble_sd_init())
        return -1;

    extern uint8_t adv_handle;
    extern volatile bool ble_scan_active;

    if (adv_handle != 0xFF)
        svc_gap_adv_stop(adv_handle);

    ble_scan_active = true;

    static uint8_t scan_buf[BLE_GAP_SCAN_BUFFER_MIN];
    sd_data_t adv_buf = { .p_data = scan_buf, .len = sizeof(scan_buf) };

    sd_scan_params_t params;
    memset(&params, 0, sizeof(params));
    params.scan_phys = BLE_GAP_PHY_1MBPS;
    params.interval  = 160;
    params.window    = 80;
    params.timeout   = (uint16_t)(duration_ms / 10);
    if (params.timeout == 0) params.timeout = 1;

    uint32_t rc = svc_scan_start(&params, &adv_buf);
    if (rc != NRF_SUCCESS) {
        cli_printf("ble: scan_start 0x%04lx\r\n", (unsigned long)rc);
        return -1;
    }

    int count = 0;
    __attribute__((aligned(4))) uint8_t evt[256];
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms + 500);

    for (;;) {
        if (xTaskGetTickCount() >= deadline)
            break;

        uint16_t len = sizeof(evt);
        rc = svc_ble_evt_get(evt, &len);

        if (rc == NRF_ERROR_NOT_FOUND) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (rc != NRF_SUCCESS)
            continue;

        uint16_t evt_id = *(uint16_t *)&evt[0];

        if (evt_id == BLE_GAP_EVT_ADV_REPORT) {
            const sd_adv_report_t *rpt =
                (const sd_adv_report_t *)&evt[ADV_REPORT_OFFSET];

            char name[29];
            parse_ad_name(rpt->data.p_data, rpt->data.len,
                          name, sizeof(name));

            count++;
            if (cb)
                cb(rpt->peer_addr.addr, rpt->peer_addr.addr_type,
                   rpt->rssi, name);

            svc_scan_start(NULL, &adv_buf);

        } else if (evt_id == BLE_GAP_EVT_TIMEOUT) {
            if (evt[ADV_REPORT_OFFSET] == BLE_GAP_TIMEOUT_SRC_SCAN)
                break;
        }
    }

    svc_scan_stop();
    ble_scan_active = false;

    if (adv_handle != 0xFF)
        svc_gap_adv_start_scan(adv_handle, 0);

    return count;
}

/* ---- Radio info ---- */

void cu_ble_radio_info(hal_radio_info_t *info)
{
    __builtin_memset(info, 0, sizeof(*info));

    const uint8_t *si = (const uint8_t *)0x3000;
    uint8_t struct_sz = si[0];
    if (struct_sz == 0 || struct_sz == 0xFF)
        return;

    info->available = true;
    info->secure_flash_start = 0x1000;

    uint32_t sd_size = *(const uint32_t *)(si + 0x08);
    info->secure_flash_kb = sd_size / 1024;

    if (struct_sz > 0x10)
        info->ws_type = *(const uint32_t *)(si + 0x10);

    if (struct_sz > 0x14) {
        uint32_t ver = *(const uint32_t *)(si + 0x14);
        info->ws_major = ver / 1000000;
        info->ws_minor = (ver / 1000) % 1000;
        info->ws_sub   = ver % 1000;
    }
}
