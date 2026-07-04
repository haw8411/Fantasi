/* Fantasi / Chameleon Ultra - HAL glue.
 *
 * Wires the USBD peripheral to TinyUSB and routes the POWER/CLOCK IRQ
 * to TinyUSB's nRF-specific power-event hook (needed so it sees
 * VBUS-detected / VBUS-ready transitions - otherwise the device
 * never enumerates).
 *
 * Read/write/connected/heap live in hal/tinyusb/. */

#include "nrf.h"
#include "../../hal/hal.h"
#include "../../hal/hal_name.h"
#include "ble.h"
#include "ble_serial.h"
#include "hal_storage.h"
#include "tusb.h"
#include "FreeRTOS.h"

/* Declared in TinyUSB's portable/nordic/nrf5x/dcd_nrf5x.c. Forward-
 * declared here so this file doesn't need to reach into TinyUSB's
 * private headers. Event codes are fixed by the TinyUSB API:
 *   0 = USB detected, 1 = USB removed, 2 = USB power ready. */
extern void tusb_hal_nrf_power_event(uint32_t event);
#define FANTASI_USB_EVT_DETECTED 0
#define FANTASI_USB_EVT_REMOVED  1
#define FANTASI_USB_EVT_READY    2

/* The CU's visible LEDs are a multiplexed matrix: 8 slot-position
 * pins (active-low cathode select) × 3 RGB colour pins (active-low
 * anode select). Both a slot pin AND a colour pin must be driven LOW
 * for any light to appear. */
static void gpio_output_low(NRF_GPIO_Type *port, uint32_t pin)
{
    port->PIN_CNF[pin] = (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos)
                       | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    port->OUTCLR = (1UL << pin);
}

static void gpio_output_high(NRF_GPIO_Type *port, uint32_t pin)
{
    port->PIN_CNF[pin] = (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos)
                       | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    port->OUTSET = (1UL << pin);
}

void hal_init(void)
{
    /* Pass the raw priority level to NVIC_SetPriority - CMSIS shifts it
     * up by (8 - __NVIC_PRIO_BITS) internally. Pre-shifting here would
     * double-shift to near-zero (highest priority) and trip FreeRTOS's
     * vPortValidateInterruptPriority assert on the first FromISR call. */
    const uint32_t irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;

    /* Route USB-attach/detach to TinyUSB via POWER_CLOCK_IRQ. */
    NRF_POWER->INTENSET = POWER_INTENSET_USBDETECTED_Msk |
                          POWER_INTENSET_USBREMOVED_Msk  |
                          POWER_INTENSET_USBPWRRDY_Msk;

    NVIC_SetPriority(POWER_CLOCK_IRQn, irq_prio);
    NVIC_SetPriority(USBD_IRQn,        irq_prio);
    NVIC_EnableIRQ(POWER_CLOCK_IRQn);
    NVIC_EnableIRQ(USBD_IRQn);

    /* Steady blue on all 8 slot LEDs.
     * Colour channels (active low, sink side): blue LOW = on.
     * Slot positions  (active high, source side): HIGH = on. */
    gpio_output_high(NRF_P0, 24);              /* LED_R off */
    gpio_output_high(NRF_P0, 22);              /* LED_G off */
    gpio_output_low (NRF_P1,  0);              /* LED_B on  */

    gpio_output_high(NRF_P0, 20);              /* LED_1 on */
    gpio_output_high(NRF_P0, 17);              /* LED_2 on */
    gpio_output_high(NRF_P0, 15);              /* LED_3 on */
    gpio_output_high(NRF_P0, 13);              /* LED_4 on */
    gpio_output_high(NRF_P0, 12);              /* LED_5 on */
    gpio_output_high(NRF_P1,  9);              /* LED_6 on */
    gpio_output_high(NRF_P0,  8);              /* LED_7 on */
    gpio_output_high(NRF_P0,  6);              /* LED_8 on */

    hal_storage_init();

    tusb_init();
}

void USBD_IRQHandler(void) { tud_int_handler(0); }

void POWER_CLOCK_IRQHandler(void)
{
    /* Translate the three USB power events TinyUSB cares about. */
    if (NRF_POWER->EVENTS_USBDETECTED) {
        NRF_POWER->EVENTS_USBDETECTED = 0;
        tusb_hal_nrf_power_event(FANTASI_USB_EVT_DETECTED);
    }
    if (NRF_POWER->EVENTS_USBPWRRDY) {
        NRF_POWER->EVENTS_USBPWRRDY = 0;
        tusb_hal_nrf_power_event(FANTASI_USB_EVT_READY);
    }
    if (NRF_POWER->EVENTS_USBREMOVED) {
        NRF_POWER->EVENTS_USBREMOVED = 0;
        tusb_hal_nrf_power_event(FANTASI_USB_EVT_REMOVED);
    }
}

void hal_post_init(void)
{
    /* Bring BLE up at boot unless the persisted setting says off, mirroring
     * the Flipper's hal_post_init. Default on when the key is absent.
     * cu_ble_sd_init() masks USBD across its VTOR-swap window, so SoftDevice
     * enable is safe even mid-enumeration - no settle delay needed. */
    char v[4] = "1";
    hal_settings_get("ble", v, sizeof(v));
    if (v[0] == '0') return;
    ble_serial_resume();
}

const char *hal_device_id(void) { return "CU"; }

const char *hal_device_name(void)
{
    static char name[16];
    if (name[0]) return name;
    volatile uint32_t *ficr = (volatile uint32_t *)0x10000060;
    uint32_t u[2] = { ficr[0], ficr[1] };
    hal_name_generate(u, 2, name, sizeof(name));
    return name;
}

extern uint8_t _eflash;

int32_t hal_flash_free_bytes(void)
{
    uint32_t used_end  = (uint32_t)&_eflash;
    uint32_t flash_end = 0x00027000U + 0xCC000U;
    if (flash_end <= used_end) return 0;
    return (int32_t)(flash_end - used_end);
}

static uint32_t map_range(uint32_t x, uint32_t in_min, uint32_t in_max,
                          uint32_t out_min, uint32_t out_max)
{
    if (x < in_min) return out_min;
    uint32_t v = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return v < out_max ? v : out_max;
}

static int mv_to_percent(uint32_t mv)
{
    if (mv >= 4034) return (int)map_range(mv, 4034, 4200, 80, 100);
    if (mv >= 3904) return (int)map_range(mv, 3904, 4034, 60, 80);
    if (mv >= 3824) return (int)map_range(mv, 3824, 3904, 40, 60);
    if (mv >= 3754) return (int)map_range(mv, 3754, 3824, 20, 40);
    if (mv >= 3644) return (int)map_range(mv, 3644, 3754, 5, 20);
    return 0;
}

static void saadc_oneshot(volatile int16_t *buf)
{
    NRF_SAADC->RESULT.PTR    = (uint32_t)buf;
    NRF_SAADC->RESULT.MAXCNT = 1;

    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_START    = 1;
    while (!NRF_SAADC->EVENTS_STARTED) {}

    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (!NRF_SAADC->EVENTS_END) {}

    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->TASKS_STOP     = 1;
    while (!NRF_SAADC->EVENTS_STOPPED) {}
}

int hal_battery_percent(void)
{
    /* One-shot SAADC read on AIN2 (P0.04) - the CU battery sense pin.
     * Config matches the original ChameleonUltra firmware: gain 1/6,
     * internal 0.6 V reference, 10 µs acquisition, 14-bit resolution,
     * single-ended. */
    volatile int16_t result;

    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_14bit;
    NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;

    NRF_SAADC->CH[0].PSELP  = SAADC_CH_PSELP_PSELP_AnalogInput2;
    NRF_SAADC->CH[0].PSELN  = 0;
    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos)
                             | (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos)
                             | (SAADC_CH_CONFIG_TACQ_10us      << SAADC_CH_CONFIG_TACQ_Pos)
                             | (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos)
                             | (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos);

    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;

    /* Run offset calibration - the SAADC's internal offset drifts with
     * temperature and supply; without this the first sample after a cold
     * boot can be off by tens of mV. */
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    while (!NRF_SAADC->EVENTS_CALIBRATEDONE) {}

    /* Discard the first sample after calibration - the nRF52840 errata
     * notes that it can carry stale comparator state. */
    saadc_oneshot(&result);
    saadc_oneshot(&result);

    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;

    int16_t raw = result;
    if (raw < 0) raw = 0;
    uint32_t mv = (uint32_t)raw * 7200 / 16383 + 100;

    return mv_to_percent(mv);
}

int hal_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms)
{
    return cu_ble_scan(cb, duration_ms);
}

void hal_radio_info(hal_radio_info_t *info)
{
    cu_ble_radio_info(info);
}


int hal_enter_msc_mode(void) { return -1; }

/* Composite: the vendor/WebUSB interface is always present alongside CDC. */
int hal_enter_webusb_mode(void) { return -1; }
int hal_enter_cdc_mode(void) { return -1; }

int hal_ble_pair_setup(uint8_t io_cap)
{ return ble_pair_setup_security(io_cap); }
void hal_ble_pair_begin(void) { ble_pair_set_manual(true); }
void hal_ble_pair_end(void)   { ble_pair_set_manual(false); }
int hal_ble_pair_connect(const uint8_t *addr, uint8_t addr_type)
{ return ble_pair_connect(addr, addr_type); }
void hal_ble_shutdown(void) { ble_serial_stop(); }
void hal_ble_activate_fus(void) {}
bool hal_ble_is_active(void) { return ble_serial_is_active(); }
int hal_ble_pair_initiate(uint16_t conn_handle)
{ return ble_pair_initiate(conn_handle); }
int hal_ble_pair_passkey(uint16_t conn_handle, uint32_t passkey)
{ return ble_pair_send_passkey(conn_handle, passkey); }
int hal_ble_pair_confirm(uint16_t conn_handle, bool accept)
{ return ble_pair_numeric_confirm(conn_handle, accept); }
int hal_ble_pair_wait(hal_ble_evt_t *evt, uint32_t timeout_ms)
{ return ble_pair_wait_event(evt, timeout_ms); }
int hal_ble_disconnect(uint16_t conn_handle)
{ return ble_pair_disconnect(conn_handle); }
uint32_t hal_ble_generate_passkey(void)
{ return 0; }
int hal_ble_connections(hal_ble_conn_info_t *out, int max)
{ (void)out; (void)max; return 0; }
int hal_ble_get_bonded(hal_ble_bonded_t *out, int max)
{ (void)out; (void)max; return 0; }
int hal_ble_remove_bond(const uint8_t *addr, uint8_t addr_type)
{ (void)addr; (void)addr_type; return -1; }
int hal_ble_clear_bonds(void)
{ return -1; }

extern uint8_t _ram_start, _ram_end;
extern uint8_t __heap_start__, __heap_end__;

int hal_mem_regions(hal_mem_region_t *out, int max)
{
    int n = 0;
    if (n < max) {
        uint32_t unalloc = (uint32_t)&__heap_end__ - (uint32_t)&__heap_start__;
        out[n].name  = "RAM";
        out[n].total = (uint32_t)&_ram_end - (uint32_t)&_ram_start;
        out[n].free  = (uint32_t)hal_free_heap_bytes() + unalloc;
        out[n].note  = NULL;
        n++;
    }
    return n;
}

int hal_test_regions(hal_test_region_t *out, int max)
{
    int n = 0;
    if (n < max) {
        out[n].name = "RAM";
        out[n].addr = (uint32_t)&__heap_start__;
        out[n].size = (uint32_t)&__heap_end__ - (uint32_t)&__heap_start__;
        n++;
    }
    return n;
}

void hal_reboot(void)
{
    NVIC_SystemReset();
    for (;;);
}

/* Hand control back to the nRF secure bootloader so it advertises its
 * DFU service. GPREGRET is a retention byte that survives soft reset;
 * the stock Nordic bootloader reads it early and, on 0xB1
 * (BOOTLOADER_DFU_START = BOOTLOADER_DFU_GPREGRET_MASK | START_BIT),
 * enters DFU instead of forwarding to the application.
 *
 * The ChameleonUltra upstream writes this via sd_power_gpregret_set()
 * because its application runs under the SoftDevice; we don't, so a
 * plain MMIO write is correct. */
#define NRF_BOOTLOADER_DFU_START 0xB1U

void hal_set_dfu_magic(void)
{
    NRF_POWER->GPREGRET = NRF_BOOTLOADER_DFU_START;
}

void hal_reboot_dfu(void)
{
    hal_set_dfu_magic();
    NVIC_SystemReset();
    for (;;);
}

/* SoftDevice SVC for SYSTEM OFF: must be used instead of the raw register
 * when the SoftDevice is active, or it faults. SVC number is the raw Nordic
 * value SOC_SVC_BASE_NOT_AVAILABLE (0x2C) + 7 - matches the other SD SVCs
 * this platform declares (e.g. SD_FLASH_PAGE_ERASE = SOC_SVC_BASE 0x20 + 8). */
#define SVC_SD_POWER_SYSTEM_OFF 0x33
SVCALL(SVC_SD_POWER_SYSTEM_OFF, uint32_t, svc_power_system_off(void))

#define CU_BUTTON_B_PIN 2   /* Button B = P1.02 (active-high, on-die pull-down) */

/* nRF52840 System OFF: the deepest sleep - RAM off, ~1 µA, wakes only via
 * RESET or a GPIO configured with SENSE. We arm button B as that wake source
 * so the next press powers the device back up. Does not return. */
int hal_shutdown(void)
{
    /* Blank the slot LEDs first. They're driven directly by GPIO and the
     * nRF52 retains pin output levels through System OFF, so without this the
     * boot-time blue (hal_init) would stay lit after "power off". Driving the
     * colour channels (active-low sinks) high turns every slot off. */
    gpio_output_high(NRF_P0, 24);   /* LED_R off */
    gpio_output_high(NRF_P0, 22);   /* LED_G off */
    gpio_output_high(NRF_P1,  0);   /* LED_B off */

    /* Wait for release first: entering System OFF while the button is still
     * held (e.g. the long-hold that triggered this) would latch an immediate
     * wake and loop. */
    while (NRF_P1->IN & (1UL << CU_BUTTON_B_PIN))
        ;

    NRF_P1->PIN_CNF[CU_BUTTON_B_PIN] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
        (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);

    __DSB();
    if (cu_ble_sd_is_active())
        svc_power_system_off();     /* SoftDevice performs SYSTEM OFF; no return */
    NRF_POWER->SYSTEMOFF = 1;       /* direct path when SD is down; no return */
    for (;;) __WFE();
}

/* Button B = P1.02. Configure as input+pull-down lazily (idempotent) so we
 * don't depend on any other init running first. */
bool hal_shutdown_button_held(void)
{
    static bool cfg;
    if (!cfg) {
        NRF_P1->PIN_CNF[CU_BUTTON_B_PIN] =
            (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
        cfg = true;
    }
    return (NRF_P1->IN & (1UL << CU_BUTTON_B_PIN)) != 0;
}
