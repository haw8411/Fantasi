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
#include "../../hal/hal_power.h"
#include "../../core/log.h"
#include "ble.h"
#include "ble_serial.h"
#include "power.h"
#include "hal_storage.h"
#include "flash_storage.h"
#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_api.h"   /* FANTASI_HID_* bits for hal_hid_host */
#ifdef FANTASI_ENABLE_APPS
#include "cli.h"
#include "app_run.h"   /* shortcut_run for the A-button launcher */
#endif

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
 * anode select). Both a slot pin and a colour pin must be driven LOW
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

#ifdef FANTASI_ENABLE_APPS
void cu_launcher_init(void);   /* A-button shortcut launcher, defined below */
#endif

/* Previous-boot diagnostics, stashed pre-SoftDevice and logged from
 * hal_post_init (the logger isn't up during hal_init). */
static uint32_t s_resetreas;
static uint8_t  s_crash_note;

void hal_crash_note(uint8_t code)
{
    NRF_POWER->GPREGRET2 = code;   /* retained across soft reset */
}

void hal_init(void)
{
    /* Reset forensics: reason + any crash fingerprint from the previous run.
     * Registers are unrestricted here (SoftDevice not up yet). Also clear
     * GPREGRET: stale DFU magic (0xB1) left after a `dfu` session makes every
     * later reset - including crash resets - detour ~30 s through the
     * bootloader's DFU mode before chaining to the app. */
    s_resetreas  = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFFu;    /* write-1-to-clear */
    s_crash_note = (uint8_t)NRF_POWER->GPREGRET2;
    NRF_POWER->GPREGRET2 = 0;
    NRF_POWER->GPREGRET  = 0;

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

    /* Sleep machinery: RTC1 (tickless), button-wake GPIOTE, idle policy
     * timer. Before the SoftDevice, so registers are unrestricted. */
    cu_power_init();

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

    /* Apply the USB interface toggles before USB comes up so the device
     * enumerates correctly the first time. Defaults (missing keys): HID
     * persistent, MSC on. */
    {
        char hv[12] = "persistent";
        hal_settings_get("hid", hv, sizeof(hv));
        hal_hid_set_persistent(hv[0] != 's');   /* "switch" -> non-persistent */

        char mv[4] = "1";
        hal_settings_get("msc", mv, sizeof(mv));
        hal_msc_set_enabled(mv[0] != '0');
    }

    tusb_init();

#ifdef FANTASI_ENABLE_APPS
    cu_launcher_init();   /* A-button shortcut launcher (owns the position LEDs) */
#endif
}

void USBD_IRQHandler(void) { tud_int_handler(0); }

void POWER_CLOCK_IRQHandler(void)
{
    /* Translate the three USB power events TinyUSB cares about. */
    if (NRF_POWER->EVENTS_USBDETECTED) {
        NRF_POWER->EVENTS_USBDETECTED = 0;
        cu_power_vbus(true);
        tusb_hal_nrf_power_event(FANTASI_USB_EVT_DETECTED);
    }
    if (NRF_POWER->EVENTS_USBPWRRDY) {
        NRF_POWER->EVENTS_USBPWRRDY = 0;
        tusb_hal_nrf_power_event(FANTASI_USB_EVT_READY);
    }
    if (NRF_POWER->EVENTS_USBREMOVED) {
        NRF_POWER->EVENTS_USBREMOVED = 0;
        cu_power_vbus(false);
        tusb_hal_nrf_power_event(FANTASI_USB_EVT_REMOVED);
    }
}

void hal_post_init(void)
{
    /* Log the previous boot's exit: reset reason (RESETREAS: bit0 pin-reset,
     * bit2 soft-reset/SYSRESETREQ, bit3 CPU lockup, ...) and any crash
     * fingerprint (0xA1 stack overflow, 0xA2 SoftDevice fault). */
    fantasi_log(LOG_INFO, "resetreas 0x%lx crash 0x%02x",
                (unsigned long)s_resetreas, s_crash_note);

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
    /* Free program flash = the slack below the LittleFS storage region, not up to the
     * bootloader. The 256 KB LittleFS sits at the top of the app region (~0xB3000, just
     * below the bootloader) and is the "/" mount reported separately in df, so the ceiling
     * is its base, not the bootloader address. */
    uint32_t used_end  = (uint32_t)&_eflash;
    uint32_t flash_end = storage_flash_base();
    if (!flash_end || flash_end <= used_end) return 0;
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

/* ---- USB HID keyboard emulation ----
 * Persistent by default (keyboard always enumerated with CDC/MSC/vendor),
 * switch mode re-enumerates. All TinyUSB-generic. State lives in
 * usb_descriptors.c. */
void    usb_desc_set_hid_persistent(bool on);
bool    usb_desc_hid_persistent(void);
void    usb_desc_set_hid_active(bool on);
bool    usb_desc_hid_active(void);
uint8_t usb_desc_hid_host_leds(void);
void    usb_desc_set_msc_enabled(bool on);

static void hid_release_keys(void)
{
    uint8_t empty[6] = { 0 };
    if (tud_hid_ready()) tud_hid_keyboard_report(0, 0, empty);
}

int hal_hid_enable(int on)
{
    if (usb_desc_hid_persistent()) {
        if (!on) { hid_release_keys(); return 0; }
    } else {
        if ((bool)on != usb_desc_hid_active()) {
            if (!on) hid_release_keys();
            usb_desc_set_hid_active(on);
            hal_usb_reenumerate();
        }
        if (!on) return 0;
    }
    for (int i = 0; i < 3000; i++) {
        if (tud_mounted() && tud_hid_ready()) return 0;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return tud_mounted() ? 0 : -1;
}

int hal_hid_send(uint8_t modifiers, const uint8_t *keys, uint8_t n)
{
    if (!tud_mounted()) return -1;

    uint8_t report[6] = { 0 };
    for (uint8_t i = 0; i < n && i < 6; i++) report[i] = keys[i];

    for (int i = 0; i < 200 && !tud_hid_ready(); i++) vTaskDelay(pdMS_TO_TICKS(1));
    if (!tud_hid_ready()) return -1;

    return tud_hid_keyboard_report(0, modifiers, report) ? 0 : -1;
}

uint32_t hal_hid_host(void)
{
    uint32_t bits = usb_desc_hid_host_leds();
    if (tud_mounted()) bits |= FANTASI_HID_HOST_MOUNTED;
    return bits;
}

void hal_hid_set_persistent(bool persistent)
{
    usb_desc_set_hid_persistent(persistent);
    if (!persistent) usb_desc_set_hid_active(false);
}

void hal_msc_set_enabled(bool enabled)
{
    usb_desc_set_msc_enabled(enabled);
}

/* ---- A-button shortcut launcher (screenless slot selection) ----
 * The CU has no screen, so app shortcuts (scN=<path> in settings.cfg, slots 0-7)
 * are chosen with the A button and shown on the 8 position LEDs:
 *   - LED_8 stays lit at all times; it also marks slot 0 (slot 0 = LED_8 only).
 *   - Slots 1..7 additionally light LED_1..LED_7, so a short-press "walk" shows
 *     which slot you're on. A short press advances the selection (wraps 7->0).
 *   - Holding A for CU_HOLD_MS launches the app in the selected slot.
 * Button A = the Ultra's BUTTON_2 (P0.26, active-high with an on-die pull-down);
 * the shutdown button (P1.02) is Button B. LEDs are all blue (shared rail). */
#ifdef FANTASI_ENABLE_APPS

#define CU_BUTTON_A_PORT  NRF_P0
#define CU_BUTTON_A_PIN   26
#define CU_HOLD_MS        600

/* LED_1..LED_8 slot pins (index 0..7); mirrors the boot setup in hal_init. */
static const struct { NRF_GPIO_Type *port; uint8_t pin; } cu_slot_led[8] = {
    { NRF_P0, 20 }, { NRF_P0, 17 }, { NRF_P0, 15 }, { NRF_P0, 13 },
    { NRF_P0, 12 }, { NRF_P1,  9 }, { NRF_P0,  8 }, { NRF_P0,  6 },
};

/* Light blue on the slots set in `mask` (bit i -> LED_(i+1)); the rest off. */
static void cu_leds_show(uint8_t mask)
{
    gpio_output_high(NRF_P0, 24);   /* R off */
    gpio_output_high(NRF_P0, 22);   /* G off */
    gpio_output_low (NRF_P1,  0);   /* B on  */
    for (int i = 0; i < 8; i++) {
        if (mask & (1u << i)) gpio_output_high(cu_slot_led[i].port, cu_slot_led[i].pin);
        else                  gpio_output_low (cu_slot_led[i].port, cu_slot_led[i].pin);
    }
}

static bool cu_button_a_down(void)
{
    return (CU_BUTTON_A_PORT->IN & (1UL << CU_BUTTON_A_PIN)) != 0;   /* active-high */
}

/* Everything dark: colour sinks high (off), slot sources low. Unlike
 * cu_leds_show(0) this also releases the blue rail. */
static void cu_leds_dark(void)
{
    gpio_output_high(NRF_P0, 24);
    gpio_output_high(NRF_P0, 22);
    gpio_output_high(NRF_P1,  0);
    for (int i = 0; i < 8; i++)
        gpio_output_low(cu_slot_led[i].port, cu_slot_led[i].pin);
}

/* Idle fade-out: soft-PWM the shared blue sink (P1.00) down in coarse duty
 * steps - every lit LED fades together since they share that rail. 4 ms
 * frames (250 Hz, no visible flicker) paced by the tick, so no busy-waiting:
 * 75% -> 50% -> 25% over ~600 ms, then dark. */
static void cu_leds_fade_out(void)
{
    for (int duty = 3; duty >= 1; duty--) {
        for (int i = 0; i < 50; i++) {
            gpio_output_low (NRF_P1, 0);              /* blue rail on  */
            vTaskDelay(pdMS_TO_TICKS(duty));
            gpio_output_high(NRF_P1, 0);              /* blue rail off */
            vTaskDelay(pdMS_TO_TICKS(4 - duty));
        }
    }
    cu_leds_dark();
}

/* Discarding CLI session so app_run() works from this task (like the Flipper's
 * gui task). No abort channel here - launcher-run apps run to completion (or are
 * stopped with `kill` over USB/BLE). */
static cli_ctx_t cu_launcher_ctx;
static size_t cu_tp_write(const uint8_t *b, size_t n, void *c) { (void)b; (void)c; return n; }
static size_t cu_tp_read(uint8_t *b, size_t n, void *c) { (void)b; (void)n; (void)c; return 0; }
static bool   cu_tp_connected(void *c) { (void)c; return true; }

static void cu_launcher_task(void *arg)
{
    (void)arg;

    CU_BUTTON_A_PORT->PIN_CNF[CU_BUTTON_A_PIN] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
        (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);   /* keep the GPIOTE PORT wake */

    cu_launcher_ctx.transport.write     = cu_tp_write;
    cu_launcher_ctx.transport.read      = cu_tp_read;
    cu_launcher_ctx.transport.connected = cu_tp_connected;
    cli_bind_ctx(&cu_launcher_ctx);

    int sel = 0;
    bool prev = false, launched = false, leds_dark = false;
    TickType_t press_start = 0;

    for (;;) {
        /* Idle LED policy (platforms/chameleon/power.c drives the flag; this
         * task owns the pins). Fade down when the device goes idle; any
         * activity (button, USB attach, BLE connect) brings them back. */
        bool idle = cu_power_led_idle();
        if (idle && !leds_dark) {
            cu_leds_fade_out();
            leds_dark = true;
        } else if (!idle && leds_dark) {
            leds_dark = false;               /* redrawn below */
        }

        if (!leds_dark) {
            /* LED_8 is always on and doubles as the position-0 marker; slots
             * 1..7 additionally light LED_1..LED_7. So slot 0 = LED_8 only. */
            uint8_t mask = 1u << 7;
            if (sel > 0) mask |= 1u << (sel - 1);
            cu_leds_show(mask);
        }

        bool now = cu_button_a_down();
        TickType_t t = xTaskGetTickCount();

        if (now && leds_dark) {
            /* Wake press: restore the LEDs and consume the press - waking the
             * display must not also advance the launcher selection. (The
             * GPIOTE ISR already noted the activity.) */
            leds_dark = false;
            uint8_t mask = 1u << 7;
            if (sel > 0) mask |= 1u << (sel - 1);
            cu_leds_show(mask);
            while (cu_button_a_down()) vTaskDelay(pdMS_TO_TICKS(20));
            prev = false;
            continue;
        }

        if (now && !prev) { press_start = t; launched = false; }        /* press edge */
        if (now && !launched && (t - press_start) >= pdMS_TO_TICKS(CU_HOLD_MS)) {
            launched = true;                 /* fire once per hold */
            cu_leds_show(0xFF);              /* brief all-on = launching */
            shortcut_run(sel);               /* blocks until the app exits */
            cli_bind_ctx(&cu_launcher_ctx);  /* app_run rebinds the ctx; restore ours */
        }
        if (!now && prev && !launched) sel = (sel + 1) & 7;             /* short press */
        prev = now;

        if (now)
            vTaskDelay(pdMS_TO_TICKS(20));   /* press in progress: poll the hold */
        else
            /* Block until a button edge (GPIOTE PORT) or an idle-policy nudge
             * (power.c wakes the waiters on LED-state transitions); the
             * timeout is only a backstop against a missed nudge. */
            hal_button_wait(5000);
    }
}

void cu_launcher_init(void)
{
    /* pinned absolute (not configMINIMAL * 8); launcher only polls + spawns the app task */
    xTaskCreate(cu_launcher_task, "sclaunch", 1024,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
#endif /* FANTASI_ENABLE_APPS */

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
        /* The FreeRTOS heap spans all app RAM (ucHeap is aliased onto the linker
         * heap region), so free RAM is the free heap - there is no separate
         * unallocated libc arena to add in. */
        out[n].name  = "RAM";
        out[n].total = (uint32_t)&_ram_end - (uint32_t)&_ram_start;
        out[n].free  = (uint32_t)hal_free_heap_bytes();
        out[n].note  = NULL;
        n++;
    }
    return n;
}

int hal_test_regions(hal_test_region_t *out, int max)
{
    (void)out; (void)max;
    /* Nothing to memtest: the RAM region (__heap_start__..__heap_end__) is now
     * the live FreeRTOS heap, and everything below it is either static .bss or
     * the SoftDevice's reservation - none of it can be safely pattern-tested
     * while in use. */
    return 0;
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
    /* Freeze the scheduler first. The launcher task redraws the LEDs every 20 ms
     * (LED_8 is its always-on marker) and would re-light them during the
     * wait-for-release below, leaving LED_8 lit through System OFF since the nRF52
     * retains GPIO output levels. Suspended, pwr_button_task keeps running while
     * no other task does; the button poll and System OFF path below need no
     * scheduler, and System OFF never returns so there is no resume. */
    vTaskSuspendAll();

    /* Blank the slot LEDs. They're driven directly by GPIO and the nRF52 retains
     * pin output levels through System OFF, so without this the boot-time blue
     * (hal_init) would stay lit after "power off". Driving the colour channels
     * (active-low sinks) high turns every slot off. */
    gpio_output_high(NRF_P0, 24);   /* LED_R off */
    gpio_output_high(NRF_P0, 22);   /* LED_G off */
    gpio_output_high(NRF_P1,  0);   /* LED_B off */

    /* Wait for release first: entering System OFF while a button is still
     * held (e.g. the long-hold that triggered this) would latch an immediate
     * wake and loop. Both buttons are armed as wake sources below, so both
     * must be released. */
    while ((NRF_P1->IN & (1UL << CU_BUTTON_B_PIN)) ||
           (NRF_P0->IN & (1UL << 26 /* button A */)))
        ;

    NRF_P1->PIN_CNF[CU_BUTTON_B_PIN] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
        (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);
    NRF_P0->PIN_CNF[26 /* button A */] =
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
            (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
            (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);   /* keep GPIOTE PORT wake */
        cfg = true;
    }
    return (NRF_P1->IN & (1UL << CU_BUTTON_B_PIN)) != 0;
}

/* App-facing button read for the Berry `hardware` module (overrides the weak
 * default in core/app_run.c). The CU has two physical buttons: A -> OK, B ->
 * BACK. Both are active-high with on-die pull-downs; reading IN is
 * non-destructive, so this coexists with the launcher (which owns A, but is
 * blocked in shortcut_run while an app runs) and the power path (which owns B
 * for wake/System-OFF). Pins are configured input+pull-down idempotently in case
 * an app reads them before either owner's init has run. */
uint32_t hal_app_buttons(void)
{
    static bool cfg;
    if (!cfg) {
        CU_BUTTON_A_PORT->PIN_CNF[CU_BUTTON_A_PIN] =
            (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
            (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);
        NRF_P1->PIN_CNF[CU_BUTTON_B_PIN] =
            (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
            (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);
        cfg = true;
    }
    uint32_t m = 0;
    if (CU_BUTTON_A_PORT->IN & (1UL << CU_BUTTON_A_PIN)) m |= FANTASI_BTN_OK;
    if (NRF_P1->IN & (1UL << CU_BUTTON_B_PIN))           m |= FANTASI_BTN_BACK;
    return m;
}
