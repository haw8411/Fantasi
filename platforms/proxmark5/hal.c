/* Fantasi / Proxmark5 (AT32F435, Cortex-M4) - HAL glue.
 *
 * Brings up the OTGFS2 USB device controller (PB14/PB15) and connects it to
 * TinyUSB's DWC2 driver, then hands the read/write/connected/heap surface
 * to the shared hal/tinyusb/hal_serial_tinyusb.c (composite CDC+MSC+HID+
 * vendor, same as Flipper/Chameleon). The RFID FPGA frontend is implemented in
 * platforms/proxmark5/rfid.c (LF + HF read via the GW1N-4B gateware). The
 * ESP32-C2 BLE link is not integrated yet: the hal_ble_* stubs below return
 * sentinels, so scan/pair/radio degrade gracefully at runtime.
 */

#include "at32f435.h"
#include "../../hal/hal.h"
#include "../../hal/hal_rfid.h"
#include "../../hal/hal_name.h"
#include "hal_storage.h"
#include "flash_storage.h"

#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "app_api.h"   /* FANTASI_HID_* / FANTASI_BTN_* bits */
#include "cli.h"
#include "app_run.h"   /* shortcut_run for the button launcher */

#ifdef FANTASI_ENABLE_APPS
void pm5_launcher_init(void);   /* button/LED shortcut launcher, defined below */
#endif

/* Board pin map is in at32f435.h (PM5_* pins). LEDs are open-drain, active-low;
 * the button (PB12) is input, pull-down, active-high. */

void OTGFS2_IRQHandler(void) { tud_int_handler(0); }

void hal_init(void)
{
    /* USB clocks + PHY pins. The 48 MHz OTG clock was routed off the PLL in
     * SystemInit(); here we gate the OTGFS2 peripheral and mux PB14/PB15 to
     * OTG2_D-/D+ (MUX12). The DWC2 driver enables the NVIC line and powers up
     * the internal FS PHY from tud_init(); we only set the priority (must be
     * numerically >= configMAX_SYSCALL so FreeRTOS FromISR calls stay legal). */
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN | CRM_AHBEN1_GPIOCEN | CRM_AHBEN1_OTGFS2EN;
    (void)CRM->AHBEN1;

    gpio_set_mux(GPIOB, 14, MUX_OTGFS2);
    gpio_set_mux(GPIOB, 15, MUX_OTGFS2);
    gpio_set_mode(GPIOB, 14, GPIO_MODE_MUX);
    gpio_set_mode(GPIOB, 15, GPIO_MODE_MUX);

    NVIC_SetPriority(OTGFS2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    /* "Fantasi on" indicator: light the blue RGB LED (I2C 0x48), matching the
     * blue LED_D convention on every other target. The four A-D LEDs are only
     * red/orange, so blue has to come from the separate RGB LED. The orange LEDs
     * are left to the button launcher (off at rest; binary slot display). */
    pm5_rgb_set(0, 0, 200);

    /* Light the two antenna-board LEDs (HF + LF) as a board indicator. */
    pm5_ant_led(true, true);

    /* Park the RF FPGA OFF. It self-loads its bitstream at power-on in a default
     * mode that can leave the antenna field energized - a large idle draw nothing
     * clears until the first RFID op. Force OFF now (also gates its 24 MHz clock). */
    hal_rfid_set_mode(RFID_OFF);

    hal_storage_init();

    /* Apply the USB interface toggles before USB comes up so the device
     * enumerates correctly the first time. Defaults: HID persistent, MSC on. */
    {
        char hv[12] = "persistent";
        hal_settings_get("hid", hv, sizeof(hv));
        hal_hid_set_persistent(hv[0] != 's');   /* "switch" -> non-persistent */

        char mv[4] = "1";
        hal_settings_get("msc", mv, sizeof(mv));
        hal_msc_set_enabled(mv[0] != '0');
    }

#ifdef FANTASI_ENABLE_APPS
    pm5_launcher_init();   /* button/LED shortcut launcher (owns LEDs A-D) */
#endif

    tusb_init();
}

void hal_post_init(void) {}

const char *hal_device_id(void) { return "PM5"; }

const char *hal_device_name(void)
{
    static char name[16];
    if (name[0]) return name;
    /* 96-bit factory unique ID at 0x1FFFF7E8 (RM section 1.3). */
    volatile uint32_t *uid = (volatile uint32_t *)UID_BASE;
    uint32_t u[3] = { uid[0], uid[1], uid[2] };
    hal_name_generate(u, 3, name, sizeof(name));
    return name;
}

extern uint8_t _eflash;

int32_t hal_flash_free_bytes(void)
{
    /* Free app-region flash = start of the storage region minus the end of
     * the image. The app links into bank1 (below STORAGE_BASE). */
    uint32_t used_end = (uint32_t)&_eflash;
    if (STORAGE_BASE <= used_end) return 0;
    return (int32_t)(STORAGE_BASE - used_end);
}

int hal_battery_percent(void) { return -1; }   /* ADC battery sense not implemented yet */

/* Composite USB: MSC/vendor/HID are always present alongside CDC, so no
 * mode switch is ever needed (the hal.h "already composite" contract). */
int hal_enter_msc_mode(void) { return -1; }
int hal_enter_webusb_mode(void) { return -1; }
int hal_enter_cdc_mode(void) { return -1; }

/* ---- USB HID keyboard emulation (all TinyUSB-generic; state lives in the
 * shared hal/tinyusb/usb_descriptors.c) ---- */
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

/* App-facing button read for the Berry `hardware` module (overrides the weak
 * default in core/app_run.c). PM5 button is PB12, input + pull-down, active-high.
 * Configured idempotently. */
uint32_t hal_app_buttons(void)
{
    static bool cfg;
    if (!cfg) {
        CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN;
        gpio_set_pull(GPIOB, PM5_BUTTON_PIN, GPIO_PULL_DOWN);
        gpio_set_mode(GPIOB, PM5_BUTTON_PIN, GPIO_MODE_INPUT);
        cfg = true;
    }
    return (GPIOB->IDT & (1u << PM5_BUTTON_PIN)) ? FANTASI_BTN_OK : 0;   /* active-high */
}

extern uint8_t _ram_start, _ram_end;

int hal_mem_regions(hal_mem_region_t *out, int max)
{
    int n = 0;
    if (n < max) {
        /* The FreeRTOS heap spans all free RAM (ucHeap is aliased onto the linker
         * heap region), so free RAM is the free heap. */
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
    return 0;
}

void hal_radio_info(hal_radio_info_t *info)
{
    __builtin_memset(info, 0, sizeof(*info));
}

void hal_ble_activate_fus(void) {}

/* Restart the firmware in place, keeping PB0 held (a hardware reset powers the
 * board off - it does not survive even on USB). */
void hal_reboot(void)
{
    pm5_deinit_and_jump(PM5_APP_BASE);
}

void hal_set_dfu_magic(void) {}

/* ---- Software DFU: self-repairing DMA holds PB0 through the Artery ROM ----
 * The Artery ROM at 0x1FFF0000 does a full GPIOB peripheral reset when it brings
 * up USB (OTGFS2 D+/D- are PB14/15), which would clear PB0 and release the
 * board's self-latching power supply - the reason a naive jump-to-ROM powers the
 * board off. The fix: a DMA1 channel driven continuously by TMR7 overflow
 * rewrites the entire GPIOB register block from g_gpiob_image_init[] forever - PB0 as
 * a driven-high push-pull output, PB14/15 as OTGFS2 AF - re-establishing PB0
 * within ~1 us of the ROM's reset, with zero CPU involvement, so the latch's
 * hold-up cap bridges the gap. The ROM leaves DMA1/TMR7 running, so PB0 stays
 * held and the ROM DFU (2e3c:df11) enumerates normally. */

/* GPIOB register image (CFGR..MUXH, offsets 0x00..0x24), source of truth in
 * flash. It is copied into SRAM (PM5_DFU_DMA_IMG) before the jump and the DMA
 * reads it from there - never from flash: DFU erases/programs bank1, during which
 * flash reads stall, so a flash source would starve the DMA mid-erase and drop
 * PB0. It also cannot live at the bottom of SRAM, where the Artery ROM places its
 * own .data/.bss; PM5_DFU_DMA_IMG sits mid-SRAM in the ROM's untouched gap. */
static const uint32_t g_gpiob_image_init[10] = {
    0xA0000001u,  /* CFGR : PB0=output(01), PB14/15=MUX(10)  */
    0x00000000u,  /* OMODE: push-pull                        */
    0x50000001u,  /* ODRVR: PB0/PB14/PB15 = large drive      */
    0x00000000u,  /* PULL : none                             */
    0x00000000u,  /* IDT  : read-only, write ignored         */
    0x00000001u,  /* ODT  : PB0 = high                       */
    0x00000000u,  /* SCR                                     */
    0x00000000u,  /* WPR                                     */
    0x00000000u,  /* MUXL : PB0-7 AF=0                        */
    0xCC000000u,  /* MUXH : PB14/PB15 = MUX12 (OTGFS2)        */
};

/* Fixed mid-SRAM landing spot for the live DMA image (see above). */
#define PM5_DFU_DMA_IMG ((volatile uint32_t *)0x20030000u)

static void pm5_start_gpiob_refresh_dma(void)
{
    for (int i = 0; i < 10; i++) PM5_DFU_DMA_IMG[i] = g_gpiob_image_init[i];

    CRM->AHBEN1 |= CRM_AHBEN1_DMA1EN;
    CRM->APB1EN |= CRM_APB1EN_TMR7EN;

    TMR7->CTRL1 = 0;
    TMR7->DIV   = 0;
    TMR7->PR    = 15;              /* fast overflow -> frequent DMA requests */
    TMR7->CVAL  = 0;
    TMR7->IDEN  = TMR_IDEN_OVFDEN;

    dma_channel_type *ch = &DMA1->CH[0];   /* channel 1 */
    ch->CTRL  = 0;
    ch->PADDR = 0x40020400u;               /* GPIOB base */
    ch->MADDR = (uint32_t)PM5_DFU_DMA_IMG;
    ch->DTCNT = 10;
    ch->CTRL  = DMA_CTRL_CHPL_VHI | DMA_CTRL_MWIDTH_32 | DMA_CTRL_PWIDTH_32 |
                DMA_CTRL_MINCM | DMA_CTRL_PINCM | DMA_CTRL_LM | DMA_CTRL_DTD_M2P;

    DMA1->MUXSEL      = DMA_MUXSEL_TBL_SEL;
    DMA1->MUXCCTRL[0] = DMAREQ_TMR7_OVERFLOW;

    ch->CTRL    |= DMA_CTRL_CHEN;
    TMR7->CTRL1 |= TMR_CTRL1_TMREN;
}

void hal_reboot_dfu(void)
{
    /* Full speed for the DFU trick: the TMR7->DMA1 refresh must re-establish PB0
     * within ~1 us of the ROM's GPIOB reset (before the power-latch hold-up cap
     * drains), and that rate scales with HCLK - the idle clock is too slow to
     * hold PB0 and the board drops. No unboost; we don't return. */
    pm5_clk_boost();

    /* Arm the self-repairing PB0 refresh, then jump to the ROM. The DMA must
     * outlive us so PB0 stays held through the ROM's GPIOB reset (see above). */
    pm5_start_gpiob_refresh_dma();

    /* Minimal deinit: mask interrupts + SysTick only. Do not reset or clock-gate
     * DMA1/TMR7/GPIOB/GPIOC or the refresh stops. */
    __disable_irq();
    for (int i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
    SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;

    /* Jump to the Artery ROM (PSP->MSP switch; the ROM sets its own VTOR). */
    __set_CONTROL(0x00);
    __ISB();
    uint32_t sp = *(volatile uint32_t *)(PM5_ROM_BOOTLOADER_BASE + 0);
    uint32_t pc = *(volatile uint32_t *)(PM5_ROM_BOOTLOADER_BASE + 4);
    __set_MSP(sp);
    __DSB(); __ISB();
    __enable_irq();
    ((void (*)(void))(pc | 1u))();
    for (;;);
}

/* The Proxmark5 can cut its own battery rail via the PB0 power lock, but that
 * path is unverified here; report unsupported for now (like the PM3). */
int hal_shutdown(void)
{
    return HAL_SHUTDOWN_UNSUPPORTED;
}

/* ---- BLE: not implemented yet.
 * hal_ble_* is a strong symbol the core command set links against, so they
 * return sentinels rather than being #ifdef'd out. FANTASI_ENABLE_BLE_CLI is
 * left unset, so the blecli task is never created. ---- */
int hal_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms)
{ (void)cb; (void)duration_ms; return -1; }
int hal_ble_pair_setup(uint8_t io_cap)
{ (void)io_cap; return -1; }
void hal_ble_pair_begin(void) {}
void hal_ble_pair_end(void)   {}
int hal_ble_pair_connect(const uint8_t *addr, uint8_t addr_type)
{ (void)addr; (void)addr_type; return -1; }
void hal_ble_shutdown(void) {}
bool hal_ble_is_active(void) { return false; }
int hal_ble_pair_initiate(uint16_t conn_handle)
{ (void)conn_handle; return -1; }
int hal_ble_pair_passkey(uint16_t conn_handle, uint32_t passkey)
{ (void)conn_handle; (void)passkey; return -1; }
int hal_ble_pair_confirm(uint16_t conn_handle, bool accept)
{ (void)conn_handle; (void)accept; return -1; }
int hal_ble_pair_wait(hal_ble_evt_t *evt, uint32_t timeout_ms)
{ (void)evt; (void)timeout_ms; return -1; }
int hal_ble_disconnect(uint16_t conn_handle)
{ (void)conn_handle; return -1; }
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

/* ---- Button shortcut launcher (screenless slot selection) ------------------
 * Mirrors the Proxmark3 launcher. The PM5 has one button (PB12, active-high)
 * and four red/orange LEDs (A-D, open-drain active-low). App shortcuts
 * (scN=<path> in settings.cfg, slots 0-7) are chosen by pressing the button to
 * count up, the slot shown in binary on LEDs A/B/C (A=bit0, B=bit1, C=bit2;
 * slot 0 = all off .. slot 7 = all on). Holding the button >= PM5_HOLD_MS
 * launches the app in the selected slot. The blue RGB LED stays lit throughout
 * as the "Fantasi on" indicator. Drive low = LED on, release high = off. */
#ifdef FANTASI_ENABLE_APPS

/* Button-hold thresholds. A short press cycles the slot. A hold that is released
 * in the launch window [PM5_HOLD_MS, PM5_LAUNCH_MAX_MS) launches the slot. Holding
 * past PM5_LAUNCH_MAX_MS abandons the launch (so a user heading for DFU never
 * fires their shortcut); at PM5_DFU_HOLD_MS the device enters USB DFU. */
#define PM5_HOLD_MS         600     /* min hold to count as a launch (like the PM3) */
#define PM5_LAUNCH_MAX_MS   3000    /* release after this = no launch (DFU-intent hold) */
#define PM5_DFU_HOLD_MS     6000    /* hold this long -> enter DFU */
#define PM5_IDLE_FADE_MS    30000   /* fade all LEDs out after this much inactivity */

static void pm5_led(uint32_t pin, bool on)
{
    if (on) GPIOC->CLR = (1u << pin);   /* drive low    = on  */
    else    GPIOC->SCR = (1u << pin);   /* release high = off */
}

static void pm5_leds_show(uint8_t slot)
{
    pm5_led(PM5_LED_A_PIN, slot & 1);
    pm5_led(PM5_LED_B_PIN, slot & 2);
    pm5_led(PM5_LED_C_PIN, slot & 4);
    pm5_led(PM5_LED_D_PIN, false);      /* D stays off; blue is the RGB LED */
}

/* ---- Idle LED fade (all 7 LEDs) ----
 * Dim all seven like the CU/FZ. The blue RGB (I2C 0x48) has a brightness
 * register and fades in hardware; the A/B/C (GPIOC) and HF/LF antenna pair
 * (I2C 0x51) are on/off only and PWM'd. The PWM uses a short busy-wait, not
 * vTaskDelay (1 ms grain caps at ~50 Hz, visibly flickering), to reach ~150 Hz.
 * Busy-waiting at idle is safe here: no watchdog or power-lock refresh to
 * starve. ~600 ms envelope. */
static void pm5_udelay(uint32_t us)
{
    for (volatile uint32_t n = us * 48u; n; n--) { __asm volatile("nop"); }
}

/* PWM on/off time for a perceptually-even fade. Eye response is ~logarithmic,
 * so the level is squared (~gamma-2) to spend most of the fade in the low-duty
 * zone where dimming is visible. `pk` is the peak on-time (us) at full
 * brightness; the period is held ~constant to keep the PWM frequency fixed. */
#define PM5_FADE_PERIOD_US 2600u
static void pm5_pwm_step(uint32_t lvl, uint32_t n, uint32_t pk,
                         void (*set)(bool on, uint8_t sel), uint8_t sel, int reps)
{
    uint32_t on = pk * lvl * lvl / (n * n);          /* gamma ~2 */
    uint32_t off = (on < PM5_FADE_PERIOD_US) ? PM5_FADE_PERIOD_US - on : 0u;
    for (int p = 0; p < reps; p++) {
        set(true, sel);  pm5_udelay(on);
        set(false, sel); pm5_udelay(off);
    }
}
static void pm5_set_ad(bool on, uint8_t sel)  { pm5_leds_show(on ? sel : 0); }

static void pm5_leds_fade_out(uint8_t sel)
{
    /* Two phases. Separating them keeps the blue's slow I2C write from chopping
     * the antenna PWM into ~40 Hz modulation.
     *   Phase 1 - blue (hardware brightness ramp) + A/B/C (GPIO PWM).
     *   Phase 2 - HF/LF antenna PWMs down continuously (~300 Hz), gamma-shaped. */
    const int N = 32;
    for (int lvl = N; lvl >= 0; lvl--) {                 /* phase 1: blue + A-D */
        pm5_rgb_set(0, 0, (uint8_t)(200 * lvl * lvl / (N * N)));
        pm5_pwm_step((uint32_t)lvl, N, 2500u, pm5_set_ad, sel, 4);
    }
    /* Phase 2 - antenna. Fast I2C (~240 kHz) shrinks the toggle; the PWM period
     * grows as it dims (2.5 ms -> ~10 ms) so the fixed ~130 us write becomes a
     * shrinking duty %, reaching ~1% (near black) before the final off. Low
     * frequency at the dim end is invisible. On-time is gamma-shaped. */
    pm5_i2c_set_fast(true);
    for (int lvl = N; lvl >= 0; lvl--) {
        uint32_t P  = 2500u + (uint32_t)(N - lvl) * 240u;        /* period grows */
        uint32_t on = (uint32_t)((uint64_t)P * lvl * lvl / (N * N)); /* gamma ~2 */
        uint32_t off = (on < P) ? P - on : 0u;
        for (int p = 0; p < 3; p++) {
            pm5_ant_led(true, true);  pm5_udelay(on);
            pm5_ant_led(false, false); pm5_udelay(off);
        }
    }
    pm5_i2c_set_fast(false);

    pm5_ant_led(false, false);
    pm5_leds_show(0);
    pm5_rgb_set(0, 0, 0);
}

static void pm5_leds_restore(uint8_t sel)
{
    /* Re-light the blue. The RGB controller is a separate slow MCU; after the
     * fade drives it to 0 a single write can silently fail (it ACKs the address
     * but count/data don't land), so push the full sequence a few times. */
    for (int i = 0; i < 4; i++) {
        pm5_rgb_set(0, 0, 200);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    pm5_ant_led(true, true);
    pm5_leds_show(sel);
}

/* Last user/host activity (button, CLI command, proto frame). hal_power_activity
 * overrides the weak core stub; the launcher uses it for the idle-fade timer. */
static volatile TickType_t s_pm5_last_activity;
void hal_power_activity(void) { s_pm5_last_activity = xTaskGetTickCount(); }

static bool pm5_button_down(void)
{
    return (GPIOB->IDT & (1u << PM5_BUTTON_PIN)) != 0;   /* active-high */
}

/* Discarding CLI session so app_run() works from this task; the app itself runs
 * on the "app" task app_run() spawns. Screenless: console output goes nowhere. */
static cli_ctx_t pm5_launcher_ctx;
static size_t pm5_tp_write(const uint8_t *b, size_t n, void *c) { (void)b; (void)c; return n; }
static size_t pm5_tp_read(uint8_t *b, size_t n, void *c) { (void)b; (void)n; (void)c; return 0; }
static bool   pm5_tp_connected(void *c) { (void)c; return true; }

static void pm5_launch(int slot)
{
    shortcut_run(slot);
    cli_bind_ctx(&pm5_launcher_ctx);   /* app_run rebinds the ctx; restore ours */
}

static void pm5_launcher_task(void *arg)
{
    (void)arg;

    /* LEDs A-D as open-drain outputs (all off); button input + pull-down. */
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN | CRM_AHBEN1_GPIOCEN;
    (void)CRM->AHBEN1;
    const uint32_t leds[4] = { PM5_LED_A_PIN, PM5_LED_B_PIN, PM5_LED_C_PIN, PM5_LED_D_PIN };
    for (int i = 0; i < 4; i++) {
        GPIOC->SCR = (1u << leds[i]);                    /* off */
        gpio_set_otype(GPIOC, leds[i], GPIO_OTYPE_OD);
        gpio_set_mode(GPIOC, leds[i], GPIO_MODE_OUTPUT);
    }
    gpio_set_pull(GPIOB, PM5_BUTTON_PIN, GPIO_PULL_DOWN);
    gpio_set_mode(GPIOB, PM5_BUTTON_PIN, GPIO_MODE_INPUT);

    pm5_launcher_ctx.transport.write     = pm5_tp_write;
    pm5_launcher_ctx.transport.read      = pm5_tp_read;
    pm5_launcher_ctx.transport.connected = pm5_tp_connected;
    cli_bind_ctx(&pm5_launcher_ctx);

    /* Light the blue "Fantasi on" RGB LED. Its controller MCU is ready a short
     * time after reset, so retry (~1 s) until the write ACKs. */
    for (int i = 0; i < 40 && !pm5_rgb_set(0, 0, 200); i++) vTaskDelay(pdMS_TO_TICKS(25));

    int sel = 0;
    bool prev = false;
    bool faded = false;
    TickType_t press_start = 0;
    s_pm5_last_activity = xTaskGetTickCount();

    /* Re-park the RF FPGA now that it has finished self-loading its bitstream.
     * The early park in hal_init() can fire during the GW1N's config load and be
     * ignored, leaving the carrier driving the antenna (a heat source `field
     * status` hides - it reports OFF while the hardware field is on). Spaced
     * retries guarantee one lands after the FPGA is ready. */
    for (int i = 0; i < 4; i++) {
        vTaskDelay(pdMS_TO_TICKS(150));
        hal_rfid_set_mode(RFID_OFF);
    }

    for (;;) {
        bool now = pm5_button_down();
        TickType_t t = xTaskGetTickCount();

        hal_rfid_field_tick();   /* auto-park the RF carrier if left on + idle */

        if (now) s_pm5_last_activity = t;   /* button counts as activity */

        /* Idle-fade policy: after 30 s with no button or host activity, fade
         * all seven LEDs out; any activity restores them. */
        bool idle = (uint32_t)(t - s_pm5_last_activity) >= pdMS_TO_TICKS(PM5_IDLE_FADE_MS);
        if (idle && !faded) {
            /* The fade's PWM timing is busy-wait (pm5_udelay) calibrated for
             * 288 MHz; boost so it runs at speed even though we idle at 48. */
            pm5_clk_boost();
            pm5_leds_fade_out((uint8_t)sel);
            pm5_clk_unboost();
            faded = true;
        } else if (!idle && faded) {
            /* Woken (button/host): restore, and if a button press did the
             * waking, consume it so it doesn't also advance the slot. */
            faded = false;
            pm5_clk_boost();
            pm5_leds_restore((uint8_t)sel);
            pm5_clk_unboost();
            while (pm5_button_down()) vTaskDelay(pdMS_TO_TICKS(20));
            prev = false;
            continue;
        }
        if (faded) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }   /* dark: poll slowly */

        if (now && !prev) press_start = t;                             /* press edge */

        if (now) {
            TickType_t dt = t - press_start;
            /* Decide by how long the button has been held, not at a single mark,
             * so the launch can be withheld for a DFU-intent long hold. */
            if (dt >= pdMS_TO_TICKS(PM5_DFU_HOLD_MS)) {
                pm5_leds_show(0);                                       /* the ROM resets GPIOC next */
                hal_reboot_dfu();                                      /* does not return */
            } else if (dt >= pdMS_TO_TICKS(PM5_LAUNCH_MAX_MS)) {
                /* Past the launch window: only D lit = "keep holding for DFU". */
                pm5_led(PM5_LED_A_PIN, false);
                pm5_led(PM5_LED_B_PIN, false);
                pm5_led(PM5_LED_C_PIN, false);
                pm5_led(PM5_LED_D_PIN, true);
            } else if (dt >= pdMS_TO_TICKS(PM5_HOLD_MS)) {
                pm5_leds_show(7);                                      /* armed: release to launch */
            } else {
                pm5_leds_show((uint8_t)sel);
            }
        } else {
            if (prev) {                                               /* release edge */
                TickType_t dt = t - press_start;
                if (dt >= pdMS_TO_TICKS(PM5_HOLD_MS) && dt < pdMS_TO_TICKS(PM5_LAUNCH_MAX_MS)) {
                    pm5_leds_show(7);
                    pm5_launch(sel);                                  /* launch on release */
                } else if (dt < pdMS_TO_TICKS(PM5_HOLD_MS)) {
                    sel = (sel + 1) & 7;                              /* short press -> next slot */
                }
                /* released in [LAUNCH_MAX, DFU): abandoned, do nothing */
            }
            pm5_leds_show((uint8_t)sel);
        }
        prev = now;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void pm5_launcher_init(void)
{
    /* 4 KB stack hosts app_run()'s ELF-load path (pm5_launch); the app runs on
     * the "app" task app_run() spawns. PM5 has 384 KB RAM, so this is roomy. */
    xTaskCreate(pm5_launcher_task, "sclaunch", configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
#endif /* FANTASI_ENABLE_APPS */
