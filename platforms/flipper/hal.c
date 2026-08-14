/* Fantasi / Flipper Zero - HAL glue.
 *
 * hal_init() finishes USB bring-up (GPIO muxing, peripheral clock, NVIC)
 * and calls tusb_init(). The read/write/connected/heap entry points live
 * in hal/tinyusb/hal_serial_tinyusb.c.
 *
 * The two STM32WB55 USB IRQs both fan into tud_int_handler(0); TinyUSB
 * only cares that its handler gets called whenever the hardware asserts.
 */

#include "stm32wbxx.h"

#include "../../hal/hal.h"
#include "../../hal/hal_name.h"
#include "ble.h"
#include "display.h"
#include "hal_storage.h"
#include "flash_storage.h"
#include "lfs.h"
#include "tusb.h"

#include "app_api.h"   /* FANTASI_BTN_* bits for hal_app_buttons / gui events */
#ifdef FANTASI_ENABLE_GUI
#include "gui.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>


/* ---- I2C1 helpers (PA9=SCL, PA10=SDA) ---- */

#define LP5562_ADDR       0x60U  /* 8-bit address */
#define BQ27220_ADDR      0xAAU  /* 8-bit address */
/* 100 kHz standard-mode timing with I2CCLK = PCLK1 = 16 MHz.
 * PRESC=3 → 4 MHz prescaled clock, SCLL=19, SCLH=15,
 * SDADEL=2, SCLDEL=4. */
#define I2C_TIMING_16M_100K  0x30420F13U

#define I2C_WAIT(cond) do { \
    for (volatile int _t = 0; _t < 50000; _t++) \
        if (cond) break; \
} while (0)

static void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    I2C1->CR2 = addr
              | (2U << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_AUTOEND_Msk
              | I2C_CR2_START_Msk;

    I2C_WAIT(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF));
    if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return; }
    I2C1->TXDR = reg;

    I2C_WAIT(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF));
    if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return; }
    I2C1->TXDR = val;

    I2C_WAIT(I2C1->ISR & I2C_ISR_STOPF);
    I2C1->ICR = I2C_ICR_STOPCF;
}

static uint16_t i2c_read_reg16(uint8_t addr, uint8_t reg)
{
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

    /* Write phase: send register address, no AUTOEND (need repeated START). */
    I2C1->CR2 = addr
              | (1U << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START_Msk;

    I2C_WAIT(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF));
    if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return 0; }
    I2C1->TXDR = reg;

    I2C_WAIT(I2C1->ISR & I2C_ISR_TC);

    /* Read phase: repeated START, read 2 bytes, AUTOEND. */
    I2C1->CR2 = addr
              | I2C_CR2_RD_WRN_Msk
              | (2U << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_AUTOEND_Msk
              | I2C_CR2_START_Msk;

    I2C_WAIT(I2C1->ISR & I2C_ISR_RXNE);
    uint8_t lo = (uint8_t)I2C1->RXDR;

    I2C_WAIT(I2C1->ISR & I2C_ISR_RXNE);
    uint8_t hi = (uint8_t)I2C1->RXDR;

    I2C_WAIT(I2C1->ISR & I2C_ISR_STOPF);
    I2C1->ICR = I2C_ICR_STOPCF;

    return (uint16_t)hi << 8 | lo;
}

static void lp5562_init(void)
{
    i2c_write_reg(LP5562_ADDR, 0x0D, 0xFF);
    for (volatile int i = 0; i < 160000; i++) {}

    i2c_write_reg(LP5562_ADDR, 0x00, 0xC0);
    for (volatile int i = 0; i < 8000; i++) {}

    i2c_write_reg(LP5562_ADDR, 0x08, 0x61);
    i2c_write_reg(LP5562_ADDR, 0x70, 0x00);
    i2c_write_reg(LP5562_ADDR, 0x05, 50);
    i2c_write_reg(LP5562_ADDR, 0x02, 0xFF);
    i2c_write_reg(LP5562_ADDR, 0x0F, 150);
    i2c_write_reg(LP5562_ADDR, 0x0E, 0xFF);
}

static TaskHandle_t bl_task_handle;
static void backlight_task(void *arg);

static uint8_t splash_buf[DISPLAY_PAGES * DISPLAY_WIDTH];
static bool splash_loaded;

/* >0 while a loadable app and/or the GUI menu owns the screen: backlight_task
 * then skips its periodic status/splash redraw so it can't paint over them.
 * A counter, not a flag, because holds nest - the GUI keeps the display for
 * as long as its menu is open, and app_run acquires again on top of that for
 * an app launched from the menu (its release must NOT repaint the splash
 * then; the menu redraws itself). */
static volatile int display_owners;

static void display_refresh(void)
{
    display_lock();
    /* Re-check ownership under the lock: a caller may have seen owners==0
     * while the GUI was acquiring - without this, the splash/battery would
     * paint right over a freshly drawn menu. */
    if (display_owners != 0) {
        display_unlock();
        return;
    }

    if (splash_loaded)
        display_blit(splash_buf, sizeof(splash_buf));
    else {
        display_clear();
        display_print(43, 3, "fantasi");
    }

    int pct = hal_battery_percent();
    if (pct >= 0) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", pct);
        int len = 0;
        while (buf[len]) len++;
        display_print_inv(DISPLAY_WIDTH - len * 6, 0, buf);
    }

#ifdef FANTASI_ENABLE_GUI
    /* Menu hint: rounded button hugging the bottom edge (bottom border 1px
     * above the panel edge). Only ever lands on the splash - while the menu
     * or an app holds the display this whole repaint is skipped. */
    int bh = font_item.ascent + font_item.descent + 4;   /* display_button height */
    display_button(DISPLAY_WIDTH / 2, DISPLAY_HEIGHT - 1 - bh, DISPLAY_CHAR_UP " Menu");
#endif

    display_flush();
    display_unlock();
}

#ifdef FANTASI_ENABLE_APPS
/* app_run.c (and the GUI menu) bracket screen use with these so our periodic
 * redraw yields the screen; the splash returns when the LAST holder lets go. */
void hal_app_display_acquire(void)
{
    taskENTER_CRITICAL();
    display_owners++;
    taskEXIT_CRITICAL();
}

void hal_app_display_release(void)
{
    bool restore;
    taskENTER_CRITICAL();
    restore = (--display_owners == 0);
    taskEXIT_CRITICAL();
    if (restore)
        display_refresh();   /* restore the normal screen immediately */
}
#endif

void hal_init(void)
{
    /* Route PA11/PA12 to the USB transceiver. Despite the "dedicated
     * USB pins" wording in ST's datasheet, PA11 (USB_DM) and PA12
     * (USB_DP) are still GPIO muxes that default to analog after
     * reset - if AF10 isn't selected, the transceiver's D+/D- drive
     * never reaches the physical pins. Must happen before tusb_init(). */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE12))
                 | (2U << GPIO_MODER_MODE11_Pos)      /* AF */
                 | (2U << GPIO_MODER_MODE12_Pos);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFU << ((11 - 8) * 4)) | (0xFU << ((12 - 8) * 4))))
                  | (10U << ((11 - 8) * 4))           /* AF10 = USB */
                  | (10U << ((12 - 8) * 4));
    GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED11_Pos)
                   |  (3U << GPIO_OSPEEDR_OSPEED12_Pos);

    /* PA3 = periph_power: open-drain, drive HIGH to turn on the
     * peripheral 3V3 rail that feeds the LP5562 LED driver. */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE3)
                 | (1U << GPIO_MODER_MODE3_Pos);          /* output */
    GPIOA->OTYPER |= (1U << 3);                           /* open-drain */
    GPIOA->BSRR = (1U << 3);                              /* set HIGH */
    for (volatile int i = 0; i < 800000; i++) {}          /* ~50 ms for LP5562 power-on */

    /* PA9 (I2C1_SCL) and PA10 (I2C1_SDA): AF4, open-drain. */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10))
                 | (2U << GPIO_MODER_MODE9_Pos)
                 | (2U << GPIO_MODER_MODE10_Pos);
    GPIOA->OTYPER |= (1U << 9) | (1U << 10);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFU << ((9 - 8) * 4)) | (0xFU << ((10 - 8) * 4))))
                  | (4U << ((9 - 8) * 4))
                  | (4U << ((10 - 8) * 4));

    /* I2C1 peripheral clock, reset, and init. */
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    (void)RCC->APB1ENR1;
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
    I2C1->TIMINGR = I2C_TIMING_16M_100K;
    I2C1->CR1     = I2C_CR1_PE;

    lp5562_init();

    /* USB peripheral clock. SYSCLK/CLK48 were set up in SystemInit(). */
    RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;
    (void)RCC->APB1ENR1;

    /* Pass the raw priority level to NVIC_SetPriority - CMSIS shifts
     * it up by (8 - __NVIC_PRIO_BITS) internally. Pre-shifting here
     * would double-shift to 0 (highest prio) and trip FreeRTOS's
     * vPortValidateInterruptPriority assert the moment any USB IRQ
     * calls a FromISR API. */
    const uint32_t irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
    NVIC_SetPriority(USB_LP_IRQn, irq_prio);
    NVIC_SetPriority(USB_HP_IRQn, irq_prio);
    NVIC_EnableIRQ(USB_LP_IRQn);
    NVIC_EnableIRQ(USB_HP_IRQn);

    hal_storage_init();

    /* Apply the USB interface toggles before USB comes up so the device
     * enumerates correctly the first time (no boot re-enumeration). Defaults
     * (missing keys): HID persistent, MSC on. */
    {
        char hv[12] = "persistent";
        hal_settings_get("hid", hv, sizeof(hv));
        hal_hid_set_persistent(strcmp(hv, "switch") != 0);

        char mv[4] = "1";
        hal_settings_get("msc", mv, sizeof(mv));
        hal_msc_set_enabled(mv[0] != '0');
    }

    tusb_init();

    display_init();

    /* Apply the persisted LCD contrast (ST7565 electronic volume). Absent key
     * keeps display_init()'s built-in default, so a stock config is unchanged. */
    {
        char cv[8];
        if (hal_settings_get("contrast", cv, sizeof(cv)) > 0) {
            int ev = atoi(cv);
            if (ev < DISPLAY_CONTRAST_MIN) ev = DISPLAY_CONTRAST_MIN;
            if (ev > DISPLAY_CONTRAST_MAX) ev = DISPLAY_CONTRAST_MAX;
            display_set_contrast((uint8_t)ev);
        }
    }

    splash_loaded = hal_storage_read_file("/splash.bin", splash_buf,
                                          sizeof(splash_buf)) == (int)sizeof(splash_buf);
    display_refresh();

    xTaskCreate(backlight_task, "bl", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, &bl_task_handle);

#ifdef FANTASI_ENABLE_GUI
    /* Menu task + its button-event queue. Must run before buttons_init()
     * unmasks the EXTI lines (the ISRs forward presses into that queue). */
    gui_init();
#endif
}

void hal_post_init(void)
{
    char v[4] = "1";
    hal_settings_get("ble", v, sizeof(v));
    if (v[0] == '0') return;
    ble_init();
}

void USB_LP_IRQHandler(void) { tud_int_handler(0); }
void USB_HP_IRQHandler(void) { tud_int_handler(0); }

/* Baked in by the platform Makefile: "FZ" for the Flipper, "KIISU" for the
 * pin-identical Kiisu build that reuses this HAL. */
#ifndef FANTASI_DEVICE_ID
#define FANTASI_DEVICE_ID "FZ"
#endif
const char *hal_device_id(void) { return FANTASI_DEVICE_ID; }

const char *hal_device_name(void)
{
    static char name[16];
    if (name[0]) return name;
    volatile uint32_t *uid = (volatile uint32_t *)0x1FFF7590;
    uint32_t u[3] = { uid[0], uid[1], uid[2] };
    hal_name_generate(u, 3, name, sizeof(name));
    return name;
}

extern uint8_t _eflash;

int32_t hal_flash_free_bytes(void)
{
    /* Free program flash = the slack below the LittleFS storage region, not up to the
     * BLE secure boundary. The 256 KB LittleFS is placed in the last 256 KB before SFSA
     * and is the "/" mount reported separately in df, so the ceiling is its base, not
     * secure_start. */
    uint32_t used_end = (uint32_t)&_eflash;
    uint32_t base = storage_flash_base();
    if (!base || base <= used_end) return 0;
    return (int32_t)(base - used_end);
}

int hal_battery_percent(void)
{
    uint16_t soc = i2c_read_reg16(BQ27220_ADDR, 0x2C);
    if (soc > 100) return -1;
    return (int)soc;
}

void hal_reboot(void)
{
    NVIC_SystemReset();
    for (;;);
}

/* Request the ST system bootloader on the next boot. The actual jump
 * happens from Reset_Handler (see startup.c): we can't jump from here
 * because once CPU2's secure world and our own peripherals are running,
 * the bootloader ROM at 0x1FFF0000 is inaccessible to normal-world
 * code. Setting the retained-SRAM magic and resetting is how the stock
 * Flipper fw handles it too. */
extern volatile uint32_t g_fantasi_dfu_magic;
#define FANTASI_DFU_MAGIC 0xD0F0FADAUL

void hal_radio_info(hal_radio_info_t *info)
{
    __builtin_memset(info, 0, sizeof(*info));
    info->available = true;

    uint32_t sfsa = (FLASH->SFR & FLASH_SFR_SFSA_Msk) >> FLASH_SFR_SFSA_Pos;
    info->secure_flash_start = FLASH_BASE + sfsa * 4096U;
    info->secure_flash_kb = (sfsa < 256U) ? (256U - sfsa) * 4U : 0U;

    /* IPCC communication with CPU2 requires SRAM2A access, which may
     * be secured by the running wireless stack. For now, report only
     * SFSA-derived info. Use scripts/radio_flash.py for stack management. */
}


static hal_ble_scan_cb_t active_scan_cb;

static void scan_bridge(const ble_scan_result_t *r)
{
    if (active_scan_cb)
        active_scan_cb(r->addr, r->addr_type, r->rssi, r->name);
}

int hal_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms)
{
    active_scan_cb = cb;
    int ret = ble_scan(scan_bridge, duration_ms);
    active_scan_cb = NULL;
    return ret;
}

int hal_ble_pair_setup(uint8_t io_cap)
{
    return ble_pair_setup_security(io_cap);
}

void hal_ble_pair_begin(void) { ble_pair_set_manual(true); }
void hal_ble_pair_end(void)   { ble_pair_set_manual(false); }

int hal_ble_pair_connect(const uint8_t *addr, uint8_t addr_type)
{
    return ble_pair_connect(addr, addr_type);
}

void hal_ble_shutdown(void)    { ble_shutdown(); }
void hal_ble_activate_fus(void) { ble_activate_fus(); }
bool hal_ble_is_active(void) { return ble_is_active(); }

int hal_ble_pair_initiate(uint16_t conn_handle)
{
    return ble_pair_initiate(conn_handle);
}

int hal_ble_pair_passkey(uint16_t conn_handle, uint32_t passkey)
{
    return ble_pair_send_passkey(conn_handle, passkey);
}

int hal_ble_pair_confirm(uint16_t conn_handle, bool accept)
{
    return ble_pair_numeric_confirm(conn_handle, accept);
}

int hal_ble_pair_wait(hal_ble_evt_t *evt, uint32_t timeout_ms)
{
    ble_event_t e;
    if (ble_pair_wait_event(&e, timeout_ms) != 0)
        return -1;
    evt->type        = (hal_ble_evt_type_t)e.type;
    evt->conn_handle = e.conn_handle;
    evt->status      = e.status;
    evt->reason      = e.reason;
    evt->passkey     = e.passkey;
    return 0;
}

int hal_ble_disconnect(uint16_t conn_handle)
{
    return ble_pair_disconnect(conn_handle);
}

uint32_t hal_ble_generate_passkey(void)
{
    return ble_generate_passkey();
}

int hal_ble_connections(hal_ble_conn_info_t *out, int max)
{
    ble_conn_info_t raw[BLE_MAX_CONN];
    int n = ble_get_connections(raw, max < BLE_MAX_CONN ? max : BLE_MAX_CONN);
    for (int i = 0; i < n; i++) {
        out[i].handle    = raw[i].handle;
        out[i].addr_type = raw[i].addr_type;
        __builtin_memcpy(out[i].addr, raw[i].addr, 6);
    }
    return n;
}

int hal_ble_get_bonded(hal_ble_bonded_t *out, int max)
{
    ble_bonded_dev_t raw[8];
    int lim = max < 8 ? max : 8;
    int n = ble_get_bonded_devices(raw, lim);
    for (int i = 0; i < n; i++) {
        out[i].addr_type = raw[i].addr_type;
        __builtin_memcpy(out[i].addr, raw[i].addr, 6);
    }
    return n;
}

int hal_ble_remove_bond(const uint8_t *addr, uint8_t addr_type)
{
    return ble_remove_bond(addr, addr_type);
}

int hal_ble_clear_bonds(void)
{
    return ble_clear_all_bonds();
}

int hal_enter_msc_mode(void) { return -1; }

/* Composite: the vendor/WebUSB interface is always present alongside CDC. */
int hal_enter_webusb_mode(void) { return -1; }
int hal_enter_cdc_mode(void) { return -1; }

/* ---- USB HID keyboard emulation ----
 * Two modes, selected by the `hid` setting (state in usb_descriptors.c):
 *   persistent (default): the keyboard is always enumerated with CDC/MSC/vendor,
 *     so nothing re-enumerates - enable is a readiness wait, disable releases
 *     held keys.
 *   switch: the keyboard appears only while armed, so enable/disable each
 *     re-enumerate the bus (CDC blips). Stealthier at rest; opt-in per device. */
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
        /* Switch mode: add/remove the interface and re-enumerate. */
        if ((bool)on != usb_desc_hid_active()) {
            if (!on) hid_release_keys();
            usb_desc_set_hid_active(on);
            hal_usb_reenumerate();
        }
        if (!on) return 0;
    }

    /* Wait for a host to have (re)enumerated and mounted the keyboard. */
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

    /* Wait for the previous report to drain off the interrupt IN endpoint. */
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

/* Apply the persistent/switch mode. In switch mode the keyboard must start
 * absent (armed only on demand); persistent keeps it present. Called at boot
 * (before enumeration) and on a live settings change. */
void hal_hid_set_persistent(bool persistent)
{
    usb_desc_set_hid_persistent(persistent);
    if (!persistent) usb_desc_set_hid_active(false);
}

void hal_msc_set_enabled(bool enabled)
{
    usb_desc_set_msc_enabled(enabled);
}

/* ---- memory region reporting ---- */

extern uint8_t _ram1_start, _ram1_end;
extern uint8_t __heap_start__, __heap_end__;
extern uint8_t _sram2a_start, _sram2a_end, _sram2a_used_end;

/* SRAM2 partitioning with the full extended BLE stack (SBRSA=1):
 *   SRAM2a  32 KB : 1 KB non-secure (ref table + MB_MEM1), 31 KB CPU2
 *   SRAM2b  32 KB : 10 KB shared (mailbox zone), 22 KB CPU2 private
 *
 * Our MB_MEM2 buffers (event pool, cmd bufs, ~2.4 KB) live in SRAM1,
 * not SRAM2, because SRAM2a's 1 KB non-secure zone is too small.
 * CPU2 accesses them via RCC_C2AHB1ENR_SRAM1EN (see ble.c).
 *
 * The 10 KB SRAM2b shared zone is where the original firmware places
 * its MB_MEM1/MB_MEM2; since we keep ours in SRAM2a + SRAM1, that
 * zone is available to the application. */
#define SRAM2A_SHARED   (1U * 1024)     /* 0x20030000-0x200303FF (SBRSA=1) */
#define SRAM2B_BASE     0x20038000U
#define SRAM2B_SIZE     (32U * 1024)
#define SRAM2B_SHARED   (10U * 1024)    /* 0x20038000-0x2003A7FF */

int hal_mem_regions(hal_mem_region_t *out, int max)
{
    int n = 0;

    if (n < max) {
        /* The FreeRTOS heap spans all free SRAM1 (ucHeap is aliased onto the
         * linker heap region), so free SRAM1 is the free heap - there is no
         * separate unallocated libc arena to add in. */
        out[n].name  = "SRAM1";
        out[n].total = (uint32_t)&_ram1_end - (uint32_t)&_ram1_start;
        out[n].free  = (uint32_t)hal_free_heap_bytes();
        out[n].note  = NULL;
        n++;
    }

    if (n < max) {
        out[n].name  = "SRAM2a";
        out[n].total = (uint32_t)&_sram2a_end - (uint32_t)&_sram2a_start;
        out[n].free  = 0;
        out[n].note  = "1 KB shared, 31 KB CPU2";
        n++;
    }

    if (n < max) {
        out[n].name  = "SRAM2b";
        out[n].total = SRAM2B_SIZE;
        out[n].free  = SRAM2B_SHARED;
        out[n].note  = "10 KB shared, 22 KB CPU2";
        n++;
    }

    return n;
}

int hal_test_regions(hal_test_region_t *out, int max)
{
    int n = 0;

    /* SRAM1: the heap region (__heap_start__..__heap_end__) contains live
     * FreeRTOS allocations, so writing test patterns over it would corrupt
     * them. Only the idle SRAM2b shared zone is tested. */
    if (n < max) {
        out[n].name = "SRAM2b";
        out[n].addr = SRAM2B_BASE;
        out[n].size = SRAM2B_SHARED;
        n++;
    }

    return n;
}

/* ---- backlight timeout (60 s) with button wake ---- */

#define BACKLIGHT_TIMEOUT_MS  60000
#define DISPLAY_REFRESH_MS    30000
#define LP5562_REG_W_PWM     0x0E

/* Button pins:
 *   UP=PB10  DOWN=PC6  LEFT=PB11  RIGHT=PB12  BACK=PC13  OK=PH3
 * All active-LOW with pull-up except OK which is active-HIGH pull-down. */
#define BTN_EXTI_MASK  ((1U<<3)|(1U<<6)|(1U<<10)|(1U<<11)|(1U<<12)|(1U<<13))

static void backlight_set(uint8_t val)
{
    i2c_write_reg(LP5562_ADDR, LP5562_REG_W_PWM, val);
}

static void buttons_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
    (void)RCC->AHB2ENR;

    /* PB10, PB11, PB12: input, pull-up (MODER=00 is reset default). */
    GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
    GPIOB->PUPDR = (GPIOB->PUPDR
                    & ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12))
                 | (1U << GPIO_PUPDR_PUPD10_Pos)
                 | (1U << GPIO_PUPDR_PUPD11_Pos)
                 | (1U << GPIO_PUPDR_PUPD12_Pos);

    /* PC6, PC13: input, pull-up. */
    GPIOC->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE13);
    GPIOC->PUPDR = (GPIOC->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD13))
                 | (1U << GPIO_PUPDR_PUPD6_Pos)
                 | (1U << GPIO_PUPDR_PUPD13_Pos);

    /* PH3: input, pull-down. */
    GPIOH->MODER &= ~GPIO_MODER_MODE3;
    GPIOH->PUPDR = (GPIOH->PUPDR & ~GPIO_PUPDR_PUPD3)
                 | (2U << GPIO_PUPDR_PUPD3_Pos);

    /* SYSCFG EXTI routing (SYSCFG is always-on on STM32WB). */
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI3)
                       | SYSCFG_EXTICR1_EXTI3_PH;
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~SYSCFG_EXTICR2_EXTI6)
                       | SYSCFG_EXTICR2_EXTI6_PC;
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(SYSCFG_EXTICR3_EXTI10 | SYSCFG_EXTICR3_EXTI11))
                       | SYSCFG_EXTICR3_EXTI10_PB | SYSCFG_EXTICR3_EXTI11_PB;
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~(SYSCFG_EXTICR4_EXTI12 | SYSCFG_EXTICR4_EXTI13))
                       | SYSCFG_EXTICR4_EXTI12_PB | SYSCFG_EXTICR4_EXTI13_PC;

    /* Falling edge for active-LOW buttons, rising edge for OK. */
    EXTI->FTSR1 |= (1U<<6)|(1U<<10)|(1U<<11)|(1U<<12)|(1U<<13);
    EXTI->RTSR1 |= (1U<<3);

    EXTI->IMR1 |= BTN_EXTI_MASK;

    const uint32_t prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
    NVIC_SetPriority(EXTI3_IRQn, prio);
    NVIC_SetPriority(EXTI9_5_IRQn, prio);
    NVIC_SetPriority(EXTI15_10_IRQn, prio);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Read by the button ISRs: while the backlight is off, a press only wakes the
 * screen - it is not forwarded to the GUI as input. */
static volatile bool bl_lit = true;

static void backlight_task(void *arg)
{
    (void)arg;
    buttons_init();

    TickType_t off_at = xTaskGetTickCount() + pdMS_TO_TICKS(BACKLIGHT_TIMEOUT_MS);

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait;

        if (bl_lit) {
            TickType_t remaining = off_at - now;
            if ((int32_t)remaining <= 0) remaining = 0;
            wait = remaining < pdMS_TO_TICKS(DISPLAY_REFRESH_MS)
                 ? remaining : pdMS_TO_TICKS(DISPLAY_REFRESH_MS);
        } else {
            wait = pdMS_TO_TICKS(DISPLAY_REFRESH_MS);
        }

        BaseType_t got = xTaskNotifyWait(0, ULONG_MAX, NULL, wait);

        now = xTaskGetTickCount();

        if (got == pdTRUE) {
            off_at = now + pdMS_TO_TICKS(BACKLIGHT_TIMEOUT_MS);
            if (!bl_lit) { backlight_set(0xFF); bl_lit = true; }
        }

        if (bl_lit && (int32_t)(now - off_at) >= 0) {
            backlight_set(0);
            bl_lit = false;
        }

        /* Don't repaint while an app or the menu owns the screen - it would
         * clobber their drawing. Restored via hal_app_display_release(). */
        if (display_owners == 0)
            display_refresh();
    }
}

static void button_irq_common(uint32_t btn_mask)
{
    BaseType_t woken = pdFALSE;
    if (bl_task_handle)
        xTaskNotifyFromISR(bl_task_handle, 1, eSetBits, &woken);
#ifdef FANTASI_ENABLE_GUI
    /* First press on a dark screen only wakes the backlight. */
    if (bl_lit)
        gui_buttons_from_isr(btn_mask, &woken);
#else
    (void)btn_mask;
#endif
    portYIELD_FROM_ISR(woken);
}

void EXTI3_IRQHandler(void)
{
    EXTI->PR1 = (1U << 3);
    button_irq_common(FANTASI_BTN_OK);        /* PH3 */
}

void EXTI9_5_IRQHandler(void)
{
    EXTI->PR1 = (1U << 6);
    button_irq_common(FANTASI_BTN_DOWN);      /* PC6 */
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t pr = EXTI->PR1 & ((1U << 10) | (1U << 11) | (1U << 12) | (1U << 13));
    EXTI->PR1 = pr;
    uint32_t mask = 0;
    if (pr & (1U << 10)) mask |= FANTASI_BTN_UP;      /* PB10 */
    if (pr & (1U << 11)) mask |= FANTASI_BTN_LEFT;    /* PB11 */
    if (pr & (1U << 12)) mask |= FANTASI_BTN_RIGHT;   /* PB12 */
    if (pr & (1U << 13)) mask |= FANTASI_BTN_BACK;    /* PC13 */
    button_irq_common(mask);
}

void hal_set_dfu_magic(void)
{
    g_fantasi_dfu_magic = FANTASI_DFU_MAGIC;
    __DSB();
}

void hal_reboot_dfu(void)
{
    hal_set_dfu_magic();
    NVIC_SystemReset();
    for (;;);
}

#define FZ_BACK_BTN_PIN 13   /* PC13, also wired to the PMIC /QON pin */

/* BQ25896 power-management IC - same I2C1 bus (PA9/PA10) we already drive for
 * the LP5562. 8-bit address; REG09 bit 5 (BATFET_DIS) forces the battery FET
 * off ("ship mode"). Values from the stock Flipper driver (bq25896_reg.h). */
#define BQ25896_ADDR             0xD6U
#define BQ25896_REG09            0x09U
#define BQ25896_REG09_BATFET_DIS (1U << 5)
#define BQ25896_REG0B            0x0BU
#define BQ25896_REG0B_RES        (1U << 1)   /* reserved: always reads 1 on a real BQ25896 */
#define BQ25896_REG0B_PG_STAT    (1U << 2)   /* power good: a valid input is present */
#define BQ25896_REG0B_VBUS_STAT  (0x7U << 5) /* VBUS input type; 0b000 = no input */

/* Real power-off: put the BQ25896 into ship mode, disconnecting the battery
 * from the whole system. Everything - CPU, radio, display, and the LP5562
 * backlight/LED - loses power at once, so there's nothing to blank first.
 * Wake by holding BACK (wired to the PMIC /QON pin) or by plugging in USB.
 *
 * This path is Flipper hardware. The Kiisu reuses this HAL but has different
 * power hardware - no BQ25896 gating the system rail - so we must NOT blindly
 * write BATFET_DIS and spin forever waiting for a rail that never drops. Two
 * guards make this safe on any board:
 *   1. REG0B bit 1 ("RES") always reads 1 on a real BQ25896; if it's clear the
 *      PMIC didn't ACK (absent / different hardware) → report unsupported.
 *   2. After the ship-mode write, wait a bounded moment for power to fall; if
 *      we're still running, recover instead of freezing.
 * Ship mode also can't cut the rail while USB feeds it, so we refuse when
 * externally powered. Does not return only when power actually drops. */
int hal_shutdown(void)
{
    /* Take the I2C1 bus atomically: stop the scheduler/ISRs so nothing else
     * (backlight task, batt reads) is mid-transfer, then reset the peripheral
     * to clear any partial transaction a just-preempted task may have left. */
    __disable_irq();
    /* Reset the I2C peripheral to clear any partial transfer a just-preempted
     * task left. PE must stay LOW for >= 3 APB cycles or the software reset
     * doesn't take and the next transaction NACKs - back-to-back writes don't
     * guarantee that, so hold it low with a few reads before re-enabling. */
    I2C1->CR1 &= ~I2C_CR1_PE;
    (void)I2C1->CR1; (void)I2C1->CR1; (void)I2C1->CR1;
    I2C1->CR1 |=  I2C_CR1_PE;

    uint8_t r0b = (uint8_t)i2c_read_reg16(BQ25896_ADDR, BQ25896_REG0B); /* low byte = REG0B */

    /* REG0B bit 1 ("RES") always reads 1 on a real BQ25896. If it's clear the
     * PMIC didn't ACK (genuinely absent), report unsupported rather than write
     * BATFET_DIS and wait for a rail cut that never comes. */
    if (!(r0b & BQ25896_REG0B_RES)) {
        __enable_irq();
        return HAL_SHUTDOWN_UNSUPPORTED;
    }

    if (r0b & (BQ25896_REG0B_PG_STAT | BQ25896_REG0B_VBUS_STAT)) {
        __enable_irq();
        return HAL_SHUTDOWN_USB_POWERED;
    }

    /* On battery: safe to power off. Wait for BACK release (it drives the PMIC
     * /QON pin) so we don't wake straight back out of ship mode. IRQs stay off
     * - we're about to lose power anyway. */
    while ((GPIOC->IDR & (1UL << FZ_BACK_BTN_PIN)) == 0)
        ;

    uint8_t r09 = (uint8_t)i2c_read_reg16(BQ25896_ADDR, BQ25896_REG09); /* low byte = REG09 */
    r09 |= BQ25896_REG09_BATFET_DIS;
    i2c_write_reg(BQ25896_ADDR, BQ25896_REG09, r09);

    /* Power should collapse within tens of ms. If it doesn't (BATFET not wired
     * to gate the system rail), don't freeze with IRQs off - spin a bounded
     * moment, then recover and report failure. */
    for (volatile uint32_t i = 0; i < 8000000; i++)
        __NOP();
    __enable_irq();
    return HAL_SHUTDOWN_UNSUPPORTED;
}

#ifdef FANTASI_ENABLE_APPS
/* Live button state for the app ABI (api->buttons()). Pins are configured by
 * buttons_init() long before any app can run. All active-low with pull-ups
 * except OK (PH3), which is active-high with a pull-down. */
uint32_t hal_app_buttons(void)
{
    uint32_t m = 0;
    if (!(GPIOB->IDR & (1UL << 10))) m |= FANTASI_BTN_UP;
    if (!(GPIOB->IDR & (1UL << 11))) m |= FANTASI_BTN_LEFT;
    if (!(GPIOB->IDR & (1UL << 12))) m |= FANTASI_BTN_RIGHT;
    if (!(GPIOC->IDR & (1UL << 6)))  m |= FANTASI_BTN_DOWN;
    if (!(GPIOC->IDR & (1UL << 13))) m |= FANTASI_BTN_BACK;
    if (GPIOH->IDR & (1UL << 3))     m |= FANTASI_BTN_OK;
    return m;
}
#endif

/* Back button = PC13, active-low with a pull-up (also configured by
 * buttons_init). Set it up lazily/idempotently so this works regardless of
 * task start order. */
bool hal_shutdown_button_held(void)
{
    static bool cfg;
    if (!cfg) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
        (void)RCC->AHB2ENR;
        GPIOC->MODER &= ~GPIO_MODER_MODE13;             /* input */
        GPIOC->PUPDR  = (GPIOC->PUPDR & ~GPIO_PUPDR_PUPD13)
                      | (1UL << GPIO_PUPDR_PUPD13_Pos); /* pull-up */
        cfg = true;
    }
    return (GPIOC->IDR & (1UL << FZ_BACK_BTN_PIN)) == 0;   /* active-low */
}
