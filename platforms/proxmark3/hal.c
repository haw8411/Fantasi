/* Fantasi / Proxmark3 (AT91SAM7S) - HAL glue for TinyUSB.
 *
 * Wires the TinyUSB device stack (CDC class) onto the AT91SAM7S UDP
 * peripheral via our dcd_at91sam7s driver. The upstream PM3
 * usb_cdc.c was a bare-metal polled design whose internal buffer
 * handling corrupted the calling task's stack frame under FreeRTOS
 * preemption - replaced entirely here.
 *
 * This file owns:
 *   - Clock and pull-up GPIO setup for the UDP peripheral
 *   - AIC wiring for UDP_IRQn → dcd_int_handler(0)
 *   - The tud_task FreeRTOS task (platform_usb_task)
 *   - Heap-size HAL helpers
 *
 * The generic read/write/connected HAL functions live in
 * hal/tinyusb/hal_serial_tinyusb.c - same file used by
 * Flipper and Chameleon. */

#include "at91sam7s512.h"
#include "../../hal/hal.h"
#include "../../hal/hal_name.h"
#include "hal_storage.h"

#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "app_api.h"   /* FANTASI_HID_* bits for hal_hid_host */
#ifdef FANTASI_ENABLE_APPS
#include "cli.h"
#include "app_run.h"   /* shortcut_run for the button launcher */
#endif

#ifndef GPIO_USB_PU
#  define GPIO_USB_PU  AT91C_PIO_PA24
#endif

/* IRQ trampoline: declared as an ARMv4T IRQ handler so GCC emits the
 * `subs pc, lr, #4` epilogue (restoring SPSR into CPSR on return).
 * A plain function would return with `bx lr`, which leaves the CPU
 * in IRQ mode with I=1. */
/* Switching ISR, modelled on FreeRTOS's vPreemptiveTick. dcd_int_handler only
 * queues USB events (notably a received SETUP) to the usbd task; it does not
 * respond in ISR context. The control response must therefore be prepared by the
 * usbd task within the xHCI control-transfer window (~150 us). A plain
 * interrupt("IRQ") handler cannot context-switch, so the woken usbd task would
 * not run until the next 1 ms tick - past the window. Saving the interrupted
 * task's context and calling vTaskSwitchContext() here lets the higher-priority
 * usbd task preempt immediately, the same pattern the tick ISR uses. */
static void udp_irq_trampoline(void) __attribute__((naked));
static void udp_irq_trampoline(void)
{
    portSAVE_CONTEXT();
    dcd_int_handler(0);
    vTaskSwitchContext();               /* run the usbd task now if the DCD woke it */
    /* AIC End-of-Interrupt: allows the AIC to drive NIRQ for the next pending
     * interrupt. Written before the epilogue so priority bookkeeping is correct. */
    AT91C_BASE_AIC->AIC_EOICR = 0;
    portRESTORE_CONTEXT();
}

#ifdef FANTASI_ENABLE_APPS
void pm3_launcher_init(void);   /* button/LED shortcut launcher, defined below */
#endif

void hal_init(void)
{
    /* Watchdog is write-once per reset; the PM3 bootloader leaves
     * WDTC_WDMR untouched so this write disables it for us. */
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    /* Peripheral clocks:
     *   - PIOA (ID 2): needed so that writes to PIOA_OER/SODR actually
     *     drive the pin - including PA24, the USB D+ pull-up. Without
     *     this, our dcd_connect() flip of SODR lands in the register
     *     but PIOA_PDSR stays 0.
     *   - UDP  (ID 11): peripheral clock for the USB device controller.
     *   - SCER.UDP: separate 48 MHz USB clock derived from PLL/USBDIV.
     *
     * The PLL USBDIV is already /1 from the PM3 bootrom so UDPCK is
     * 48 MHz. dcd_init() redundantly re-asserts these on every
     * tud_init(), but enabling them here first ensures any early AIC
     * setup or PIO access works before the USB task ever runs. */
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_PIOA) | (1u << AT91C_ID_UDP);
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;

    /* Point the AIC's UDP source vector at our trampoline. Priority 4
     * is mid-range; the FreeRTOS tick IRQ (priority 7) will still
     * preempt us if a USB transfer runs long. */
    AT91PS_AIC aic = AT91C_BASE_AIC;
    aic->AIC_IDCR = (1u << AT91C_ID_UDP);
    aic->AIC_SVR[AT91C_ID_UDP] = (uint32_t)udp_irq_trampoline;
    aic->AIC_SMR[AT91C_ID_UDP] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 4u;
    aic->AIC_ICCR = (1u << AT91C_ID_UDP);
    /* Do NOT enable the IRQ at the AIC here - dcd_int_enable() is
     * called from tud_init() on the USB task and handles that.
     * Enabling early invites an ENDBUSRES before the TinyUSB device
     * stack has a registered setup callback. */

    /* We deliberately do NOT touch the bootloader's common_area at
     * the top of SRAM. The bootrom maintains magic/version/command
     * itself across resets; writing it with a mis-laid-out struct
     * lands COMMAND=1 at the byte offset the bootrom reads as
     * COMMON_AREA_COMMAND_ENTER_FLASH_MODE - and the next warm reset
     * sticks us in the bootloader's serial-flash loop forever. */

    /* Turn off LED_A (PA0) and LED_C (PA9), turn on LED_D blue (PA8). */
    AT91C_BASE_PIOA->PIO_PER  = AT91C_PIO_PA0 | AT91C_PIO_PA8 | AT91C_PIO_PA9;
    AT91C_BASE_PIOA->PIO_OER  = AT91C_PIO_PA0 | AT91C_PIO_PA8 | AT91C_PIO_PA9;
    AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA0 | AT91C_PIO_PA9;
    AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA8;

    hal_storage_init();

#ifdef FANTASI_ENABLE_APPS
    pm3_launcher_init();   /* button/LED shortcut launcher (owns LED_A/B/C) */
#endif

    /* tud_init() is intentionally NOT called here - deferred to the
     * USB task. See platform_usb_task comment below. */
}

/* Mode-switching state - the USB task checks this flag each iteration
 * and re-enumerates with the appropriate descriptor set. */
extern volatile uint8_t pm3_usb_mode;
static volatile uint8_t pm3_mode_request = 0xFF;

int hal_enter_msc_mode(void)
{
    if (!hal_storage_mounted()) return -2;
    hal_storage_unmount();
    pm3_mode_request = 1;
    return 0;
}

void hal_on_msc_eject(void)
{
    pm3_mode_request = 0;
}

/* WebUSB (vendor protobuf) switch-mode: re-enumerate as a vendor-only device
 * (CDC torn down). The vendor interface fits SAM7S's 4 endpoints because CDC is
 * gone. `cdc` switches back to the serial CLI. The USB task performs the actual
 * disconnect/reconnect (see platform_usb_task). */
int hal_enter_webusb_mode(void)
{
    pm3_mode_request = 2;
    return 0;
}

int hal_enter_cdc_mode(void)
{
    pm3_mode_request = 0;
    return 0;
}

/* ---- USB HID keyboard (switch-mode: HID-only, CDC dropped) ----
 * The SAM7S's 4 endpoints can't host HID alongside CDC, so arming the keyboard
 * re-enumerates as a HID-only device (mode 3) and disarming restores the prior
 * personality (normally CDC). */
extern volatile uint8_t pm3_hid_host_leds;
static volatile uint8_t pm3_prev_mode;   /* personality to restore on disarm */

int hal_hid_enable(int on)
{
    if (on) {
        pm3_prev_mode   = pm3_usb_mode;   /* usually 0 (CDC) or 2 (vendor) */
        pm3_mode_request = 3;             /* USB task re-enumerates as HID-only */
        for (int i = 0; i < 4000; i++) {
            if (pm3_usb_mode == 3 && tud_mounted() && tud_hid_ready()) return 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return (pm3_usb_mode == 3 && tud_mounted()) ? 0 : -1;
    }

    /* Only switch USB back if we actually enumerated as HID (mode 3). app_run
     * calls this after EVERY app as a safety net, including plain non-HID apps
     * that never armed HID; for those pm3_usb_mode is still the CDC/vendor mode,
     * and requesting a switch would needlessly re-enumerate the pipe - dropping
     * the app's final `exit N`/`[killed]` line and churning the device. */
    if (pm3_usb_mode != 3)
        return 0;

    uint8_t empty[6] = { 0 };
    if (tud_hid_ready()) tud_hid_keyboard_report(0, 0, empty);   /* release keys */
    pm3_mode_request = pm3_prev_mode;                            /* back to CDC/vendor */
    return 0;
}

int hal_hid_send(uint8_t modifiers, const uint8_t *keys, uint8_t n)
{
    if (pm3_usb_mode != 3 || !tud_mounted()) return -1;

    uint8_t report[6] = { 0 };
    for (uint8_t i = 0; i < n && i < 6; i++) report[i] = keys[i];

    for (int i = 0; i < 200 && !tud_hid_ready(); i++) vTaskDelay(pdMS_TO_TICKS(1));
    if (!tud_hid_ready()) return -1;

    return tud_hid_keyboard_report(0, modifiers, report) ? 0 : -1;
}

uint32_t hal_hid_host(void)
{
    uint32_t bits = pm3_hid_host_leds;
    if (pm3_usb_mode == 3 && tud_mounted()) bits |= FANTASI_HID_HOST_MOUNTED;
    return bits;
}

/* ---- Button shortcut launcher (screenless slot selection) ----
 * The PM3 has one button (PA23, active-low) and 4 LEDs. App shortcuts (scN=<path>
 * in settings.cfg, slots 0-7) are chosen by pressing the button to count up, with
 * the slot shown in BINARY on the 3 non-blue LEDs (A=bit0, B=bit1, C=bit2; slot 0
 * = all off .. slot 7 = all on). The blue LED_D stays lit throughout. Holding the
 * button launches the app in the selected slot. LEDs are active-high (SODR=on). */
#ifdef FANTASI_ENABLE_APPS

#define PM3_LED_A     AT91C_PIO_PA0   /* binary bit 0 */
#define PM3_LED_B     AT91C_PIO_PA2   /* binary bit 1 */
#define PM3_LED_C     AT91C_PIO_PA9   /* binary bit 2 */
#define PM3_LED_D     AT91C_PIO_PA8   /* blue, always on */
#define PM3_BUTTON    AT91C_PIO_PA23  /* active-low (pull-up) */
#define PM3_HOLD_MS   600

static void pm3_leds_show(uint8_t slot)
{
    uint32_t set = PM3_LED_D, clr = 0;    /* blue always on */
    if (slot & 1) set |= PM3_LED_A; else clr |= PM3_LED_A;
    if (slot & 2) set |= PM3_LED_B; else clr |= PM3_LED_B;
    if (slot & 4) set |= PM3_LED_C; else clr |= PM3_LED_C;
    AT91C_BASE_PIOA->PIO_SODR = set;      /* drive high = on  */
    AT91C_BASE_PIOA->PIO_CODR = clr;      /* drive low  = off */
}

static bool pm3_button_down(void)
{
    return (AT91C_BASE_PIOA->PIO_PDSR & PM3_BUTTON) == 0;   /* active-low */
}

/* App-facing button read for the Berry `hardware` module (overrides the weak
 * default in core/app_run.c). The PM3 has a single button (PA23) -> OK. Reading
 * PDSR is non-destructive, so this coexists with pm3_launcher_task (which owns
 * the button); the pin is configured input+pull-up idempotently in case an app
 * reads it before the launcher's init has run. There is no BACK button, so a
 * hardware.loop() on the PM3 is exited with ^C (kill) rather than a button. */
uint32_t hal_app_buttons(void)
{
    static bool cfg;
    if (!cfg) {
        AT91C_BASE_PIOA->PIO_PER   = PM3_BUTTON;   /* PIO controls the pin */
        AT91C_BASE_PIOA->PIO_ODR   = PM3_BUTTON;   /* input */
        AT91C_BASE_PIOA->PIO_PPUER = PM3_BUTTON;   /* pull-up */
        cfg = true;
    }
    return (AT91C_BASE_PIOA->PIO_PDSR & PM3_BUTTON) == 0 ? FANTASI_BTN_OK : 0;
}

/* Discarding CLI session so app_run() works from this task. */
static cli_ctx_t pm3_launcher_ctx;
static size_t pm3_tp_write(const uint8_t *b, size_t n, void *c) { (void)b; (void)c; return n; }
static size_t pm3_tp_read(uint8_t *b, size_t n, void *c) { (void)b; (void)n; (void)c; return 0; }
static bool   pm3_tp_connected(void *c) { (void)c; return true; }

/* Run the selected shortcut's app. Runs on the launcher task itself: on PM3's
 * tight heap, spawning a separate worker task (its own stack on top of the app
 * image + Berry VM) would OOM, so the app-runner is this baseline task allocated
 * at boot. It hosts app_run()'s LittleFS/ELF-load path and button polling - see
 * pm3_launcher_init for sizing. */
static void pm3_launch(int slot)
{
    shortcut_run(slot);
    cli_bind_ctx(&pm3_launcher_ctx);   /* app_run rebinds the ctx; restore ours */
}

static void pm3_launcher_task(void *arg)
{
    (void)arg;

    /* LED_B as output (A/C/D already set up in hal_init); button as input+pull-up. */
    AT91C_BASE_PIOA->PIO_PER   = PM3_LED_B | PM3_BUTTON;
    AT91C_BASE_PIOA->PIO_OER   = PM3_LED_B;
    AT91C_BASE_PIOA->PIO_ODR   = PM3_BUTTON;
    AT91C_BASE_PIOA->PIO_PPUER = PM3_BUTTON;

    pm3_launcher_ctx.transport.write     = pm3_tp_write;
    pm3_launcher_ctx.transport.read      = pm3_tp_read;
    pm3_launcher_ctx.transport.connected = pm3_tp_connected;
    cli_bind_ctx(&pm3_launcher_ctx);

    int sel = 0;
    bool prev = false, launched = false;
    TickType_t press_start = 0;

    for (;;) {
        pm3_leds_show((uint8_t)sel);

        bool now = pm3_button_down();
        TickType_t t = xTaskGetTickCount();

        if (now && !prev) { press_start = t; launched = false; }        /* press edge */
        if (now && !launched && (t - press_start) >= pdMS_TO_TICKS(PM3_HOLD_MS)) {
            launched = true;                                            /* fire once per hold */
            AT91C_BASE_PIOA->PIO_SODR = PM3_LED_A | PM3_LED_B | PM3_LED_C | PM3_LED_D;
            pm3_launch(sel);
        }
        if (!now && prev && !launched) sel = (sel + 1) & 7;            /* short press -> next slot */
        prev = now;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void pm3_launcher_init(void)
{
    /* 4 KB: this task hosts app_run()'s ELF-load path (pm3_launch). The app is
     * run in a separately-spawned "app" task. Small enough so PM3's 40 KB heap
     * has room for a Berry payload (~5 KB VM) alongside the ~9 KB app image. */
    xTaskCreate(pm3_launcher_task, "sclaunch", configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
#endif /* FANTASI_ENABLE_APPS */

void platform_usb_task(void *arg)
{
    (void)arg;
    tud_init(0);
    for (;;) {
        /* Bounded timeout (not the default block-forever tud_task()): with no USB
         * traffic - e.g. an on-device button launch, where nothing is talking to
         * the CDC/vendor pipe - a forever-blocking tud_task() would never return
         * to check pm3_mode_request, so a mode switch requested by the app (arm
         * HID) would never happen. Waking every 10 ms lets the switch run. */
        tud_task_ext(10, false);

        if (pm3_mode_request != 0xFF) {
            uint8_t new_mode = pm3_mode_request;
            pm3_mode_request = 0xFF;

            tud_disconnect();
            vTaskDelay(pdMS_TO_TICKS(800));
            pm3_usb_mode = new_mode;
            tud_connect();
        }

        vTaskDelay(1);
    }
}

/* HAL serial wrappers on top of TinyUSB's CDC class. We don't share
 * hal/tinyusb/hal_serial_tinyusb.c because platform_usb_task needs
 * a different shape on PM3 (tud_init runs from the task, not hal_init
 * - see the comment there). */
size_t hal_serial_write(const uint8_t *buf, size_t len)
{
    if (!tud_cdc_connected()) return 0;
    uint32_t wrote = tud_cdc_write(buf, len);
    tud_cdc_write_flush();
    return (size_t)wrote;
}

size_t hal_serial_read(uint8_t *buf, size_t len)
{
    if (!tud_cdc_available()) return 0;
    return (size_t)tud_cdc_read(buf, len);
}

bool hal_serial_connected(void)                { return tud_cdc_connected(); }
size_t hal_free_heap_bytes(void)               { return (size_t)xPortGetFreeHeapSize(); }
size_t hal_min_ever_free_heap_bytes(void)      { return (size_t)xPortGetMinimumEverFreeHeapSize(); }

void hal_post_init(void) {}

const char *hal_device_id(void) { return "PM3"; }

static uint16_t spi_xfer(uint32_t data)
{
    AT91C_BASE_SPI->SPI_TDR = data;
    while (!(AT91C_BASE_SPI->SPI_SR & AT91C_SPI_RDRF)) {}
    return AT91C_BASE_SPI->SPI_RDR & 0xFFFF;
}

const char *hal_device_name(void)
{
    static char name[16];
    if (name[0]) return name;

    /* Read the external SPI flash's 64-bit unique ID (cmd 0x4B).
     * SPI flash is on NPCS2 (PA10). */
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_SPI);
    AT91C_BASE_PIOA->PIO_PDR = (1u<<10)|(1u<<11)|(1u<<12)|(1u<<13)|(1u<<14);
    AT91C_BASE_PIOA->PIO_ASR = (1u<<11)|(1u<<12)|(1u<<13)|(1u<<14);
    AT91C_BASE_PIOA->PIO_BSR = (1u<<10);

    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SWRST;
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SWRST;
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SPIEN;

    /* NPCS2 selected, master, fixed peripheral, mode-fault disabled */
    AT91C_BASE_SPI->SPI_MR = ((~(1u << 2) & 0xF) << 16) |
                              (1u << 4) | AT91C_SPI_PS_FIXED | AT91C_SPI_MSTR;

    /* CSR2: ~24 MHz (MCK/2), CPOL=0, NCPHA=1, CSAAT=1, 8-bit */
    AT91C_BASE_SPI->SPI_CSR[2] = (2u << 8) |
                                  AT91C_SPI_BITS_8 |
                                  (1u << 1) |  /* NCPHA */
                                  (1u << 3);   /* CSAAT */

    /* Command 0x4B: Read Unique ID - 1 cmd byte + 4 dummy + 8 data */
    spi_xfer(0x4B);
    spi_xfer(0xFF);
    spi_xfer(0xFF);
    spi_xfer(0xFF);
    spi_xfer(0xFF);

    uint32_t u[2];
    uint8_t *b = (uint8_t *)u;
    b[7] = (uint8_t)spi_xfer(0xFF);
    b[6] = (uint8_t)spi_xfer(0xFF);
    b[5] = (uint8_t)spi_xfer(0xFF);
    b[4] = (uint8_t)spi_xfer(0xFF);
    b[3] = (uint8_t)spi_xfer(0xFF);
    b[2] = (uint8_t)spi_xfer(0xFF);
    b[1] = (uint8_t)spi_xfer(0xFF);
    b[0] = (uint8_t)spi_xfer(0xFF | AT91C_SPI_LASTXFER);

    /* Disable SPI */
    AT91C_BASE_SPI->SPI_CSR[0] = 0;
    AT91C_BASE_SPI->SPI_CSR[1] = 0;
    AT91C_BASE_SPI->SPI_CSR[2] = 0;
    AT91C_BASE_SPI->SPI_CSR[3] = 0;
    AT91C_BASE_SPI->SPI_MR = 0;
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SPIDIS;

    hal_name_generate(u, 2, name, sizeof(name));
    return name;
}

extern uint8_t _ram_start, _ram_end;
extern uint8_t __heap_start__, __heap_end__;

int hal_mem_regions(hal_mem_region_t *out, int max)
{
    int n = 0;
    if (n < max) {
        /* The FreeRTOS heap spans all free RAM (ucHeap is aliased onto the linker
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
     * the live FreeRTOS heap, and everything below it is static .bss - none of
     * it can be safely pattern-tested while in use. */
    return 0;
}

extern uint8_t _eflash;

int32_t hal_flash_free_bytes(void)
{
    uint32_t cidr = *AT91C_DBGU_CIDR;
    uint32_t nvpsiz = (cidr >> 8) & 0xFU;
    uint32_t total;
    switch (nvpsiz) {
        case  3: total =  32U * 1024; break;
        case  5: total =  64U * 1024; break;
        case  7: total = 128U * 1024; break;
        case  9: total = 256U * 1024; break;
        case 10: total = 512U * 1024; break;
        default: return -1;
    }
    uint32_t flash_end = 0x00100000U + total;
    uint32_t used_end  = (uint32_t)&_eflash;
    if (flash_end <= used_end) return 0;
    return (int32_t)(flash_end - used_end);
}

int hal_battery_percent(void) { return -1; }

int hal_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms)
{
    (void)cb; (void)duration_ms;
    return -1;
}

int hal_ble_pair_setup(uint8_t io_cap)
{ (void)io_cap; return -1; }
void hal_ble_pair_begin(void) {}
void hal_ble_pair_end(void)   {}
int hal_ble_pair_connect(const uint8_t *addr, uint8_t addr_type)
{ (void)addr; (void)addr_type; return -1; }
void hal_ble_shutdown(void) {}
void hal_ble_activate_fus(void) {}
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

void hal_radio_info(hal_radio_info_t *info)
{
    __builtin_memset(info, 0, sizeof(*info));
}


/* The PM3 bootrom maintains a 16-byte "common_area" struct at the top
 * 32 bytes of SRAM (see linker.ld). Across warm resets the bootrom
 * preserves magic/version/flags and dispatches on the 1-byte COMMAND
 * field at offset 5: COMMAND=1 (ENTER_FLASH_MODE) makes it stay in the
 * serial-flash loop instead of chaining into our osimage.
 *
 * Layout (from original_fw/proxmark3/include/proxmark3_arm.h):
 *   [0..3]  int   magic    = 0x43334D50 ('PM3C')
 *   [4]     char  version  = 1
 *   [5]     char  command  = 0 normal, 1 enter-flash-mode
 *   [6]     flags (packed bitfield, 1 byte)
 *   [7..14] int arg1, arg2
 *
 * We deliberately only poke the COMMAND byte - the rest was set up by
 * the bootrom and overwriting it can brick the warm-boot handoff (magic
 * goes missing → bootrom zeroes the area → the `bootrom_present` flag is
 * lost for subsequent resets). */
#define PM3_COMMON_AREA_ADDR       (0x00210000U - 0x20U)
#define PM3_COMMON_AREA_MAGIC      0x43334D50U
#define PM3_COMMON_AREA_CMD_OFFSET 5
#define PM3_COMMON_AREA_CMD_FLASH  1

#define PM3_RSTC_RCR_KEY           (0xA5U << 24)
#define PM3_RSTC_RCR_PROCRST       (1U << 0)
#define PM3_RSTC_RCR_PERRST        (1U << 2)

static void pm3_reset_now(void) __attribute__((noreturn));
static void pm3_reset_now(void)
{
    /* Reset both the processor and peripherals. PROCRST alone leaves
     * the UDP peripheral mid-transfer, which can confuse the host's
     * disconnect/re-enumerate bookkeeping. */
    AT91C_BASE_RSTC->RSTC_RCR = PM3_RSTC_RCR_KEY
                              | PM3_RSTC_RCR_PROCRST
                              | PM3_RSTC_RCR_PERRST;
    for (;;);
}

void hal_reboot(void)
{
    pm3_reset_now();
}

/* Strong fantasi_reset() for the PM3; the weak core/main.c fallback only spins.
 * libc_glue.c routes assert()/_exit() here and LFS_ASSERT is live on the PM3, so
 * this must actually reset the part: a spin would hang the faulting task with the
 * filesystem left mid-operation, recoverable only by a physical replug. The
 * Cortex-M platforms define theirs in startup.c. */
void fantasi_reset(void)
{
    pm3_reset_now();
}

/* The AT91SAM7S has no true off state (no System OFF / Shutdown mode, and
 * the PM3 has no power-management IC we drive). Report unsupported. */
int hal_shutdown(void)
{
    return HAL_SHUTDOWN_UNSUPPORTED;
}

void hal_set_dfu_magic(void)
{
    volatile uint32_t *magic = (volatile uint32_t *)PM3_COMMON_AREA_ADDR;
    volatile uint8_t  *cmd   = (volatile uint8_t  *)(PM3_COMMON_AREA_ADDR
                                                     + PM3_COMMON_AREA_CMD_OFFSET);
    /* Only set the command if the bootrom's common_area is intact. If
     * magic is missing the bootrom will re-init the region on the next
     * reset anyway; poking COMMAND would be ignored at best, and at
     * worst land a stray 1 in an uninitialised byte that the bootrom
     * then zeroes before reading. */
    if (*magic == PM3_COMMON_AREA_MAGIC)
        *cmd = PM3_COMMON_AREA_CMD_FLASH;
}

void hal_reboot_dfu(void)
{
    hal_set_dfu_magic();
    pm3_reset_now();
}
