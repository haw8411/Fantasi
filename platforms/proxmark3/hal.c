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

#ifndef GPIO_USB_PU
#  define GPIO_USB_PU  AT91C_PIO_PA24
#endif

/* IRQ trampoline: declared as an ARMv4T IRQ handler so GCC emits the
 * `subs pc, lr, #4` epilogue (restoring SPSR into CPSR on return).
 * A plain function would return with `bx lr`, which leaves the CPU
 * in IRQ mode with I=1. */
static void udp_irq_trampoline(void) __attribute__((interrupt("IRQ")));
static void udp_irq_trampoline(void)
{
    dcd_int_handler(0);
    /* AIC End-of-Interrupt: allows the AIC to drive NIRQ for the next
     * pending interrupt. Must be written before the IRQ epilogue so
     * priority bookkeeping is correct. */
    AT91C_BASE_AIC->AIC_EOICR = 0;
}

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

void platform_usb_task(void *arg)
{
    (void)arg;
    tud_init(0);
    for (;;) {
        tud_task();

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
