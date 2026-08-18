/* Fantasi / Chameleon Ultra system bring-up
 *
 * The nRF52840 needs HFCLK from the external 32 MHz XTAL for the USB
 * peripheral to be spec-compliant; the internal RC isn't precise
 * enough. The DCDC regulator is enabled (the Chameleon has the external
 * inductors; stock also runs DCDC) - it roughly halves active current
 * vs the LDO default.
 *
 * FPU is enabled for the Cortex-M4F port. SystemCoreClock is fixed at
 * 64 MHz (nRF52840 runs core at a fixed 64 MHz - no PLL to configure). */

#include "nrf.h"
#include <stdint.h>

uint32_t SystemCoreClock = 64000000u;

void SystemInit(void)
{
    SCB->CPACR |= ((3U << (10*2)) | (3U << (11*2)));
    __DSB(); __ISB();

    /* DCDC regulator: left in LDO mode. Empirically, enabling DCDC here (raw
     * DCDCEN pre-SD, with or without sd_power_dcdc_mode_set post-SD) stalls
     * boot on this board even though stock runs DCDC - stock only ever sets it
     * through the SD SVC after SoftDevice init. Halving active current this
     * way is future work. */

    /* Disable the hardware APPROTECT enforcement introduced on post-2021
     * nRF52840 silicon (revision E and later, build codes QIAA-E*).
     * Without this write, the SWD port is **permanently locked** on the
     * first boot of any firmware that doesn't clear it - recovery then
     * requires a full `nrfjprog --recover` chip erase via an external
     * J-Link. The DFU path still works (so not a hard brick), but we'd
     * lose SWD forever on the user's device. Benign on older silicon:
     * the register exists on all variants; the hardware semantic is a
     * no-op on pre-errata-249 parts. Must run before anything else can
     * generate a debug event. */
    NRF_APPROTECT->DISABLE = 0x5Au;

    /* Start HFCLK on the external crystal. Required for USB; the
     * POWER/CLOCK IRQ also uses this event later to drive TinyUSB's
     * nrf power callbacks. */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
}
