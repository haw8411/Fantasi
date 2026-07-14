/* Fantasi / Flipper Zero (STM32WB55) system bring-up
 *
 * Clock config: switch SYSCLK from MSI (4 MHz default) to the 32 MHz HSE
 * crystal, turn on HSI48 for the USB 48 MHz domain, and let CRS trim HSI48
 * against the USB SOF.
 *
 * Why HSE 32 MHz: the CPU2 wireless stack switches SYSCLK to HSE when BLE comes
 * up, so CPU1 owns HSE here, before the scheduler sizes SysTick, to keep one
 * constant frequency that configCPU_CLOCK_HZ matches. CPU2's later switch is then
 * a no-op, and the FreeRTOS tick stays a true 1 kHz.
 *
 * Sequence raises voltage and latency before frequency: VOS Range 1 (the reset
 * default; Range 2 caps at 16 MHz), flash 1 WS (needed 18-36 MHz), HSE with the
 * Flipper's 0x26 cap-tune, then switch SYSCLK. USB is on the separate HSI48
 * domain, and CRS on the SOF pulse trims it inside USB-FS spec.
 *
 * 32 MHz needs no PLL; the stock 64 MHz path (HSE+PLL) would add flash 3WS and
 * PLLSAI1 sequencing for speed a CLI does not need. */

#include "stm32wbxx.h"

#include <stdint.h>

/* SYSCLK is the 32 MHz HSE crystal; configCPU_CLOCK_HZ must match it. */
uint32_t SystemCoreClock = 32000000u;

void SystemInit(void)
{
    /* Enable the FPU (CP10/CP11 full access). Safe at any voltage scale. */
    SCB->CPACR |= ((3U << (10*2)) | (3U << (11*2)));
    __DSB(); __ISB();

    /* Make the 32 MHz HSE the SYSCLK (the comment in the header above covers why
     * HSE). Order is load-bearing: raise voltage (VOS Range 1) and flash latency
     * (1 WS) before the frequency, then enable HSE with the Flipper's 0x26
     * cap-tune and switch. */
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | (1U << PWR_CR1_VOS_Pos);
    while (PWR->SR2 & PWR_SR2_VOSF) { }

    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_1WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_1WS) { }

    RCC->HSECR = (RCC->HSECR & ~RCC_HSECR_HSETUNE_Msk)
               | (0x26U << RCC_HSECR_HSETUNE_Pos);
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) { }

    /* Switch SYSCLK from MSI to HSE (SW field = 0b10). */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (2U << RCC_CFGR_SW_Pos);
    while (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 2U) { }

    /* Start HSI48 (internal 48 MHz RC dedicated to USB/RNG). */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0) { }

    /* Route the CLK48 mux to HSI48 (CCIPR field CLK48SEL = 0b00). */
    RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL;

    /* VDDUSB supply monitoring - required for USB FS to see bus power.
     * PWR is always clocked on STM32WB, no bus-enable step needed. */
    PWR->CR2 |= PWR_CR2_USV;

    /* Bring up the CRS (Clock Recovery System) locked to USB SOF so
     * HSI48 is trimmed to <±0.25 %, well within USB-FS spec. Without
     * CRS, HSI48 factory accuracy is only ±1 %, which some hosts
     * accept for enumeration but which drops bulk packets. Safe order:
     *   1. Enable CRS bus clock
     *   2. Program CFGR with USB SOF source (SYNCSRC=10) and the
     *      default reload value for a 1 kHz sync (RELOAD = 48000-1)
     *   3. Enable auto-trim + CRS (AUTOTRIMEN + CEN)
     * USB host drives SOF every 1 ms after reset, so by the time
     * tusb_init() finishes, the trim loop is converging. */
    RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;
    (void)RCC->APB1ENR1;

    CRS->CFGR = (47999U << CRS_CFGR_RELOAD_Pos)      /* reload = fSYS/fSYNC - 1 = 48e6/1e3 - 1 */
              | (0x22U   << CRS_CFGR_FELIM_Pos)      /* frequency error limit (default) */
              | (0x2U    << CRS_CFGR_SYNCSRC_Pos);   /* 0b10 = USB SOF */
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    SystemCoreClock = 32000000u;
}
