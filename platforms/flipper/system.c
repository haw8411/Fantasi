/* Fantasi / Flipper Zero (STM32WB55) system bring-up
 *
 * Minimal clock config: switch SYSCLK from MSI (4 MHz default) to
 * HSI16 (16 MHz internal RC), turn on HSI48 for the USB 48 MHz
 * domain, and let the CRS peripheral trim HSI48 against the USB SOF.
 * No HSE, no PLL, no voltage-scaling change - 16 MHz is the ceiling
 * for VOS Range 2, so we stay in reset-default low-power mode and
 * can't mis-sequence a speed/VOS dependency.
 *
 * Why 16 MHz and not 4: the USB peripheral accesses its PMA via the
 * AHB bus at SYSCLK frequency. At 4 MHz AHB can't keep up with the
 * 12 Mbps FS packet rate - USB_ISTR raises PMAOVR on every packet
 * and enumeration stalls before SET_ADDRESS.
 *
 * Why not HSE+PLL (what the stock Flipper firmware does for 64 MHz):
 * that path needs HSE cap-tuning, VOS Range 1, flash latency 3WS,
 * and PLLSAI1 - every one of which has to be right before the first
 * instruction after the PLL switch. For a CLI we don't need the
 * speed, and fewer sequencing dependencies = less to go wrong on
 * bring-up. CRS on the SOF pulse brings HSI48 inside USB-FS spec
 * (±0.25%) so enumeration is reliable.
 *
 * If you later need 64 MHz: enable PWR, set VOS Range 1 + wait for
 * VOSF, set flash 3WS, start HSE (HSECR cap-tune 0x26), bring up
 * PLL, then switch SYSCLK. Nothing here precludes that. */

#include "stm32wbxx.h"

#include <stdint.h>

/* SYSCLK is HSI16 (16 MHz internal RC). FreeRTOSConfig.h's
 * configCPU_CLOCK_HZ must match. MSI (4 MHz default) is too slow for
 * the USB peripheral's PMA access over AHB - we hit PMAOVR errors
 * during enumeration. 16 MHz is the minimum reliable SYSCLK for
 * STM32WB USB FS and stays within VOS Range 2, so no voltage
 * scaling change is needed. */
uint32_t SystemCoreClock = 16000000u;

void SystemInit(void)
{
    /* Enable the FPU (CP10/CP11 full access). Safe at any voltage scale. */
    SCB->CPACR |= ((3U << (10*2)) | (3U << (11*2)));
    __DSB(); __ISB();

    /* Start HSI16 (internal 16 MHz RC). After reset HSION is already
     * set and HSI16 is ready, but we explicitly (re)enable and wait
     * to keep the sequence explicit. */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) { }

    /* Switch SYSCLK from MSI to HSI16 (SW field = 0b01). */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (1U << RCC_CFGR_SW_Pos);
    while (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 1U) { }

    /* Flash: 0 wait states is fine up to 18 MHz in Range 2. No change needed. */

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

    SystemCoreClock = 16000000u;
}
