/* Fantasi / Proxmark5 (AT32F435) system bring-up.
 *
 * SystemInit() runs from Reset_Handler before main(): enables the FPU, asserts the
 * board power-lock, and brings the core clock to 288 MHz off the 8 MHz HEXT crystal
 * with a 48 MHz USB clock (288/6). Register detail is from RM_AT32F435_437_V2.08
 * (see at32f435.h). Wait loops are bounded so a bad crystal can't hang the core.
 */

#include "at32f435.h"
#include <stdint.h>

uint32_t SystemCoreClock = 288000000u;

/* FreeRTOS idle hook: halt the core until the next interrupt instead of letting
 * the idle task spin at 288 MHz. Wakes on the 1 ms SysTick and any peripheral IRQ
 * (USB, etc.), so latency is unchanged - it just stops burning power doing nothing.
 * This is not low-power mode (no sleep states, no tick suppression); that's coming
 * from Soei. It is baseline "don't busy-wait" behaviour. Cuts active current
 * substantially. - noproto */
void vApplicationIdleHook(void)
{
    __WFI();
}

#define CLK_WAIT_TIMEOUT 500000u

void SystemInit(void)
{
    /* The PB0 power-supply lock is already held from the top of Reset_Handler
     * (startup.c) - it must be latched before the .bss clear, far earlier than
     * here, or the board powers itself off. */

    /* Enable the FPU: full access to CP10/CP11 (Cortex-M4F, -mfloat-abi=hard). */
    SCB->CPACR |= ((3u << (10 * 2)) | (3u << (11 * 2)));
    __DSB();
    __ISB();

    /* --- Core clock: HEXT -> PLL 288 MHz --- */

    /* Start HEXT (8 MHz crystal) and wait for it to stabilise. */
    CRM->CTRL |= CRM_CTRL_HEXTEN;
    for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
        if (CRM->CTRL & CRM_CTRL_HEXTSTBL) break;

    /* Raise the LDO to 1.3 V - required for any SCLK above 240 MHz. The LDO
     * voltage is only changeable while running on HICK/HEXT (we're on HICK
     * here, the reset default). PWC needs its APB1 clock gate on first. */
    CRM->APB1EN |= CRM_APB1EN_PWCEN;
    (void)CRM->APB1EN;
    PWC->LDOOV = PWC_LDOOV_1P3V;

    /* Configure and start the PLL: 8 MHz x144 / (1*4) = 288 MHz, HEXT ref. */
    CRM->PLLCFG = CRM_PLLCFG_288M_HEXT8;
    CRM->CTRL |= CRM_CTRL_PLLEN;
    int pll_locked = 0;
    for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
        if (CRM->CTRL & CRM_CTRL_PLLSTBL) { pll_locked = 1; break; }

    if (pll_locked) {
        /* Bus dividers: AHB /1 (288 MHz), APB1 /2 and APB2 /2 (144 MHz, the
         * APB ceiling). The zero-wait flash region is 0-wait to 288 MHz, so
         * there is no wait-state register to program (unlike STM32). */
        uint32_t cfg = CRM->CFG;
        cfg &= ~((0xFu << CRM_CFG_AHBDIV_POS) |
                 (0x7u << CRM_CFG_APB1DIV_POS) |
                 (0x7u << CRM_CFG_APB2DIV_POS));
        cfg |= (CRM_AHB_DIV1 << CRM_CFG_AHBDIV_POS) |
               (CRM_APB_DIV2 << CRM_CFG_APB1DIV_POS) |
               (CRM_APB_DIV2 << CRM_CFG_APB2DIV_POS);
        CRM->CFG = cfg;

        /* Enable the stepwise clock switch required when moving to >108 MHz. */
        CRM->MISC2 |= CRM_MISC2_AUTO_STEP_EN;

        /* Switch SCLK to the PLL and wait for the status to confirm. */
        CRM->CFG = (CRM->CFG & ~CRM_CFG_SCLKSEL_MSK) |
                   (CRM_SCLK_PLL << CRM_CFG_SCLKSEL_POS);
        for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
            if (((CRM->CFG & CRM_CFG_SCLKSTS_MSK) >> CRM_CFG_SCLKSTS_POS) == CRM_SCLK_PLL)
                break;

        /* USB 48 MHz off the 288 MHz PLL (HICK_TO_USB=0, the reset default; /6). */
        CRM->MISC1 &= ~CRM_MISC1_HICK_TO_USB;
        CRM->MISC2 = (CRM->MISC2 & ~CRM_MISC2_USBDIV_MSK) |
                     (CRM_USBDIV_6 << CRM_MISC2_USBDIV_POS);
    } else {
        /* HEXT/PLL never came up (no/odd crystal, wrong assumption). Do not switch
         * SCLK onto a dead PLL - that stops the core clock and the board looks
         * dead. Stay on the 48 MHz HICK the part booted on, and source USB
         * straight from HICK (the crystal-less USB path: HICK is factory-trimmed
         * to 48 MHz, and ACC can lock it to USB SOF). SystemCoreClock is corrected
         * so FreeRTOS tick timing follows the real clock. */
        SystemCoreClock = 48000000u;
        CRM->MISC1 |= CRM_MISC1_HICK_TO_USB;   /* USB clock = HICK 48 MHz */
    }
}
