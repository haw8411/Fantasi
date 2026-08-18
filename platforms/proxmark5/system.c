/* Fantasi / Proxmark5 (AT32F435) system bring-up.
 *
 * SystemInit() runs from Reset_Handler before main(): enables the FPU, asserts the
 * board power-lock, and brings the core clock to 288 MHz off the 8 MHz HEXT crystal
 * with a 48 MHz USB clock (288/6). Register detail is from RM_AT32F435_437_V2.08
 * (see at32f435.h). Wait loops are bounded so a bad crystal can't hang the core.
 */

#include "at32f435.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "../../hal/hal_power.h"

uint32_t SystemCoreClock = 288000000u;

#define CLK_WAIT_TIMEOUT 500000u

/* No downclock / no sleep-veto exemptions during early boot: USB enumeration +
 * storage/FPGA bring-up run on a fully awake, full-speed system. */
#define PM5_BOOT_GRACE_MS 3000

/* ---- Dynamic clock + voltage scaling (idle downclock) ----------------------
 * Idle heat here is dominated by the 288 MHz PLL and 1.3 V LDO, not the core/bus
 * dynamic (per the datasheet Run-mode table) - so at idle we take SCLK off the
 * PLL to 48 MHz (HICK), power the PLL down, and drop the LDO to 1.1 V. Boost
 * restores 288 MHz / 1.3 V for timing-critical work (RFID; the LED-fade PWM).
 *
 * USB is sourced from HICK-48 (crystal-less, ACC-calibrated in SystemInit),
 * independent of the PLL, so powering the PLL up/down and muxing SCLK never
 * touch the USB clock. SCLK-from-HICK is 48 MHz with HICKDIV=1 + HICK_TO_SCLK=1.
 *
 * Ordering is voltage-safe: up, LDO before frequency; down, frequency before
 * LDO. Flash defaults (FDIV=2) span both operating points. SysTick is
 * reprogrammed on each switch to hold the 1 kHz tick, so the compile-time-clock
 * tickless idle is off - a clock-aware idle-hook WFI sleeps instead. */
#define PM5_SCLK_HIGH  288000000u
#define PM5_SCLK_LOW    48000000u   /* HICK-48 direct; the floor. 24 MHz (AHB /2) can't
                                     * sustain USB: the DWC2 core needs a continuous
                                     * >=~48 MHz for the host's own link maintenance (SOF,
                                     * control), which a proto-activity boost can't cover. */

static volatile uint32_t s_clk_boost;      /* >0 = something is holding 288 MHz */
static volatile bool     s_clk_high = true;/* boot runs at 288 MHz */
static volatile bool     s_clk_settled;    /* boot grace elapsed -> idle may downclock */

static void pm5_systick_reload(uint32_t hclk_hz)
{
    /* FreeRTOS ARM_CM4F SysTick counts the core clock; hold the tick at 1 kHz. */
    SysTick->LOAD = (hclk_hz / configTICK_RATE_HZ) - 1u;
    SysTick->VAL  = 0u;
}

static void pm5_clk_apply(bool high)
{
    if (high) {
        /* low -> high: raise LDO first (we're on HICK, so the LDO is writable),
         * restore AHB /1, spin the PLL back up, then mux SCLK onto it. */
        PWC->LDOOV = PWC_LDOOV_1P3V;
        for (volatile uint32_t d = 0; d < 2000u; d++) { }   /* LDO settle */
        CRM->CTRL |= CRM_CTRL_PLLEN;
        for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
            if (CRM->CTRL & CRM_CTRL_PLLSTBL) break;
        CRM->CFG = (CRM->CFG & ~CRM_CFG_SCLKSEL_MSK) | (CRM_SCLK_PLL << CRM_CFG_SCLKSEL_POS);
        for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
            if (((CRM->CFG & CRM_CFG_SCLKSTS_MSK) >> CRM_CFG_SCLKSTS_POS) == CRM_SCLK_PLL) break;
    } else {
        /* high -> low: drop SCLK to HICK-48 (frequency-down before voltage-down),
         * then power the PLL off and lower the LDO. USB PHY stays on HICK-48. */
        CRM->CFG = (CRM->CFG & ~CRM_CFG_SCLKSEL_MSK) | (CRM_SCLK_HICK << CRM_CFG_SCLKSEL_POS);
        for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
            if (((CRM->CFG & CRM_CFG_SCLKSTS_MSK) >> CRM_CFG_SCLKSTS_POS) == CRM_SCLK_HICK) break;
        CRM->CTRL &= ~CRM_CTRL_PLLEN;          /* PLL off - the big saving */
        PWC->LDOOV = PWC_LDOOV_1P1V;           /* 1.3 V -> 1.1 V */
    }
    SystemCoreClock = high ? PM5_SCLK_HIGH : PM5_SCLK_LOW;
    pm5_systick_reload(SystemCoreClock);
    s_clk_high = high;
}

/* Refcounted boost: any caller needing full speed holds one. Called from task
 * context (RFID, launcher) - a short IRQ-off guard keeps the count + switch
 * atomic against each other and the idle task's settle/unboost. */
void pm5_clk_boost(void)
{
    __disable_irq();
    if (s_clk_boost++ == 0 && !s_clk_high) pm5_clk_apply(true);
    __enable_irq();
}

void pm5_clk_unboost(void)
{
    __disable_irq();
    if (s_clk_boost > 0 && --s_clk_boost == 0 && s_clk_settled && s_clk_high)
        pm5_clk_apply(false);
    __enable_irq();
}

/* One-shot: once boot grace elapses, drop to 48 MHz if nothing is holding a
 * boost. Called from the idle hook (so it runs only after the scheduler is up
 * and the system has gone idle at least once). */
void pm5_clk_settle(void)
{
    if (s_clk_settled) return;
    if (xTaskGetTickCount() < pdMS_TO_TICKS(PM5_BOOT_GRACE_MS)) return;
    __disable_irq();
    s_clk_settled = true;
    if (s_clk_boost == 0 && s_clk_high) pm5_clk_apply(false);
    __enable_irq();
}

/* Actual asleep-time accounting. We measure each WFI directly with the DWT
 * cycle counter (free-running at SCLK); cycles-per-us tracks the current clock
 * (288 or 48) so the fraction stays right across a downclock. The 32-bit CYCCNT
 * wraps in ~15 s at 288 MHz / ~89 s at 48 MHz, but each WFI is <= 1 ms (bounded
 * by SysTick) so a per-WFI delta never wraps; whole ms are credited, the sub-ms
 * remainder carried in s_us_resid so nothing is lost. */
static uint32_t s_us_resid;

static void pm5_credit_wfi(uint32_t cyc)
{
    uint32_t cyc_per_us = SystemCoreClock / 1000000u;   /* 288 or 48 */
    s_us_resid += cyc / cyc_per_us;
    if (s_us_resid >= 1000u) {
        pwr_note_slept(s_us_resid / 1000u);   /* whole ms (= ticks at 1 kHz) */
        s_us_resid %= 1000u;
    }
}

/* FreeRTOS idle hook: settle the clock down after boot, then halt the core
 * until the next interrupt instead of spinning. With tickless off the core
 * wakes each 1 kHz SysTick, but WFI still sleeps the gap, so the DWT-measured
 * asleep fraction stays ~99%. `power sleep off` keeps it spinning. */
void vApplicationIdleHook(void)
{
    pm5_clk_settle();
    if (pwr_allowed_depth() == HAL_SLEEP_NONE) return;
    pwr_note_light_sleep();
    /* Mask IRQs across the WFI so the DWT delta measures only the sleep. WFI
     * still wakes on a pending (enabled) IRQ with PRIMASK set; we read the
     * counter before re-enabling, so the waking ISR (the SysTick tick handler)
     * runs after the measurement, not counted as sleep. */
    __disable_irq();
    uint32_t t0 = DWT->CYCCNT;
    __WFI();
    pm5_credit_wfi(DWT->CYCCNT - t0);
    __enable_irq();
}

void SystemInit(void)
{
    /* The PB0 power-supply lock is already held from the top of Reset_Handler
     * (startup.c) - it must be latched before the .bss clear, far earlier than
     * here, or the board powers itself off. */

    /* Enable the FPU: full access to CP10/CP11 (Cortex-M4F, -mfloat-abi=hard). */
    SCB->CPACR |= ((3u << (10 * 2)) | (3u << (11 * 2)));
    __DSB();
    __ISB();

    /* Free-running DWT cycle counter - the timebase for measuring actual WFI
     * (asleep) duration, so `power` reports a real time-asleep fraction. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* --- Core clock: HEXT -> PLL 288 MHz --- */

    /* Start HEXT (8 MHz crystal) and wait for it to stabilise. */
    CRM->CTRL |= CRM_CTRL_HEXTEN;
    for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
        if (CRM->CTRL & CRM_CTRL_HEXTSTBL) break;

    /* Raise the LDO to 1.3 V for the 288 MHz PLL boot (changeable only while on
     * HICK/HEXT - we're on HICK here, the reset default). PWC needs its APB1
     * clock gate on first. The idle settle later drops this to 1.1 V. */
    CRM->APB1EN |= CRM_APB1EN_PWCEN;
    (void)CRM->APB1EN;
    PWC->LDOOV = PWC_LDOOV_1P3V;

    /* USB runs crystal-less off HICK-48, independent of the PLL. HICKDIV=1 makes
     * HICK 48 MHz; HICK_TO_SCLK=1 makes SCLK-from-HICK 48 MHz (the idle operating
     * point); HICK_TO_USB=1 routes USB off HICK-48. ACC then trims HICK to
     * +/-0.25% against the USB SOF (driven as soon as the port is enabled, so it
     * locks before enumeration finishes). Set before the PLL: HICKDIV must not
     * move with the PLL running. Also lifts SCLK from 8 MHz to 48 MHz. */
    CRM->MISC1 |= CRM_MISC1_HICKDIV | CRM_MISC1_HICK_TO_SCLK | CRM_MISC1_HICK_TO_USB;
    CRM->APB2EN |= CRM_APB2EN_ACCEN;
    (void)CRM->APB2EN;
    ACC_CTRL1 |= ACC_CTRL1_ENTRIM | ACC_CTRL1_CALON;

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

        /* Enable the stepwise clock switch required when moving to >108 MHz -
         * pm5_clk_apply() relies on it for the 48<->288 idle/boost switches too. */
        CRM->MISC2 |= CRM_MISC2_AUTO_STEP_EN;

        /* Boot at 288 MHz (SCLK on the PLL) for fast USB enum + bring-up; the
         * idle settle drops SCLK to HICK-48, powers the PLL down and lowers the
         * LDO. USB (on HICK-48) is unaffected throughout. */
        CRM->CFG = (CRM->CFG & ~CRM_CFG_SCLKSEL_MSK) |
                   (CRM_SCLK_PLL << CRM_CFG_SCLKSEL_POS);
        for (uint32_t i = 0; i < CLK_WAIT_TIMEOUT; i++)
            if (((CRM->CFG & CRM_CFG_SCLKSTS_MSK) >> CRM_CFG_SCLKSTS_POS) == CRM_SCLK_PLL)
                break;
    } else {
        /* PLL never came up (no/odd crystal). Stay on HICK-48, which is already
         * our SCLK and USB source. SystemCoreClock corrected for the tick. */
        SystemCoreClock = 48000000u;
    }
}
