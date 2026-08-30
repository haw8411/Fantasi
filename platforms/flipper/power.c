// Fantasi / Flipper Zero - low-power

#include "stm32wbxx.h"
#include "power.h"
#include "ble.h"
#include "../../hal/hal.h"
#include "../../hal/hal_power.h"
#include "../../core/log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <stdio.h>

/* No sleep during early boot: USB enumeration + CPU2/WS bring-up run on a
 * fully awake system (keeps any sleep regression reachable). */
#define PWR_BOOT_GRACE_MS 5000

/* ---- Reset forensics ---- */

/* Survives warm reset (linker .noinit, same mechanism as g_fantasi_dfu_magic).
 * High 24 bits: magic; low 8: hal_power.h HAL_CRASH_* code. */
#define CRASH_MAGIC 0x46435200u   /* "FCR\0" */
__attribute__((section(".noinit"))) static volatile uint32_t s_crash_word;

/* Fault recovery: record the crash code and warm-reset (preserves .noinit, so
 * the boot log reports the crash) instead of hanging in the startup weak
 * spin-loop handlers. */
static void fz_fault_reset(void)
{
    s_crash_word = CRASH_MAGIC | HAL_CRASH_RADIO_FAULT;
    NVIC_SystemReset();
    for (;;);
}
void HardFault_Handler(void)  { fz_fault_reset(); }
void BusFault_Handler(void)   { fz_fault_reset(); }
void UsageFault_Handler(void) { fz_fault_reset(); }

static uint32_t s_reset_flags;
static uint8_t  s_crash_note;

/* Deep-sleep (Stop 2) gating state - declared up here so fz_power_boot_log
 * (which reads the deepsleep opt-in) and idle_policy_cb both see it. */
static volatile bool s_vbus = true;      /* conservative until first I2C read */
static volatile bool s_deep_ok;
static bool          s_deep_enabled;     /* off unless settings deepsleep=1 */
static volatile bool s_tickless_blocked;

void hal_crash_note(uint8_t code)
{
    s_crash_word = CRASH_MAGIC | code;
}

void fz_power_boot_log(void)
{
    /* RCC->CSR flags: PINRSTF, BORRSTF, SFTRSTF (NVIC_SystemReset),
     * IWDGRSTF, WWDGRSTF, LPWRRSTF - top byte of CSR. */
    fantasi_log(LOG_INFO, "resetflags 0x%02lx crash 0x%02x",
                (unsigned long)(s_reset_flags >> 24), s_crash_note);

    /* Deep sleep (Stop 2): on by default now that the CPU1/CPU2 protocol is
     * bench-validated (7 real Stop 2 + 23 CSleep episodes, no hang/fault).
     * The gate keeps it battery-only (VBUS absent), so USB operation is never
     * affected; `deepsleep=0` is the escape hatch. Read here, not in
     * fz_power_init: storage isn't up during hal_init. */
    char d[4] = "1";
    hal_settings_get("deepsleep", d, sizeof(d));
    s_deep_enabled = (d[0] != '0');
}

/* ---- Buttons: blocking wait (overrides the weak poll fallback) ---- */

#define BTN_WAITERS 4
static volatile TaskHandle_t s_btn_waiters[BTN_WAITERS];

void hal_button_wait(uint32_t timeout_ms)
{
    int slot = -1;
    taskENTER_CRITICAL();
    for (int i = 0; i < BTN_WAITERS; i++) {
        if (!s_btn_waiters[i]) { s_btn_waiters[i] = xTaskGetCurrentTaskHandle(); slot = i; break; }
    }
    taskEXIT_CRITICAL();
    if (slot < 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
    }
    /* No pre-check of pin state here (six pins, mixed polarity): the callers
     * poll their own button after every wake, and a press raced in before
     * registration re-fires EXTI on the next edge. The bounded timeout in
     * pwr_button_task's caller-side loop covers the residual window. */
    TickType_t t = (timeout_ms == UINT32_MAX) ? portMAX_DELAY
                                              : pdMS_TO_TICKS(timeout_ms);
    ulTaskNotifyTake(pdTRUE, t);
    s_btn_waiters[slot] = NULL;
}

static volatile TickType_t s_last_activity;
static volatile bool       s_idle;

void fz_power_buttons_wake(BaseType_t *woken)
{
    s_last_activity = xTaskGetTickCountFromISR();
    s_idle = false;
    for (int i = 0; i < BTN_WAITERS; i++) {
        TaskHandle_t t = s_btn_waiters[i];
        if (t) vTaskNotifyGiveFromISR(t, woken);
    }
}

/* Core's host-interaction hint (CLI dispatch, proto frames - task context). */
void hal_power_activity(void)
{
    s_last_activity = xTaskGetTickCount();
    s_idle = false;
}

/* ---- Idle policy: slow advertising ---- */

#define IDLE_AFTER_MS    30000
#define POLICY_PERIOD_MS 1000

bool fz_power_adv_slow(void)
{
    return s_idle;
}

static void idle_policy_cb(TimerHandle_t timer)
{
    (void)timer;

    /* Host-side activity edges: a new USB mount or BLE connection counts. */
    static int prev_usb = -1, prev_ble = -1;
    int usb = pwr_client_votes(PWR_CLIENT_USB_ACTIVE);
    int ble = pwr_client_votes(PWR_CLIENT_BLE_LINK);
    if ((prev_usb >= 0 && usb > prev_usb) || (prev_ble >= 0 && ble > prev_ble))
        hal_power_activity();
    prev_usb = usb; prev_ble = ble;

    uint32_t idle_ms = (uint32_t)(xTaskGetTickCount() - s_last_activity)
                       * 1000u / configTICK_RATE_HZ;
    bool idle = idle_ms >= IDLE_AFTER_MS;
    if (idle != s_idle) {
        s_idle = idle;
        /* Re-issue background advertising at the new interval, but only when
         * unconnected - with a link up the change simply applies on the next
         * advertising restart. (Task context here: HCI is fine.) */
        if (ble == 0)
            ble_adv_refresh();
    }

    /* Deep-sleep gate, refreshed awake so the sleep path never touches I2C:
     * VBUS absent (poll the charger every 2 s - the raw polled I2C driver has
     * no lock, so keep the duty low vs the other I2C users) + CPU2 healthy +
     * LSE running for the LPTIM1 wake timer. Conservative default: deep stays
     * off until VBUS is confirmed absent. */
    static int vbus_div;
    if (++vbus_div >= 2) {
        vbus_div = 0;
        bool vbus = fz_hal_vbus_present();
        /* USB cable returned after a battery session that ran Stop 2 (HSI48, the
         * USB clock, is off): restore it and bounce the connection so the FS
         * peripheral re-enumerates. Gated on HSI48-off so a plug-in without a
         * prior deep sleep (USB clock still live) doesn't needlessly re-enum. */
        if (vbus && !s_vbus && !(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {
            fz_usb_clock_restore();
            hal_usb_reenumerate();
        }
        s_vbus = vbus;
    }
    /* Deep sleep (Stop 2) is allowed only on battery (VBUS absent), with CPU2
     * alive and the LSE running for the LPTIM1 wake timer. The CPU1/CPU2
     * entry/exit protocol is bench-validated (30-episode burst: 7 real Stop 2
     * + 23 CSleep, no hang/fault; real 8-48 ms sleeps waking on radio events
     * like stock). USB present always keeps it at light sleep, so bench/USB
     * operation is never affected. `deepsleep=0` disables it entirely. */
    s_deep_ok = s_deep_enabled && !s_vbus && ble_cpu2_running()
              && (RCC->BDCR & RCC_BDCR_LSERDY);
}

/* =====================================================================
 * Phase B: Stop 2 deep sleep (the CPU1/CPU2 protocol, ported from stock
 * furi_hal_power_deep_sleep / furi_hal_os / furi_hal_idle_timer).
 *
 * Engages only when every gate holds (checked awake, cached by the policy
 * timer where I2C is involved):
 *   - vote depth == HAL_SLEEP_DEEP (no USB session, no BLE link, no flash op,
 *     no app, no SHCI round-trip in flight),
 *   - CPU2 booted and healthy (ble_cpu2_running - mirrors stock's
 *     furi_hal_bt_is_alive gate; a single-core system stays on light sleep),
 *   - VBUS absent (BQ25896 charger, polled over I2C - the WB55 USB core
 *     cannot sense VBUS itself). USB present => light sleep only.
 *   - LSE running (LPTIM1 wake timer clock; ble_init starts it).
 *
 * The Stop 2 entry/exit protocol is stock's, verbatim in structure:
 * HSEM #3 (RCC) around every SYSCLK change because CPU2 also manipulates
 * RCC on wake; HSEM #4 (ENTRY_STOP) to arbitrate who prepares Stop; switch
 * HSE->HSI16 before WFI only when CPU2 is not awake (if CPU2 runs, WFI is
 * just CSleep and the clock stays put); wake lands on HSI16 (STOPWUCK) and
 * is switched back to HSE under HSEM #3. No PLL step - Fantasi runs fixed
 * HSE 32 MHz, which is why this port is simpler than stock (no
 * SHCI_C2_SetSystemClock renegotiation).
 * ===================================================================== */

/* RM0434 field values the CMSIS device header leaves undefined (it only
 * provides the masks / _Pos). SW/SWS: 10=HSE, 01=HSI16. LPMS: 010=Stop 2. */
#define RCC_SW_HSI   (1U << RCC_CFGR_SW_Pos)
#define RCC_SW_HSE   (2U << RCC_CFGR_SW_Pos)
#define RCC_SWS_HSI  (1U << RCC_CFGR_SWS_Pos)
#define RCC_SWS_HSE  (2U << RCC_CFGR_SWS_Pos)
#define LPMS_STOP2_1 (2U << PWR_CR1_LPMS_Pos)
#define LPMS_STOP2_2 (2U << PWR_C2CR1_LPMS_Pos)

#define HSEM_COREID_CPU1    0x04U
#define HSEM_RCC_SEM        3   /* CFG_HW_RCC_SEMID        */
#define HSEM_ENTRY_STOP_SEM 4   /* CFG_HW_ENTRY_STOP_MODE_SEMID */

static inline int hsem_lock(int sem)   /* 0 = locked OK */
{
    return (HSEM->RLR[sem] !=
            (HSEM_RLR_LOCK_Msk | (HSEM_COREID_CPU1 << HSEM_RLR_COREID_Pos))) ? 1 : 0;
}
static inline void hsem_unlock(int sem)
{
    HSEM->R[sem] = (HSEM_COREID_CPU1 << HSEM_R_COREID_Pos);
}

/* Stock furi_hal_clock_switch_hse2hsi / hsi2hse, register-level. */
static void clk_hse_to_hsi(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SW_HSI);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_SWS_HSI) {}
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_0WS);
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0WS) {}
}
static void clk_hsi_to_hse(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1WS);
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_1WS) {}
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SW_HSE);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_SWS_HSE) {}
}

/* ---- LPTIM1 on LSE: the Stop 2 wake timer (stock furi_hal_idle_timer) ---- */

#define LPTIM_MAX_CNT 0xFFF0u   /* 16-bit @ 32768 Hz -> ~2 s per episode */

static void lptim_start(uint32_t count)
{
    count--;
    LPTIM1->CR = LPTIM_CR_ENABLE;
    while (!(LPTIM1->CR & LPTIM_CR_ENABLE)) {}
    LPTIM1->IER = LPTIM_IER_CMPMIE;      /* stock's order: after enable */
    LPTIM1->CMP = count - 3;             /* margin for the ARRM quirk */
    LPTIM1->ARR = count;
    LPTIM1->CR |= LPTIM_CR_SNGSTRT;
}

static uint32_t lptim_get_cnt(void)
{
    /* Async clock domain: read until two consecutive reads agree. */
    uint32_t c = LPTIM1->CNT, s = LPTIM1->CNT;
    while (c != s) { c = s; s = LPTIM1->CNT; }
    return c;
}

static void lptim_reset(void)
{
    /* Hard bus reset - per errata the only reliable stop (stock does the
     * same between every use). */
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;
    NVIC_ClearPendingIRQ(LPTIM1_IRQn);
}

void LPTIM1_IRQHandler(void)
{
    LPTIM1->ICR = 0x7F;   /* clear all - the IRQ only has to wake the core */
}

static uint32_t s_acc_units;             /* sub-tick residual: 1 ms = 32768 */
static pwr_wake_t s_deep_wake;           /* wake cause, captured before lptim_reset clears it */

/* Pending-IRQ scan (stock furi_hal_os_is_pending_irq): NVIC + both EXTI
 * banks - an already-pending wake source means don't bother sleeping. */
static bool pending_irq(void)
{
    for (int i = 0; i < 2; i++)
        if (NVIC->ISPR[i]) return true;
    if (EXTI->PR1) return true;
    if (EXTI->PR2) return true;
    return false;
}

static pwr_wake_t deep_wake_reason(void)
{
    if (NVIC_GetPendingIRQ(LPTIM1_IRQn) || (LPTIM1->ISR & LPTIM_ISR_CMPM))
        return PWR_WAKE_TIMER;
    if (EXTI->PR1 & ((1U<<3)|(1U<<6)|(1U<<10)|(1U<<11)|(1U<<12)|(1U<<13)))
        return PWR_WAKE_BUTTON;
    if (NVIC_GetPendingIRQ(IPCC_C1_RX_IRQn) || NVIC_GetPendingIRQ(IPCC_C1_TX_IRQn))
        return PWR_WAKE_RADIO;
    if (NVIC_GetPendingIRQ(USB_LP_IRQn) || NVIC_GetPendingIRQ(USB_HP_IRQn))
        return PWR_WAKE_USB;
    return PWR_WAKE_OTHER;
}

/* One Stop 2 episode, PRIMASK held throughout (stock structure). Returns
 * whole ticks to credit. */
static uint32_t stop2_sleep_ticks(uint32_t expected_ticks)
{
    uint32_t want_cnt = (uint32_t)((uint64_t)expected_ticks * 32768u
                                   / configTICK_RATE_HZ);
    if (want_cnt > LPTIM_MAX_CNT) want_cnt = LPTIM_MAX_CNT;
    if (want_cnt < 8) return 0;   /* too short to be worth a Stop entry */

    /* Stop the tick; capture the consumed fraction of the current period
     * (SysTick freezes in Stop 2 - measured time comes from LPTIM1). */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    uint32_t frac_cyc = (SysTick->LOAD & SysTick_LOAD_RELOAD_Msk) - SysTick->VAL;

    lptim_start(want_cnt);

    /* ---- stock furi_hal_power_deep_sleep protocol ---- */
    while (hsem_lock(HSEM_RCC_SEM) != 0) {}
    if (hsem_lock(HSEM_ENTRY_STOP_SEM) == 0) {
        if (PWR->EXTSCR & (PWR_EXTSCR_C2DS | PWR_EXTSCR_C2SBF)) {
            /* CPU2 already in CStop/standby: no arbitration needed. Switch to
             * HSI16 for a real Stop 2. */
            hsem_unlock(HSEM_ENTRY_STOP_SEM);
            clk_hse_to_hsi();
        }
        /* else CPU2 awake: hold ENTRY_STOP, stay on HSE (CPU1 CSleep only) */
    } else {
        /* CPU2 owns ENTRY_STOP (it is mid Stop-entry): prepare for real
         * Stop 2 - wake must come up on HSI16 (STOPWUCK). */
        clk_hse_to_hsi();
    }
    hsem_unlock(HSEM_RCC_SEM);

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
    __ISB();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    hsem_unlock(HSEM_ENTRY_STOP_SEM);   /* unconditional, stock-style */
    while (hsem_lock(HSEM_RCC_SEM) != 0) {}
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_SWS_HSI)
        clk_hsi_to_hse();
    hsem_unlock(HSEM_RCC_SEM);
    /* ---- end stock protocol ---- */

    uint32_t cnt  = lptim_get_cnt();
    bool cmpm = (LPTIM1->ISR & LPTIM_ISR_CMPM) != 0;
    bool arrm = (LPTIM1->ISR & LPTIM_ISR_ARRM) != 0;
    if (cmpm && arrm) cnt += want_cnt;   /* stock's wrap case */
    /* Capture the wake cause now: PRIMASK is still held (the LPTIM ISR hasn't
     * run), so the timer's pending flag is intact - but lptim_reset() below
     * clears it, and the caller's pwr_note_wake runs after that, which is why
     * timer wakes were mis-attributed to PWR_WAKE_OTHER. */
    s_deep_wake = deep_wake_reason();
    lptim_reset();

    /* Credit via the CU-proven fixed-point accumulator (1 ms = 32768 units;
     * 1 LSE tick = 1000 units; 1 CPU cycle @32 MHz = 1.024 units). */
    uint64_t total = (uint64_t)s_acc_units
                   + (uint64_t)cnt * 1000u
                   + (uint64_t)frac_cyc * 1024u / 1000u;
    uint32_t step = (uint32_t)(total / 32768u);
    s_acc_units   = (uint32_t)(total % 32768u);
    if (step > expected_ticks) step = expected_ticks;

    /* Tick back on: full period from now; sub-ms residual carries over. */
    SysTick->VAL  = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    return step;
}

/* ---- The tickless entry: overrides the kernel's weak implementation ----
 *
 * Light path: the kernel's own SysTick-reprogramming algorithm, copied from
 * port.c (which stays authoritative for the subtle pending-tick races) with
 * local constants - SysTick keeps counting through plain WFI on STM32, so
 * timekeeping is exact. Deep path: LPTIM1 + Stop 2 above. */

#define TIMER_COUNTS_PER_TICK (configCPU_CLOCK_HZ / configTICK_RATE_HZ)  /* 32000 */
#define MAX_SUPPRESSED_TICKS  (0xFFFFFFu / TIMER_COUNTS_PER_TICK)
#define STOPPED_TIMER_COMP    45u

static void light_suppress(TickType_t xExpectedIdleTime)
{
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements,
             ulSysTickDecrementsLeft;

    if (xExpectedIdleTime > MAX_SUPPRESSED_TICKS)
        xExpectedIdleTime = MAX_SUPPRESSED_TICKS;

    __asm volatile ("cpsid i" ::: "memory");
    __asm volatile ("dsb");
    __asm volatile ("isb");

    if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
        __asm volatile ("cpsie i" ::: "memory");
        return;
    }

    /* Stop SysTick (keep CLKSOURCE|TICKINT so COUNTFLAG state is preserved). */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

    ulSysTickDecrementsLeft = SysTick->VAL;
    if (ulSysTickDecrementsLeft == 0)
        ulSysTickDecrementsLeft = TIMER_COUNTS_PER_TICK;

    ulReloadValue = ulSysTickDecrementsLeft
                  + (TIMER_COUNTS_PER_TICK * (xExpectedIdleTime - 1u));
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
        ulReloadValue -= TIMER_COUNTS_PER_TICK;
    }
    if (ulReloadValue > STOPPED_TIMER_COMP)
        ulReloadValue -= STOPPED_TIMER_COMP;

    SysTick->LOAD = ulReloadValue;
    SysTick->VAL  = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    pwr_note_light_sleep();
    __asm volatile ("dsb" ::: "memory");
    __asm volatile ("wfi");
    __asm volatile ("isb");

    /* Let the waking interrupt run, then re-mask for the accounting. */
    __asm volatile ("cpsie i" ::: "memory");
    __asm volatile ("dsb");
    __asm volatile ("isb");
    __asm volatile ("cpsid i" ::: "memory");
    __asm volatile ("dsb");
    __asm volatile ("isb");

    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        /* The tick interrupt ended the sleep: a new tick period started. */
        uint32_t ulCalculatedLoadValue = (TIMER_COUNTS_PER_TICK - 1u)
                                       - (ulReloadValue - SysTick->VAL);
        if ((ulCalculatedLoadValue <= STOPPED_TIMER_COMP) ||
            (ulCalculatedLoadValue > TIMER_COUNTS_PER_TICK))
            ulCalculatedLoadValue = TIMER_COUNTS_PER_TICK - 1u;
        SysTick->LOAD = ulCalculatedLoadValue;
        ulCompleteTickPeriods = xExpectedIdleTime - 1u;
    } else {
        /* Something else woke us. */
        ulSysTickDecrementsLeft = SysTick->VAL;
        ulCompletedSysTickDecrements =
            (xExpectedIdleTime * TIMER_COUNTS_PER_TICK) - ulSysTickDecrementsLeft;
        ulCompleteTickPeriods = ulCompletedSysTickDecrements / TIMER_COUNTS_PER_TICK;
        SysTick->LOAD = ((ulCompleteTickPeriods + 1u) * TIMER_COUNTS_PER_TICK)
                      - ulCompletedSysTickDecrements;
    }

    /* Restart with the partial-period LOAD, step the kernel, then restore
     * the normal period (SysTick reloads LOAD on the next wrap). */
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    vTaskStepTick(ulCompleteTickPeriods);
    SysTick->LOAD = TIMER_COUNTS_PER_TICK - 1u;

    __asm volatile ("cpsie i" ::: "memory");
}

/* With a live BLE link, cap each Stop 2 episode well under the link's 2 s
 * supervision timeout so CPU1 wakes at least this often to service the BLE
 * stack (drain/release CPU2's event buffers, handle data). A single slow or
 * missed wake then can't stack up toward a supervision-timeout link drop.
 * Between events it's still ~95% asleep. */
#define BLE_DEEP_MAX_MS 500

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    if (s_tickless_blocked) return;
    if (xTaskGetTickCount() < pdMS_TO_TICKS(PWR_BOOT_GRACE_MS)) return;

    /* A live BLE link does not force light sleep: CPU2 maintains the radio link
     * autonomously and wakes CPU1 via IPCC (an EXTI Stop 2 wake source) for each
     * host event, and stop2_sleep_ticks's HSEM protocol keeps CPU1 in CSleep
     * (not real Stop 2) whenever CPU2 is mid-event. So CPU1 can Stop 2 between
     * connection events without dropping the link. USB/flash/sd-op still force
     * light (they need the core clock or must not be interrupted). */
    hal_sleep_depth_t allowed = pwr_allowed_depth_except(PWR_CLIENT_BLE_LINK);
    if (allowed == HAL_SLEEP_NONE) return;

    bool deep = s_deep_ok && allowed == HAL_SLEEP_DEEP;
    if (!deep) {
        light_suppress(xExpectedIdleTime);
        return;
    }

    /* Deep: Stop 2 with the LPTIM1 wake timer. */
    __disable_irq();
    __DSB(); __ISB();
    if (eTaskConfirmSleepModeStatus() == eAbortSleep || pending_irq()) {
        __enable_irq();
        return;
    }
    TickType_t idle = xExpectedIdleTime;
    if (pwr_client_votes(PWR_CLIENT_BLE_LINK) > 0
        && idle > pdMS_TO_TICKS(BLE_DEEP_MAX_MS))
        idle = pdMS_TO_TICKS(BLE_DEEP_MAX_MS);
    uint32_t stepped = stop2_sleep_ticks(idle);
    pwr_note_wake(s_deep_wake);   /* captured inside, before lptim_reset cleared it */
    if (stepped > 0) {
        vTaskStepTick(stepped);
        pwr_note_deep_sleep(stepped);
    }
    __enable_irq();
}

void fz_power_tickless_block(bool block)
{
    s_tickless_blocked = block;
}

/* ---- Init ---- */

void fz_power_init(void)
{
    s_reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;              /* clear reset flags */
    if ((s_crash_word & 0xFFFFFF00u) == CRASH_MAGIC)
        s_crash_note = (uint8_t)s_crash_word;
    s_crash_word = 0;

    /* Stop-mode plumbing (idempotent, harmless while only light sleep runs):
     * - wake from Stop lands on HSI16, never MSI (STOPWUCK) - the restore
     *   path in stop2_sleep_ticks assumes it;
     * - both cores request Stop 2 (the effective mode is the shallower of
     *   the two LPMS fields; stock sets both at power init). */
    RCC->CFGR |= RCC_CFGR_STOPWUCK;
    MODIFY_REG(PWR->CR1,   PWR_CR1_LPMS,   LPMS_STOP2_1);
    MODIFY_REG(PWR->C2CR1, PWR_C2CR1_LPMS, LPMS_STOP2_2);

    /* (Stop-mode debug via DBGMCU is intentionally not enabled: it raises Stop
     * current, and the BMP can't halt this secure core anyway - diagnosis is
     * via the .noinit breadcrumb trace below, which needs no live debugger.) */

    /* LPTIM1 on LSE: the Stop 2 wake timer (LPTIM1 is the only LPTIM alive
     * in Stop 2). Kernel clock + sleep clock + lowest-priority IRQ. */
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL,
               3U << RCC_CCIPR_LPTIM1SEL_Pos);          /* LSE */
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
    (void)RCC->APB1ENR1;
    RCC->APB1SMENR1 |= RCC_APB1SMENR1_LPTIM1SMEN;
    NVIC_SetPriority(LPTIM1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(LPTIM1_IRQn);

    /* Stop 2 wake routing, all in the IMR2 bank (lines 32-63): LPTIM1 (line 33,
     * the wake timer - without it a battery idle whose only wake is the LPTIM
     * timeout, not a radio event, never wakes from Stop 2) and IPCC CPU1 RX/TX
     * (lines 36/37 - radio events must wake CPU1 or CPU2's event pool starves).
     * Buttons are on EXTI already. */
    EXTI->IMR2 |= (1U << (33 - 32)) | (1U << (36 - 32)) | (1U << (37 - 32));

    s_last_activity = 0;

    TimerHandle_t t = xTimerCreate("idlepol", pdMS_TO_TICKS(POLICY_PERIOD_MS),
                                   pdTRUE, NULL, idle_policy_cb);
    if (t) xTimerStart(t, 0);
}
