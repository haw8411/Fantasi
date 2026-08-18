/* Fantasi / Chameleon Ultra - low-power implementation.
 *
 * Two layers, gated by core/power.c's vote API:
 *
 *  1. Tickless idle (this file's cu_suppress_ticks_and_sleep, wired via
 *     portSUPPRESS_TICKS_AND_SLEEP): for idle windows >= 2 ticks, SysTick is
 *     stopped, RTC1 (32.768 kHz LFXO, 24-bit -> up to ~511 s per episode) is
 *     armed as the wake timer, and the CPU sleeps in System ON low-power
 *     (~2-3 uA + peripherals). On wake the elapsed RTC time is credited with
 *     vTaskStepTick. Modeled on Nordic's port_cmsis_systick.c:
 *       - With the SoftDevice active we must not hold PRIMASK across the
 *         sleep (SD radio interrupts have hard deadlines). Instead all *app*
 *         interrupts are disabled at the NVIC (the SD-reserved set stays
 *         live), PRIMASK is released, and a SEVONPEND __WFE loop sleeps until
 *         an app interrupt pends (a disabled-but-pending IRQ still sets ISPR
 *         and, with SEVONPEND, generates a wake event) or the RTC fires.
 *       - Without the SoftDevice a classic PRIMASK-held __WFI is used.
 *
 *  2. Idle policy (1 s FreeRTOS timer): after ~30 s without activity, dim the
 *     launcher LEDs (the launcher task does the actual fade - it owns the
 *     pins) and drop BLE advertising from 100 ms to 1 s intervals; after
 *     `power off-timeout` (default 5 min) of no USB/BLE/app/VBUS, System OFF
 *     via hal_shutdown(). Any button press / host attach restores everything.
 *
 * Button wake: both buttons (A=P0.26, B=P1.02) get GPIO SENSE + the single
 * GPIOTE PORT event - 2.4 uA idle vs 17 uA for GPIOTE IN events (nRF52840 PS
 * v1.1 5.3), and SENSE doubles as the System OFF wake source. */

#include "nrf.h"
#include "power.h"
#include "ble.h"
#include "ble_serial.h"
#include "../../hal/hal.h"
#include "../../hal/hal_power.h"
#include "../../core/app_run.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* ---- Pins / IRQ layout ---- */

#define BTN_A_PORT NRF_P0
#define BTN_A_PIN  26
#define BTN_B_PORT NRF_P1
#define BTN_B_PIN  2

/* SoftDevice-reserved interrupts (nrf_nvic.h __NRF_NVIC_SD_IRQS_0): these
 * must stay enabled while we sleep - the SD's radio timing depends on them.
 * IRQ 30 is the SD's reserved NVMC slot (no real NVMC interrupt exists). */
#define SD_IRQ_MASK0                                                    \
    ((1u << POWER_CLOCK_IRQn) | (1u << RADIO_IRQn) |                    \
     (1u << TIMER0_IRQn)      | (1u << RTC0_IRQn)  |                    \
     (1u << TEMP_IRQn)        | (1u << RNG_IRQn)   |                    \
     (1u << ECB_IRQn)         | (1u << CCM_AAR_IRQn) |                  \
     (1u << SWI5_EGU5_IRQn)   | (1u << 30))

/* ---- State ---- */

static volatile bool       s_vbus;
static volatile bool       s_led_idle;
static volatile TickType_t s_last_activity;
static bool                s_rtc_running;    /* latched once RTC1 counts (LFXO up) */
static bool                s_lfclk_kicked;   /* manual LFXO start issued (SD-off case) */

/* No sleep machinery during early boot: USB enumeration + SoftDevice bring-up
 * run on a fully-awake system, and a sleep-path regression stays reachable
 * over the CLI instead of bricking the boot. */
#define PWR_BOOT_GRACE_MS 5000

/* Sub-tick residual carried between tickless episodes so long-run timekeeping
 * stays exact. Units: 1 ms = 32768, 1 RTC tick = 1000, 1 CPU cycle = 0.512. */
#define UNITS_PER_MS 32768u
static uint32_t s_acc_units;

/* Tasks blocked in hal_button_wait (launcher + power-button task). */
#define BTN_WAITERS 4
static volatile TaskHandle_t s_btn_waiters[BTN_WAITERS];

/* ---- Buttons ---- */

static bool any_button_down(void)
{
    return (BTN_A_PORT->IN & (1UL << BTN_A_PIN)) ||
           (BTN_B_PORT->IN & (1UL << BTN_B_PIN));
}

/* Input + pull-down + SENSE_High: SENSE feeds the GPIOTE PORT event (running)
 * and the System OFF wake logic. Other button-pin configs in hal.c must keep
 * the SENSE bits too, or the wake path silently dies. */
static void button_pin_cfg(NRF_GPIO_Type *port, uint32_t pin)
{
    port->PIN_CNF[pin] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
        (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos);
}

static void button_waiters_wake(bool from_isr)
{
    BaseType_t woken = pdFALSE;
    for (int i = 0; i < BTN_WAITERS; i++) {
        TaskHandle_t t = s_btn_waiters[i];
        if (!t) continue;
        if (from_isr) vTaskNotifyGiveFromISR(t, &woken);
        else          xTaskNotifyGive(t);
    }
    if (from_isr) portYIELD_FROM_ISR(woken);
}

/* Overrides the poll-fallback weak in core/power.c. */
void hal_button_wait(uint32_t timeout_ms)
{
    int slot = -1;
    taskENTER_CRITICAL();
    for (int i = 0; i < BTN_WAITERS; i++) {
        if (!s_btn_waiters[i]) { s_btn_waiters[i] = xTaskGetCurrentTaskHandle(); slot = i; break; }
    }
    taskEXIT_CRITICAL();
    if (slot < 0) {                       /* can't happen with 2 waiters; poll */
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
    }
    /* Re-check after registering: a press that raced in ahead of the PORT
     * event registration must not sleep the full timeout. */
    if (!any_button_down()) {
        TickType_t t = (timeout_ms == UINT32_MAX) ? portMAX_DELAY
                                                  : pdMS_TO_TICKS(timeout_ms);
        ulTaskNotifyTake(pdTRUE, t);
    }
    s_btn_waiters[slot] = NULL;
}

void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_PORT) {
        NRF_GPIOTE->EVENTS_PORT = 0;
        /* Clear pin latches too: if DETECTMODE were ever LDETECT (bootloader
         * leftovers), an uncleared LATCH bit re-asserts DETECT forever - an
         * interrupt storm that starves every task. Write-1-to-clear. */
        NRF_P0->LATCH = NRF_P0->LATCH;
        NRF_P1->LATCH = NRF_P1->LATCH;
        /* Press edge: note activity (restores LEDs/fast adv via the policy)
         * and wake every button waiter. SENSE stays asserted while held, so
         * PORT won't re-fire until release - the waiters poll the hold. */
        cu_power_note_activity();
        button_waiters_wake(true);
    }
}

/* ---- Activity / idle policy ---- */

void cu_power_vbus(bool present)
{
    s_vbus = present;
}

bool cu_power_led_idle(void)
{
    return s_led_idle;
}

/* Core's user/host-interaction hint (CLI dispatch, proto frames). */
void hal_power_activity(void)
{
    cu_power_note_activity();
}

void cu_power_note_activity(void)
{
    s_last_activity = xTaskGetTickCountFromISR();   /* safe from tasks too */
    if (s_led_idle) {
        s_led_idle = false;
        /* Restore fast advertising from the next task-context poll: doing
         * SVCs from here (possibly ISR) is not allowed; the policy timer
         * corrects it within a second as well. */
    }
}

#define LED_IDLE_MS      30000   /* fade LEDs + slow advertising after 30 s */
#define POLICY_PERIOD_MS 1000

static void idle_policy_cb(TimerHandle_t timer)
{
    (void)timer;

    /* Host-side activity edges: a new USB mount or BLE connection counts. */
    static int prev_usb = -1, prev_ble = -1;
    int usb = pwr_client_votes(PWR_CLIENT_USB_ACTIVE);
    int ble = pwr_client_votes(PWR_CLIENT_BLE_LINK);
    if ((prev_usb >= 0 && usb > prev_usb) || (prev_ble >= 0 && ble > prev_ble))
        cu_power_note_activity();
    prev_usb = usb; prev_ble = ble;

    uint32_t idle_ms = (uint32_t)(xTaskGetTickCount() - s_last_activity)
                       * 1000u / configTICK_RATE_HZ;

    bool idle = idle_ms >= LED_IDLE_MS;
    if (idle != s_led_idle) {
        s_led_idle = idle;
        button_waiters_wake(false);       /* launcher re-evaluates its LEDs */
    }
    /* Advertising interval follows the same idle state (task context here,
     * so SVCs are fine). Cheap no-op when unchanged. */
    ble_serial_set_adv_slow(idle && ble == 0);

    /* Auto power off: only on battery (no VBUS - a charging device must not
     * switch off), with no USB/BLE session, no app, no flash op in flight,
     * and only if an off-timeout is configured. hal_shutdown() never returns. */
    uint32_t off_s = pwr_off_timeout_s();
    if (off_s && !s_vbus && usb == 0 && ble == 0 &&
        pwr_client_votes(PWR_CLIENT_FLASH) == 0 &&
        !app_is_running() &&
        idle_ms >= off_s * 1000u)
        hal_shutdown();
}

/* ---- Light sleep ----
 *
 * Deliberately no idle-hook __WFE: on nRF52 the CPU clock stops in WFE and
 * SysTick stops with it, so an idle-hook sleep freezes the tick for however
 * long the sleep lasts. Idle windows >= 2 ticks sleep via the tickless hook
 * below (RTC1-measured and credited); sub-2-tick windows just spin, which
 * is rare and brief. */

/* ---- Tickless idle: RTC1 wake timer ---- */

/* Clears whatever RTC1 latched; the real bookkeeping happens inline in
 * cu_suppress_ticks_and_sleep (the IRQ only has to wake the core). */
void RTC1_IRQHandler(void)
{
    NRF_RTC1->INTENCLR = RTC_INTENCLR_COMPARE0_Msk;
    NRF_RTC1->EVENTS_COMPARE[0] = 0;
    NRF_RTC1->EVENTS_OVRFLW = 0;
}

static pwr_wake_t wake_reason(uint32_t pend0, uint32_t pend1)
{
    if (pend0 & (1u << GPIOTE_IRQn))                        return PWR_WAKE_BUTTON;
    if ((pend1 & (1u << (USBD_IRQn - 32))) ||
        (pend0 & (1u << POWER_CLOCK_IRQn)))                 return PWR_WAKE_USB;
    if (pend0 & (1u << SWI2_EGU2_IRQn))                     return PWR_WAKE_RADIO;
    if (NRF_RTC1->EVENTS_COMPARE[0] ||
        (pend0 & (1u << RTC1_IRQn)))                        return PWR_WAKE_TIMER;
    return PWR_WAKE_OTHER;
}

void cu_suppress_ticks_and_sleep(uint32_t expected_idle_ticks)
{
    /* Sleep disabled (or a NONE-depth condition): leave the tick running. */
    if (xTaskGetTickCount() < pdMS_TO_TICKS(PWR_BOOT_GRACE_MS)) return;
    if (pwr_allowed_depth() == HAL_SLEEP_NONE) return;

    /* RTC1 only counts once an LFCLK runs. The clock's owner matters: with
     * BLE on (the default), the SoftDevice starts the LFXO itself from the
     * sd_lfclk_cfg in cu_ble_sd_init - the app must not pre-start it (the SD
     * expects to own the clock; racing it can stall sd_softdevice_enable).
     * Only when the SD never came up (persisted ble=0) do we start the LFXO
     * by hand, lazily, here - CLOCK registers are unrestricted with no SD. */
    if (!s_rtc_running) {
        if (NRF_RTC1->COUNTER == 0) {
            if (!s_lfclk_kicked && !cu_ble_sd_is_active()) {
                s_lfclk_kicked = true;
                NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
                NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
                NRF_CLOCK->TASKS_LFCLKSTART = 1;
            }
            return;
        }
        s_rtc_running = true;
    }

    /* The SoC-event drain owns the SD event queue during flash ops; the
     * 1 ms-poll waits involved never present a >=2 tick idle window anyway,
     * but be explicit: no long sleeps while a flash op is in flight. */
    if (pwr_client_votes(PWR_CLIENT_FLASH) > 0) return;

    __disable_irq();

    if (eTaskConfirmSleepModeStatus() == eAbortSleep ||
        (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)) {
        __enable_irq();
        return;
    }

    /* Stop the tick; capture the consumed fraction of the current tick
     * period so it isn't lost (SysTick stops counting in System ON sleep). */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    uint32_t frac_cyc = (SysTick->LOAD & SysTick_LOAD_RELOAD_Msk) - SysTick->VAL;

    /* Arm RTC1 COMPARE0 for the expected idle time (floor -> never oversleep;
     * 24-bit counter caps one episode at ~511 s, the kernel just calls again). */
    uint32_t r0 = NRF_RTC1->COUNTER;
    uint64_t want_rtc = (uint64_t)expected_idle_ticks * 32768u / configTICK_RATE_HZ;
    if (want_rtc > 0x00FFFF00u) want_rtc = 0x00FFFF00u;
    if (want_rtc < 3) {                       /* nRF52 RTC CC minimum margin */
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        __enable_irq();
        return;
    }
    NRF_RTC1->CC[0] = (r0 + (uint32_t)want_rtc) & 0x00FFFFFFu;
    NRF_RTC1->EVENTS_COMPARE[0] = 0;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    uint32_t pend0 = 0, pend1 = 0;

    if (cu_ble_sd_is_active()) {
        /* Mask app interrupts at the NVIC, keep SD ones live, drop PRIMASK,
         * and WFE-sleep until an app IRQ pends (SEVONPEND) or RTC1 fires.
         * SD interrupts run normally in this window; their handlers may pend
         * app IRQs (SWI2), which exits the loop.
         * Re-assert SEVONPEND every entry: the SoftDevice does not preserve
         * it, and without it a pended-but-masked app IRQ (including our RTC1
         * wake timer) generates no WFE event - the loop then oversleeps until
         * the next radio event and the clock falls behind wall time. */
        SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
        uint32_t app0 = NVIC->ISER[0] & ~SD_IRQ_MASK0;
        uint32_t app1 = NVIC->ISER[1];
        NVIC->ICER[0] = app0;
        NVIC->ICER[1] = app1;
        __DSB(); __ISB();
        __enable_irq();

        for (;;) {
            pend0 = NVIC->ISPR[0] & app0;
            pend1 = NVIC->ISPR[1] & app1;
            if (pend0 || pend1) break;
            __DSB();
            __WFE();
        }

        __disable_irq();
        NVIC->ISER[0] = app0;
        NVIC->ISER[1] = app1;
    } else {
        /* No SoftDevice: classic PRIMASK-held WFI - wakes on any enabled
         * interrupt becoming pending without taking it. */
        __DSB();
        __WFI();
        pend0 = NVIC->ISPR[0];
        pend1 = NVIC->ISPR[1];
    }

    pwr_note_wake(wake_reason(pend0, pend1));

    /* Disarm the wake timer (the pended IRQ may still run its handler after
     * PRIMASK release; it only clears events). */
    NRF_RTC1->INTENCLR = RTC_INTENCLR_COMPARE0_Msk;
    NRF_RTC1->EVENTS_COMPARE[0] = 0;

    /* Credit the slept time. All bookkeeping in 1/32768-ms units via the
     * residual accumulator so rounding never drifts the clock: RTC ticks map
     * to 1000 units, CPU cycles (64 MHz) to 0.512 units each. */
    uint32_t d_rtc = (NRF_RTC1->COUNTER - r0) & 0x00FFFFFFu;
    uint64_t total_units = (uint64_t)s_acc_units
                         + (uint64_t)d_rtc * 1000u
                         + (uint64_t)frac_cyc * 512u / 1000u;
    uint32_t step = (uint32_t)(total_units / UNITS_PER_MS);
    s_acc_units   = (uint32_t)(total_units % UNITS_PER_MS);
    /* Wake latency past the deadline: vTaskStepTick may only advance up to
     * the expected idle time. With SEVONPEND asserted (above) the RTC1 pend
     * wakes the WFE loop promptly, so any overshoot is interrupt-latency
     * scale (sub-ms) - discard it rather than re-queue it (re-queuing strands
     * time and runs the clock slow whenever wakes are late). */
    if (step > expected_idle_ticks)
        step = expected_idle_ticks;
    if (step > 0) {
        vTaskStepTick(step);
        pwr_note_deep_sleep(step);
    }

    /* Tick back on: full period from now; the sub-ms fraction lives on in
     * s_acc_units and is credited on the next episode. */
    SysTick->VAL  = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    __enable_irq();
}

void cu_power_release_lfclk_for_sd(void)
{
    if (!s_lfclk_kicked) return;
    s_lfclk_kicked = false;
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
    /* The SD restarts the LFXO from its sd_lfclk_cfg before sd_enable
     * returns; RTC1 freezes only across that call, during which no task
     * runs, so the tickless readiness latch can stay set. */
}

/* ---- Init ---- */

void cu_power_init(void)
{
    /* Initial VBUS state, read directly - the SoftDevice isn't up yet. */
    s_vbus = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;

    /* A pending-but-disabled interrupt must generate a WFE wake event: the
     * tickless sleep loop depends on it (SoftDevice-active path). */
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    /* NOTE: the LFXO is deliberately not started here. With BLE on (default)
     * the SoftDevice starts and owns it (sd_lfclk_cfg XTAL); pre-starting it
     * from the app can stall sd_softdevice_enable. The ble=0 case starts it
     * lazily from the tickless path instead. */

    /* RTC1 free-running at 32768 Hz (prescaler 0) as the tickless wake
     * timer (it counts once an LFCLK runs). RTC0 belongs to the SoftDevice;
     * RTC1 is ours. The SD forwards its IRQ through our vector table like
     * SysTick/PendSV. */
    NRF_RTC1->PRESCALER = 0;
    NRF_RTC1->EVTENCLR = 0xFFFFFFFFu;
    NRF_RTC1->INTENCLR = 0xFFFFFFFFu;
    NRF_RTC1->TASKS_CLEAR = 1;
    NRF_RTC1->TASKS_START = 1;
    NVIC_SetPriority(RTC1_IRQn, 7);       /* lowest app priority */
    NVIC_ClearPendingIRQ(RTC1_IRQn);
    NVIC_EnableIRQ(RTC1_IRQn);

    /* GPIO detect hygiene before arming PORT: the DFU bootloader may leave
     * SENSE bits, latched pins, or LDETECT mode behind. A stale latch with
     * LDETECT would re-assert the PORT event forever (an interrupt storm that
     * starves every task); stray SENSE pins would block future wake edges.
     * Reset to direct DETECT, clear every SENSE field, clear latches. */
    NRF_P0->DETECTMODE = 0;
    NRF_P1->DETECTMODE = 0;
    for (int pin = 0; pin < 32; pin++)
        NRF_P0->PIN_CNF[pin] &= ~GPIO_PIN_CNF_SENSE_Msk;
    for (int pin = 0; pin < 16; pin++)
        NRF_P1->PIN_CNF[pin] &= ~GPIO_PIN_CNF_SENSE_Msk;
    NRF_P0->LATCH = 0xFFFFFFFFu;   /* write-1-to-clear */
    NRF_P1->LATCH = 0xFFFFFFFFu;

    /* Button wake: SENSE on both buttons + the single GPIOTE PORT event. */
    button_pin_cfg(BTN_A_PORT, BTN_A_PIN);
    button_pin_cfg(BTN_B_PORT, BTN_B_PIN);
    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_SetPriority(GPIOTE_IRQn, 6);     /* app-allowed, above syscall floor */
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);

    s_last_activity = 0;

    /* Idle policy: LED dim + slow advertising + auto power off. */
    TimerHandle_t t = xTimerCreate("idlepol", pdMS_TO_TICKS(POLICY_PERIOD_MS),
                                   pdTRUE, NULL, idle_policy_cb);
    if (t) xTimerStart(t, 0);
}
