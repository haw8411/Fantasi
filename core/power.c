/* Sleep governance: counted inhibitor votes + sleep statistics.
 *
 * Pure bookkeeping - the actual sleep instructions live in each platform's
 * power code (e.g. platforms/chameleon/power.c), which consults
 * pwr_allowed_depth() from its idle/tickless path.
 *
 * Compiles on the host with -DPWR_HOST_TEST (no FreeRTOS/HAL) so the vote
 * state machine is unit-testable (tests/unit/test_power_votes.py). */

#include "../hal/hal_power.h"

#ifdef PWR_HOST_TEST
static bool app_is_running(void) { return false; }
#else
#include "../hal/hal.h"
#include "app_run.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#endif

static volatile uint16_t s_votes[PWR_CLIENT_COUNT];
static pwr_stats_t s_stats;
static bool     s_sleep_enabled = true;
static uint32_t s_off_timeout_s;          /* 0 = never */

static const char *const s_client_name[PWR_CLIENT_COUNT] = {
    "usb", "ble-link", "flash", "sd-op", "cli",
};

/* Votes use GCC atomics, not critical sections: on the STM32WB the BLE
 * connection lifecycle runs inside the IPCC RX ISR, and taskENTER_CRITICAL
 * from an ISR trips the port's not-in-interrupt configASSERT. Atomics are
 * safe from any context on Cortex-M. */
void pwr_inhibit_enter(pwr_client_t c)
{
    if (c >= PWR_CLIENT_COUNT) return;
    __atomic_fetch_add(&s_votes[c], 1, __ATOMIC_SEQ_CST);
}

void pwr_inhibit_exit(pwr_client_t c)
{
    if (c >= PWR_CLIENT_COUNT) return;
    /* Decrement only if positive (never negative): CAS loop. */
    uint16_t cur = __atomic_load_n(&s_votes[c], __ATOMIC_SEQ_CST);
    while (cur > 0 &&
           !__atomic_compare_exchange_n(&s_votes[c], &cur, cur - 1, false,
                                        __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
        ;
}

int pwr_client_votes(pwr_client_t c)
{
    return (c < PWR_CLIENT_COUNT) ? s_votes[c] : 0;
}

const char *pwr_client_name(pwr_client_t c)
{
    return (c < PWR_CLIENT_COUNT) ? s_client_name[c] : "?";
}

hal_sleep_depth_t pwr_allowed_depth_except(pwr_client_t ignore)
{
    if (!s_sleep_enabled) return HAL_SLEEP_NONE;
    for (int c = 0; c < PWR_CLIENT_COUNT; c++)
        if (c != (int)ignore && s_votes[c] > 0) return HAL_SLEEP_LIGHT;
    if (app_is_running()) return HAL_SLEEP_LIGHT;
    return HAL_SLEEP_DEEP;
}

hal_sleep_depth_t pwr_allowed_depth(void)
{
    return pwr_allowed_depth_except(PWR_CLIENT_COUNT);   /* COUNT = ignore nothing */
}

bool pwr_sleep_enabled(void)
{
    return s_sleep_enabled;
}

uint32_t pwr_off_timeout_s(void)
{
    return s_off_timeout_s;
}

/* ---- Statistics (callers may have IRQs masked - plain writes only) ---- */

void pwr_note_light_sleep(void)               { s_stats.light_entries++; }
void pwr_note_deep_sleep(uint32_t ticks)      { s_stats.deep_entries++;
                                                s_stats.slept_ticks += ticks; }
/* Credit measured asleep time without a depth-specific entry counter - used by
 * light-only platforms (PM5) that measure real WFI duration so `power` reports
 * a true "time asleep" instead of 0 (the kernel owns light-sleep tick math, so
 * pwr_note_light_sleep only counts episodes). */
void pwr_note_slept(uint32_t ticks)           { s_stats.slept_ticks += ticks; }
void pwr_note_wake(pwr_wake_t r)              { if (r < PWR_WAKE_COUNT)
                                                    s_stats.wake[r]++; }

void pwr_get_stats(pwr_stats_t *out)
{
    *out = s_stats;   /* advisory counters; torn reads are acceptable */
}

/* ---- Persistence ---- */

#ifdef PWR_HOST_TEST
void pwr_init(void) {}
int  pwr_set_sleep_enabled(bool on) { s_sleep_enabled = on; return 0; }
int  pwr_set_off_timeout_s(uint32_t s) { s_off_timeout_s = s; return 0; }
#else

/* Auto-power-off default: 5 minutes on battery-powered idle (matches stock
 * ChameleonUltra's inactivity shutdown). USB/BLE/app activity resets it;
 * platforms without a power-off state simply never read it. */
#define PWR_OFF_TIMEOUT_DEFAULT_S 300

void pwr_init(void)
{
    char v[12] = "1";
    hal_settings_get("sleep", v, sizeof(v));
    s_sleep_enabled = (v[0] != '0');

    char t[12] = "";
    if (hal_settings_get("offtimeout", t, sizeof(t)) > 0)
        s_off_timeout_s = (uint32_t)strtoul(t, NULL, 10);
    else
        s_off_timeout_s = PWR_OFF_TIMEOUT_DEFAULT_S;
}

int pwr_set_sleep_enabled(bool on)
{
    s_sleep_enabled = on;
    return hal_settings_set("sleep", on ? "1" : "0");
}

int pwr_set_off_timeout_s(uint32_t s)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s);
    s_off_timeout_s = s;
    return hal_settings_set("offtimeout", buf);
}

/* ---- Weak platform hooks ---- */

/* Poll fallback for platforms without a button wake IRQ: bounded delay so the
 * caller's poll loop keeps its historical cadence. */
__attribute__((weak)) void hal_button_wait(uint32_t timeout_ms)
{
    uint32_t ms = timeout_ms > 50 ? 50 : timeout_ms;
    if (ms) vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Activity hint no-op for platforms without an idle policy. */
__attribute__((weak)) void hal_power_activity(void) { }

/* Crash-fingerprint no-op for platforms without a retained register. */
__attribute__((weak)) void hal_crash_note(uint8_t code) { (void)code; }

#endif /* !PWR_HOST_TEST */
