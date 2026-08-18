#ifndef FANTASI_HAL_POWER_H
#define FANTASI_HAL_POWER_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Sleep governance (core/power.c) ----
 *
 * Subsystems that need the CPU (or a clock a sleep state would stop) hold a
 * counted inhibitor vote; the platform's idle/tickless path asks
 * pwr_allowed_depth() before sleeping. Modeled on stock Flipper's insomnia
 * counter merged with CubeWB's UTIL_LPM client bitmask. */

/* Sleep depth the platform may enter from the idle path right now. */
typedef enum {
    HAL_SLEEP_NONE,   /* busy: don't even WFI (sleep disabled, flash op...)    */
    HAL_SLEEP_LIGHT,  /* WFI/WFE-class: CPU clock stops, everything else runs  */
    HAL_SLEEP_DEEP,   /* Stop2 / System ON uA idle / SLCK mode                 */
} hal_sleep_depth_t;

/* Durable inhibitor clients. Each holds counted votes; any held vote caps the
 * allowed depth at HAL_SLEEP_LIGHT. (PWR_CLIENT_APP is implicit - a running
 * app is detected via app_running_pid(), no votes needed.) */
typedef enum {
    PWR_CLIENT_USB_ACTIVE,   /* USB configured and not bus-suspended          */
    PWR_CLIENT_BLE_LINK,     /* BLE connection established                    */
    PWR_CLIENT_FLASH,        /* flash erase/write in flight                   */
    PWR_CLIENT_SD_OP,        /* radio-coprocessor command awaiting response   */
    PWR_CLIENT_CLI_SESSION,  /* recent interactive CLI activity               */
    PWR_CLIENT_COUNT
} pwr_client_t;

/* Wake-source accounting for the `power` command. */
typedef enum {
    PWR_WAKE_TIMER,    /* tickless wake timer expired                */
    PWR_WAKE_USB,      /* USB IRQ / VBUS event                       */
    PWR_WAKE_RADIO,    /* BLE / SoftDevice / radio-coprocessor event */
    PWR_WAKE_BUTTON,   /* button edge                                */
    PWR_WAKE_OTHER,
    PWR_WAKE_COUNT
} pwr_wake_t;

typedef struct {
    uint32_t light_entries;               /* idle-hook / WFI-class sleeps     */
    uint32_t deep_entries;                /* tickless suppressed-tick sleeps  */
    uint32_t slept_ticks;                 /* total ticks credited by tickless */
    uint32_t wake[PWR_WAKE_COUNT];
} pwr_stats_t;

/* Load persisted settings (sleep enable, off-timeout). Call once at boot,
 * after storage is up but before the scheduler starts. */
void pwr_init(void);

void pwr_inhibit_enter(pwr_client_t c);
void pwr_inhibit_exit(pwr_client_t c);
hal_sleep_depth_t pwr_allowed_depth(void);

/* Like pwr_allowed_depth() but ignores one client's vote. A platform whose radio
 * can wake the core from deep sleep (WB55: CPU1 wakes on IPCC) passes
 * PWR_CLIENT_BLE_LINK so a maintained-but-idle BLE link doesn't force light
 * sleep. Pass PWR_CLIENT_COUNT to ignore nothing (== pwr_allowed_depth). */
hal_sleep_depth_t pwr_allowed_depth_except(pwr_client_t ignore);

int         pwr_client_votes(pwr_client_t c);
const char *pwr_client_name(pwr_client_t c);

/* Global runtime enable ("power sleep on|off", persisted as sleep=0|1). */
bool pwr_sleep_enabled(void);
int  pwr_set_sleep_enabled(bool on);          /* persists; 0 on success */

/* Auto-power-off timeout in seconds, 0 = never ("power off-timeout <s>",
 * persisted as offtimeout=<s>). Platforms with a power-off state poll this. */
uint32_t pwr_off_timeout_s(void);
int      pwr_set_off_timeout_s(uint32_t s);   /* persists; 0 on success */

/* Sleep accounting - called by the platform sleep code (IRQs may be masked). */
void pwr_note_light_sleep(void);
void pwr_note_deep_sleep(uint32_t slept_ticks);
void pwr_note_slept(uint32_t slept_ticks);   /* credit asleep time, no entry counter */
void pwr_note_wake(pwr_wake_t reason);
void pwr_get_stats(pwr_stats_t *out);

/* ---- Platform hooks ---- */

/* Block until button activity is possible (edge IRQ) or timeout_ms elapsed.
 * Weak default polls: delays min(timeout_ms, 50 ms) and returns, so callers
 * loop exactly as before on platforms without a button wake IRQ. */
void hal_button_wait(uint32_t timeout_ms);

/* User/host interaction hint - called on CLI command dispatch and proto
 * frames. Platforms with an idle policy (LED dim, slow advertising, auto
 * power off) reset their inactivity clock here; weak default is a no-op. */
void hal_power_activity(void);

/* Record a crash fingerprint in a reset-surviving register just before a
 * self-reset, so the next boot can log what killed the previous one. Weak
 * default is a no-op. Codes: 0xA1 stack overflow, 0xA2 radio-stack fault. */
#define HAL_CRASH_STACK_OVERFLOW 0xA1
#define HAL_CRASH_RADIO_FAULT    0xA2
void hal_crash_note(uint8_t code);

#endif
