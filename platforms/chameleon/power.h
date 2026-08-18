#ifndef FANTASI_CHAMELEON_POWER_H
#define FANTASI_CHAMELEON_POWER_H

#include <stdint.h>
#include <stdbool.h>

/* One-time power/sleep bring-up: LFXO start, RTC1 (tickless wake timer),
 * GPIOTE PORT button wake, SEVONPEND, idle-policy timer. Called from
 * hal_init(), before the SoftDevice comes up. */
void cu_power_init(void);

/* VBUS presence tracking (POWER ISR pre-SD, SoC events post-SD). ISR-safe. */
void cu_power_vbus(bool present);

/* True when the idle policy wants the launcher LEDs dark. The launcher task
 * owns the LEDs and acts on this from its own loop. */
bool cu_power_led_idle(void);

/* Note user/host activity: resets the idle timer (and thus LED dim, slow
 * advertising, and the auto-off countdown). ISR-safe. */
void cu_power_note_activity(void);

/* FreeRTOS tickless hook (portSUPPRESS_TICKS_AND_SLEEP). */
void cu_suppress_ticks_and_sleep(uint32_t expected_idle_ticks);

/* Called by cu_ble_sd_init immediately before sd_softdevice_enable: if the
 * app started the LFXO by hand (the ble=0 boot case), stop it so the
 * SoftDevice can take clean ownership - enabling the SD over an app-started
 * LFCLK can stall sd_softdevice_enable. */
void cu_power_release_lfclk_for_sd(void);

#endif
