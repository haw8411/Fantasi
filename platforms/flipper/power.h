#ifndef FANTASI_FLIPPER_POWER_H
#define FANTASI_FLIPPER_POWER_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

/* One-time power bring-up: reset forensics, idle-policy timer. Called from
 * hal_init() (before the scheduler). */
void fz_power_init(void);

/* Log the previous boot's exit (reset flags + crash fingerprint). Called from
 * hal_post_init once the logger is up. */
void fz_power_boot_log(void);

/* Wake every task blocked in hal_button_wait + note user activity. Called
 * from the button EXTI ISRs (button_irq_common). */
void fz_power_buttons_wake(BaseType_t *woken);

/* True when the idle policy wants slow advertising (ble.c consults it when
 * (re)starting background advertising). */
bool fz_power_adv_slow(void);

/* VBUS presence from the BQ25896 charger (I2C; the WB55 USB core cannot
 * sense VBUS). Implemented in hal.c; polled by the idle-policy timer. */
bool fz_hal_vbus_present(void);

/* Re-enable HSI48 (the USB 48 MHz clock) after Stop 2 stopped it. Defined in
 * system.c. */
void fz_usb_clock_restore(void);

#endif
