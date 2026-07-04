/* gui - on-device menu (Flipper Zero).
 *
 * A dedicated task drives a small menu UI on the LCD: the splash screen shows
 * a "[^ Menu]" hint, UP opens the Main Menu, and from there Apps (list and
 * launch anything in /ramfs and /apps) and Settings (toggle key=value
 * settings). Button presses arrive as events from the EXTI ISRs in hal.c.
 *
 * Built only with FANTASI_ENABLE_GUI (implies FANTASI_ENABLE_APPS). */
#ifndef FANTASI_GUI_H
#define FANTASI_GUI_H

#include <stdint.h>
#include "FreeRTOS.h"

/* Create the button-event queue and the gui task. Called from hal_init()
 * before the scheduler starts (the queue must exist before buttons_init
 * unmasks the EXTI lines). */
void gui_init(void);

/* ISR-side entry: report debounced button presses. `mask` is FANTASI_BTN_*
 * bits (app_api.h); presses too soon after the previous one on the same
 * button are dropped here. */
void gui_buttons_from_isr(uint32_t mask, BaseType_t *woken);

#endif /* FANTASI_GUI_H */
