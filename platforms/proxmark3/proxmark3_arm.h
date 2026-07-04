/* Fantasi / Proxmark3 - shim for proxmark3_arm.h. The upstream header
 * pulls in the entire PM3 firmware surface (BigBuf, LEDs, FPGA, ...),
 * none of which we need. usb_cdc.c only references the AT91 register
 * set and a baud-rate constant; we satisfy both lightly here. */
#ifndef FANTASI_PM3_PROXMARK3_ARM_H
#define FANTASI_PM3_PROXMARK3_ARM_H

#include "at91sam7s512.h"
#include "common.h"

/* Values lifted verbatim from the upstream proxmark3_arm.h so the
 * ported usb_cdc.c and the AT91 register ops retain their original
 * semantics. MCK = master clock in Hz; MAINCK is the crystal. */
#define MAINCK                                  16000000
#define MCK                                     (3 * MAINCK)

#define WDT_HIT()                               AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001
#define PWM_CH_MODE_PRESCALER(x)                ((x) << 0)
#define PWM_CHANNEL(x)                          (1 << (x))

/* USB D+ pull-up control pin on the Proxmark3 schematic. */
#define GPIO_USB_PU                             AT91C_PIO_PA24

/* PM3 return-code constants that usb_cdc.c returns from the *_write
 * functions. We preserve the upstream values so callers that compare
 * against PM3_SUCCESS keep working, but nothing in our code actually
 * inspects them - hal_serial_write() only needs the byte count. */
#define PM3_SUCCESS                             0
#define PM3_EINVARG                            (-1)
#define PM3_EIO                                (-2)

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#  define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#endif
