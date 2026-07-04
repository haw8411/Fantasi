/* Fantasi / Proxmark3 - shim for usart_defs.h. Only USART_BAUD_RATE is
 * read by the ported usb_cdc.c (it gets sent back to the host in the
 * CDC LINE_CODING descriptor for anyone who cares). */
#ifndef FANTASI_PM3_USART_DEFS_H
#define FANTASI_PM3_USART_DEFS_H

#define USART_BAUD_RATE  115200

#endif
