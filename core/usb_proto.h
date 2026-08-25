#ifndef FANTASI_USB_PROTO_H
#define FANTASI_USB_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#include "tusb.h"

/* Task that runs the protobuf engine over the USB vendor interface (WebUSB).
 * On composite targets (FZ/CU) it's always active; on switch-mode targets (PM3)
 * the vendor interface is only mounted while in WebUSB mode, so it idles until
 * then. */
void usb_proto_task(void *arg);

/* Stop every device-owned WebUSB session when a switch-mode target tears down
 * the physical vendor personality. Cancels workers and releases mailboxes; the
 * serial personality remains unaffected. */
void usb_proto_transport_down(void);

/* Delegated by the descriptor-level tud_vendor_control_xfer_cb after the
 * standard WebUSB/MS OS requests have been checked. */
bool usb_proto_control_xfer(uint8_t rhport, uint8_t stage,
                            tusb_control_request_t const *request);

#endif
