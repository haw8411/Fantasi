#ifndef FANTASI_USB_PROTO_H
#define FANTASI_USB_PROTO_H

#include <stdbool.h>

/* Task that runs the protobuf engine over the USB vendor interface (WebUSB).
 * On composite targets (FZ/CU) it's always active; on switch-mode targets (PM3)
 * the vendor interface is only mounted while in WebUSB mode, so it idles until
 * then. */
void usb_proto_task(void *arg);

#endif
