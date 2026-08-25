/* Host-side transport for the device's USB vendor (WebUSB) interface, over
 * libusb. Carries the same framed protobuf as the BLE transport, so the file/CLI
 * commands work over it unchanged - used for the Proxmark3 (switch-mode) so file
 * data flows over a dedicated pipe instead of churning the MSC mount. */
#ifndef FANTASI_USB_TRANSPORT_H
#define FANTASI_USB_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/* Find the Fantasi device, claim its vendor (class 0xFF) interface, and resolve
 * its bulk endpoints. Returns 0 on success, -1 if not found / not in vendor mode. */
int  usb_transport_open(void);
void usb_transport_close(void);
bool usb_transport_connected(void);
/* Non-zero for the independent control-endpoint transport; zero means the
 * legacy claimed bulk interface fallback. */
uint32_t usb_transport_session_id(void);
bool usb_transport_multiplexed(void);

/* True for the SAM7S PM3's 8-byte EP0: a device that must pace uploads to one
 * in-flight chunk (its dual-bank OUT overruns under pipelining). */
bool usb_transport_constrained_ep0(void);

/* Restrict usb_transport_open() to the device with this name, for
 * disambiguating multiple connected Fantasi devices. The device's name
 * (hal_device_name()) is carried in the USB iSerialNumber string
 * descriptor, so this matches against iSerial. NULL/empty = any. */
void usb_transport_set_name(const char *name);

/* Active liveness probe (a GET_STATUS on EP0). Returns false and marks the
 * transport disconnected once the device is unplugged. Use to detect an idle
 * disconnect, where there's no bulk traffic to surface the removal. */
bool usb_transport_alive(void);

/* Bulk transfers. read returns 0 on timeout (not an error); both return -1 on a
 * hard error (e.g. device unplugged). */
ssize_t usb_transport_read(void *buf, size_t len);
ssize_t usb_transport_write(const void *buf, size_t len);
/* Acknowledge a fully-decoded mux response frame: releases the device
 * mailbox so the next streamed frame can be emitted, and resets the read
 * offset. No-op on the bulk path. */
void usb_transport_frame_consumed(void);
/* Rewind the per-frame read offset (no I/O) before issuing a new request. */
void usb_transport_read_reset(void);

#endif
