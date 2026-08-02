#include "usb_transport.h"

#include <libusb-1.0/libusb.h>
#include <string.h>

#define FANTASI_VID 0x1209
#define FANTASI_PID 0x0001

static libusb_context       *s_ctx;
static libusb_device_handle *s_dev;
static int      s_itf   = -1;
static uint8_t  s_ep_in;
static uint8_t  s_ep_out;
static char     s_want_name[64]; /* restrict to this device name (iSerial), for multi-device setups (empty = any) */

void usb_transport_set_name(const char *name)
{
    if (name) { strncpy(s_want_name, name, sizeof s_want_name - 1); s_want_name[sizeof s_want_name - 1] = 0; }
    else s_want_name[0] = 0;
}

/* Locate the vendor (class 0xFF) interface on the open device and record its
 * bulk endpoints. Returns 0 on success. */
static int find_vendor_interface(libusb_device *dev)
{
    struct libusb_config_descriptor *cfg;
    if (libusb_get_active_config_descriptor(dev, &cfg) != 0) return -1;

    int rc = -1;
    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        const struct libusb_interface_descriptor *id = &cfg->interface[i].altsetting[0];
        if (id->bInterfaceClass != 0xFF) continue;   /* vendor-specific */

        uint8_t ep_in = 0, ep_out = 0;
        for (int e = 0; e < id->bNumEndpoints; e++) {
            uint8_t addr = id->endpoint[e].bEndpointAddress;
            if ((id->endpoint[e].bmAttributes & 0x03) != LIBUSB_TRANSFER_TYPE_BULK)
                continue;
            if (addr & LIBUSB_ENDPOINT_IN) ep_in = addr; else ep_out = addr;
        }
        if (ep_in && ep_out) {
            s_itf = id->bInterfaceNumber;
            s_ep_in = ep_in;
            s_ep_out = ep_out;
            rc = 0;
            break;
        }
    }
    libusb_free_config_descriptor(cfg);
    return rc;
}

int usb_transport_open(void)
{
    if (s_dev) return 0;
    if (!s_ctx && libusb_init(&s_ctx) != 0) return -1;

    libusb_device **list;
    ssize_t n = libusb_get_device_list(s_ctx, &list);
    if (n < 0) return -1;

    int rc = -1;
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
        if (dd.idVendor != FANTASI_VID || dd.idProduct != FANTASI_PID) continue;
        if (find_vendor_interface(list[i]) != 0) continue;   /* not in vendor mode */

        if (libusb_open(list[i], &s_dev) != 0) { s_dev = NULL; continue; }
        if (s_want_name[0]) {                        /* filter by device name (iSerial) for multi-device setups */
            unsigned char name[64];
            int sr = dd.iSerialNumber
                   ? libusb_get_string_descriptor_ascii(s_dev, dd.iSerialNumber, name, sizeof name) : -1;
            if (sr < 0 || strcmp((char *)name, s_want_name) != 0) {
                libusb_close(s_dev); s_dev = NULL; continue;
            }
        }
        libusb_set_auto_detach_kernel_driver(s_dev, 1);
        /* A vendor device with no kernel driver may be left unconfigured by the
         * OS; set config 1 explicitly before claiming (else claim fails and the
         * device is stranded at config 0). */
        int cfg_num = 0;
        libusb_get_configuration(s_dev, &cfg_num);
        if (cfg_num != 1) libusb_set_configuration(s_dev, 1);
        if (libusb_claim_interface(s_dev, s_itf) != 0) {
            libusb_close(s_dev); s_dev = NULL; continue;
        }
        rc = 0;
        break;
    }
    libusb_free_device_list(list, 1);

    /* Drain any stale IN data the device left in the vendor FIFO from a previous
     * session - e.g. un-read app-output frames after a ^C-aborted launch. Without
     * this, the first command's response would read that leftover instead,
     * desyncing the framed stream (a fresh connection has nothing legitimately
     * pending, so discarding here is safe). */
    if (rc == 0) {
        unsigned char junk[256];
        int got;
        for (int i = 0; i < 64; i++) {
            if (libusb_bulk_transfer(s_dev, s_ep_in, junk, sizeof(junk), &got, 30) != 0
                || got == 0)
                break;
        }
    }
    return rc;
}

void usb_transport_close(void)
{
    if (s_dev) {
        libusb_release_interface(s_dev, s_itf);
        libusb_close(s_dev);
        s_dev = NULL;
    }
    if (s_ctx) { libusb_exit(s_ctx); s_ctx = NULL; }
    s_itf = -1;
}

bool usb_transport_connected(void) { return s_dev != NULL; }

/* Device is gone: close the handle so usb_transport_connected() reads false
 * and a later usb_transport_open() rescans. Keep s_ctx so reopen is cheap. */
static void usb_transport_drop(void)
{
    if (s_dev) { libusb_close(s_dev); s_dev = NULL; }
    s_itf = -1;
}

bool usb_transport_alive(void)
{
    if (!s_dev) return false;
    /* Definitive presence check: is our device still enumerated? An abrupt
     * physical unplug can make a control-transfer probe return an ambiguous
     * error (IO vs NO_DEVICE), so we'd wrongly report "alive" and never start
     * reconnecting. Comparing our device against a fresh device list is
     * unambiguous - a removed device is simply no longer in it. */
    libusb_device *mine = libusb_get_device(s_dev);
    libusb_device **list;
    ssize_t n = libusb_get_device_list(s_ctx, &list);
    if (n < 0) return true;   /* can't enumerate right now; assume still present */

    bool present = false;
    for (ssize_t i = 0; i < n; i++) {
        if (list[i] == mine) { present = true; break; }
    }
    libusb_free_device_list(list, 1);

    if (!present) { usb_transport_drop(); return false; }
    return true;
}

ssize_t usb_transport_read(void *buf, size_t len)
{
    if (!s_dev) return -1;
    int got = 0;
    int r = libusb_bulk_transfer(s_dev, s_ep_in, buf, (int)len, &got, 50);
    if (r == 0 || r == LIBUSB_ERROR_TIMEOUT) return got;
    if (r == LIBUSB_ERROR_NO_DEVICE) { usb_transport_drop(); return -1; }
    return got;   /* other transient errors: report what we got (0) */
}

ssize_t usb_transport_write(const void *buf, size_t len)
{
    if (!s_dev) return -1;
    int sent = 0;
    int r = libusb_bulk_transfer(s_dev, s_ep_out, (unsigned char *)buf, (int)len, &sent, 1000);
    if (r == LIBUSB_ERROR_NO_DEVICE) { usb_transport_drop(); return -1; }
    if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) return -1;
    return sent;
}
