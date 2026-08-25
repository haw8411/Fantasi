/* Fantasi / Proxmark3 USB descriptors.
 *
 * SAM7S UDP has only 4 endpoints, so the classes are mutually exclusive and
 * selected at runtime by re-enumerating (see hal.c). pm3_usb_mode picks the set:
 *   0 = CDC (default)     EP0 ctrl, EP1 IN notify, EP2 OUT data, EP3 IN data
 *   1 = MSC               EP0 ctrl, EP1 OUT bulk, EP2 IN bulk
 *   2 = WebUSB (vendor)   EP0 ctrl, EP1 OUT bulk, EP2 IN bulk
 *
 * WebUSB needs USB 2.1 + a BOS descriptor; we only advertise that (and the
 * Microsoft OS 2.0 / WinUSB binding) while actually in vendor mode, so Windows
 * never tries to bind WinUSB to the CDC interface in CDC mode. */

#include "tusb.h"
#include "usb_proto.h"
#include "usb_mux.h"
#include "../../hal/hal.h"
#include <string.h>

#define USB_VID  0x1209
#define USB_PID  0x0001

#define VENDOR_REQUEST_WEBUSB     1
#define VENDOR_REQUEST_MICROSOFT  2

/* Runtime mode flag - 0 = CDC (default), 1 = MSC, 2 = WebUSB.
 * Set by hal.c mode-switching logic. */
volatile uint8_t pm3_usb_mode;

/* Mutable so bcdUSB can reflect the mode: 2.1 (BOS) only in WebUSB mode. */
static tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void)
{
    desc_device.bcdUSB = (pm3_usb_mode == 2) ? 0x0210 : 0x0200;
    return (uint8_t const *)&desc_device;
}

/* ---- CDC configuration ---- */

#define CDC_CONFIG_TOTAL  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x83

static uint8_t const desc_cdc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, CDC_CONFIG_TOTAL, 0x00, 100),
    TUD_CDC_DESCRIPTOR(0, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

/* ---- MSC configuration ---- */

#if CFG_TUD_MSC
#define MSC_CONFIG_TOTAL  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
#define EPNUM_MSC_OUT     0x01
#define EPNUM_MSC_IN      0x82
static uint8_t const desc_msc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, MSC_CONFIG_TOTAL, 0x00, 100),
    TUD_MSC_DESCRIPTOR(0, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};
#endif

/* ---- Vendor (WebUSB) configuration ---- */

#if CFG_TUD_VENDOR
#define VENDOR_CONFIG_TOTAL  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)
#define EPNUM_VENDOR_OUT  0x01
#define EPNUM_VENDOR_IN   0x82
#define STRID_VENDOR      6
static uint8_t const desc_vendor_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, VENDOR_CONFIG_TOTAL, 0x00, 100),
    TUD_VENDOR_DESCRIPTOR(0, STRID_VENDOR, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),
};
#endif

/* ---- HID configuration (keyboard-only, BadUSB) ---- */

#if CFG_TUD_HID
#define HID_CONFIG_TOTAL  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define EPNUM_HID_IN      0x81
#define STRID_HID         7

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

static uint8_t const desc_hid_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, HID_CONFIG_TOTAL, 0x00, 100),
    TUD_HID_DESCRIPTOR(0, STRID_HID, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(desc_hid_report), EPNUM_HID_IN, CFG_TUD_HID_EP_BUFSIZE, 5),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return desc_hid_report;
}

/* Host keyboard-LED output report, surfaced through hal_hid_host(). */
volatile uint8_t pm3_hid_host_leds;

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id;
    if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize >= 1)
        pm3_hid_host_leds = buffer[0];
}
#endif /* CFG_TUD_HID */

/* ---- Configuration callback ---- */

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
#if CFG_TUD_HID
    if (pm3_usb_mode == 3)
        return desc_hid_configuration;
#endif
#if CFG_TUD_VENDOR
    if (pm3_usb_mode == 2)
        return desc_vendor_configuration;
#endif
#if CFG_TUD_MSC
    if (pm3_usb_mode == 1)
        return desc_msc_configuration;
#endif
    return desc_cdc_configuration;
}

/* ---- WebUSB BOS + MS OS 2.0 (only reached in vendor mode; bcdUSB is 2.1 only
 *      then, so the host requests these solely while the vendor interface - at
 *      interface 0 in this mode - is the one present). ---- */
#if CFG_TUD_VENDOR

#define BOS_TOTAL_LEN  (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define MS_OS_20_DESC_LEN  0xB2

uint8_t const desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

uint8_t const *tud_descriptor_bos_cb(void) { return desc_bos; }

uint8_t const desc_ms_os_20[] = {
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),
    /* Vendor interface is interface 0 in vendor mode. */
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08-0x08-0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050),
    '{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00, '4', 0x00, 'D', 0x00, '9', 0x00, '-', 0x00,
    '0', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00, '-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00, 'D', 0x00, '-', 0x00,
    '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00, 'C', 0x00, 'A', 0x00,
    '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00, '9', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,
};
TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect MS OS 2.0 descriptor size");

#define WEBUSB_LANDING_URL  "fantasi.cloud"
static const tusb_desc_webusb_url_t desc_url = {
    .bLength         = 3 + sizeof(WEBUSB_LANDING_URL) - 1,
    .bDescriptorType = 3,
    .bScheme         = 1,   /* https */
    .url             = WEBUSB_LANDING_URL,
};

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        request->bRequest >= FANTASI_USB_MUX_OPEN &&
        request->bRequest <= FANTASI_USB_MUX_CHUNK)
        return usb_proto_control_xfer(rhport, stage, request);
    if (stage != CONTROL_STAGE_SETUP) return true;
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR) {
        switch (request->bRequest) {
        case VENDOR_REQUEST_WEBUSB:
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)&desc_url, desc_url.bLength);
        case VENDOR_REQUEST_MICROSOFT:
            if (request->wIndex == 7) {
                uint16_t total_len;
                memcpy(&total_len, desc_ms_os_20 + 8, 2);
                return tud_control_xfer(rhport, request, (void *)(uintptr_t)desc_ms_os_20, total_len);
            }
            return false;
        default: break;
        }
    }
    return false;
}

#endif /* CFG_TUD_VENDOR */

/* ---- String descriptors ---- */

static char const *string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },
    "Fantasi",
    "Fantasi CLI",
    "000001",
    "Fantasi CLI CDC",
    "Fantasi Storage",
    "Fantasi Data",
    "Fantasi Keyboard",   /* 7: HID keyboard (switch-mode) */
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;
    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;
        const char *s = (index == 3) ? hal_device_name() : string_desc_arr[index];
        chr_count = strlen(s);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = s[i];
    }
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}
