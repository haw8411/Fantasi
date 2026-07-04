#include "tusb.h"
#include "../hal.h"

/* USB IDs - these are test/dev values. Using pid.codes 0x1209/0x0001
 * "Experimental" PID which is explicitly allocated for hobby prototypes.
 * See https://pid.codes/1209/0001/ . Swap for a real PID before distributing. */
#define USB_VID  0x1209
#define USB_PID  0x0001

/* Vendor-specific control request codes referenced by the BOS descriptor.
 * WEBUSB → GET_URL (landing page); MICROSOFT → MS OS 2.0 descriptor set. */
#define VENDOR_REQUEST_WEBUSB     1
#define VENDOR_REQUEST_MICROSOFT  2

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210,   /* 2.1 → host requests the BOS descriptor (WebUSB) */
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

uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&desc_device; }

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
#if CFG_TUD_MSC
    ITF_NUM_MSC,
#endif
#if CFG_TUD_VENDOR
    ITF_NUM_VENDOR,
#endif
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82
#define EPNUM_MSC_OUT     0x03
#define EPNUM_MSC_IN      0x83
#define EPNUM_VENDOR_OUT  0x04
#define EPNUM_VENDOR_IN   0x84

/* String descriptor indices (kept stable regardless of which classes are on). */
#define STRID_CDC     4
#define STRID_MSC     5
#define STRID_VENDOR  6

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN \
                           + (CFG_TUD_MSC    ? TUD_MSC_DESC_LEN    : 0) \
                           + (CFG_TUD_VENDOR ? TUD_VENDOR_DESC_LEN : 0))

uint8_t const desc_fs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
#if CFG_TUD_MSC
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, STRID_MSC, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
#endif
#if CFG_TUD_VENDOR
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),
#endif
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_fs_configuration;
}

/* ---- WebUSB BOS descriptor + MS OS 2.0 (Windows WinUSB auto-bind) ---- */
#if CFG_TUD_VENDOR

#define BOS_TOTAL_LEN  (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define MS_OS_20_DESC_LEN  0xB2

uint8_t const desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),      /* vendor code, iLandingPage=1 */
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

uint8_t const *tud_descriptor_bos_cb(void) { return desc_bos; }

uint8_t const desc_ms_os_20[] = {
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),
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

/* Landing page Chrome offers when the device is plugged in. HTTPS (bScheme=1).
 * Placeholder - point at the hosted WebUSB file-manager page. */
#define WEBUSB_LANDING_URL  "fantasi.cloud"
static const tusb_desc_webusb_url_t desc_url = {
    .bLength         = 3 + sizeof(WEBUSB_LANDING_URL) - 1,
    .bDescriptorType = 3,   /* WEBUSB URL */
    .bScheme         = 1,   /* https */
    .url             = WEBUSB_LANDING_URL,
};

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;

    switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
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
        break;
    default: break;
    }
    return false;   /* stall unknown */
}

#endif /* CFG_TUD_VENDOR */

/* String descriptors. Index 0 is the language list; the rest are UTF-16
 * encoded on the fly in the callback. */
static char const *string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   /* 0: English (0x0409) */
    "Fantasi",                       /* 1: Manufacturer    */
    "Fantasi CLI",                   /* 2: Product         */
    "000001",                        /* 3: Serial (placeholder) */
    "Fantasi CLI CDC",               /* 4: CDC interface   */
    "Fantasi Storage",               /* 5: MSC interface   */
    "Fantasi Data",                  /* 6: Vendor (protobuf) interface */
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
