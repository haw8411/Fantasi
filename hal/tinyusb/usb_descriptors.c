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

/* Interface order is fixed as CDC(0,1), vendor(2), then the optional MSC and
 * HID. Keeping vendor before the optionals pins it at interface 2 regardless of
 * whether MSC/HID are enumerated, so the MS OS 2.0 / WinUSB binding (which names
 * the vendor interface number below) never has to change. */
#define ITF_NUM_CDC       0
#define ITF_NUM_VENDOR    2

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82
#define EPNUM_MSC_OUT     0x03
#define EPNUM_MSC_IN      0x83
#define EPNUM_VENDOR_OUT  0x04
#define EPNUM_VENDOR_IN   0x84
#define EPNUM_HID_IN      0x85

/* String descriptor indices (kept stable regardless of which classes are on). */
#define STRID_CDC     4
#define STRID_MSC     5
#define STRID_VENDOR  6
#define STRID_HID     7

/* ---- HID keyboard report descriptor (boot-protocol keyboard) ---- */
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return desc_hid_report;
}

/* Runtime interface toggles, shared with the platform HAL via the accessors.
 *   MSC     - the mass-storage drive; `msc` setting, default on.
 *   HID     - the keyboard; persistent (always present) or switch (only while an
 *             app armed it, at the cost of a re-enumeration). `hid` setting.
 * Changing any of these re-enumerates so the host re-reads the interface set.
 * volatile: written from an app/gui task, read from the USB task/enumeration. */
static volatile bool    s_msc_enabled = true;
static volatile bool    s_hid_persistent = true;
static volatile bool    s_hid_active;
static volatile uint8_t s_hid_host_leds;

void    usb_desc_set_msc_enabled(bool on)    { s_msc_enabled = on; }
bool    usb_desc_msc_enabled(void)           { return s_msc_enabled; }
void    usb_desc_set_hid_persistent(bool on) { s_hid_persistent = on; }
bool    usb_desc_hid_persistent(void)        { return s_hid_persistent; }
void    usb_desc_set_hid_active(bool on)     { s_hid_active = on; }
bool    usb_desc_hid_active(void)            { return s_hid_active; }
uint8_t usb_desc_hid_host_leds(void)         { return s_hid_host_leds; }

/* Largest the configuration can get (every optional interface present). */
#define CONFIG_MAX_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN \
                         + TUD_VENDOR_DESC_LEN + TUD_MSC_DESC_LEN + TUD_HID_DESC_LEN)

/* The configuration descriptor is assembled per enumeration from whichever
 * interfaces are currently enabled, with sequential interface numbers. The
 * inputs (the flags above) don't change during a single enumeration, so the
 * bytes are stable across the host's repeated GET_DESCRIPTOR calls. */
static uint8_t s_cfg_desc[CONFIG_MAX_LEN];

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    bool msc = false, hid = false;
#if CFG_TUD_MSC
    msc = s_msc_enabled;
#endif
#if CFG_TUD_HID
    hid = s_hid_persistent || s_hid_active;
#endif

    uint8_t *p = s_cfg_desc + TUD_CONFIG_DESC_LEN;   /* header written last */
    uint8_t itf = 0;

    { uint8_t d[] = { TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8,
                                         EPNUM_CDC_OUT, EPNUM_CDC_IN, 64) };
      memcpy(p, d, sizeof d); p += sizeof d; itf += 2; }
#if CFG_TUD_VENDOR
    { uint8_t d[] = { TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR,
                                            EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64) };
      memcpy(p, d, sizeof d); p += sizeof d; itf += 1; }
#endif
#if CFG_TUD_MSC
    if (msc) { uint8_t d[] = { TUD_MSC_DESCRIPTOR(itf, STRID_MSC, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64) };
      memcpy(p, d, sizeof d); p += sizeof d; itf += 1; }
#endif
#if CFG_TUD_HID
    if (hid) { uint8_t d[] = { TUD_HID_DESCRIPTOR(itf, STRID_HID, HID_ITF_PROTOCOL_KEYBOARD,
                                                  sizeof(desc_hid_report), EPNUM_HID_IN,
                                                  CFG_TUD_HID_EP_BUFSIZE, 5) };
      memcpy(p, d, sizeof d); p += sizeof d; itf += 1; }
#endif

    uint16_t total = (uint16_t)(p - s_cfg_desc);
    uint8_t hdr[] = { TUD_CONFIG_DESCRIPTOR(1, itf, 0, total, 0x00, 100) };
    memcpy(s_cfg_desc, hdr, sizeof hdr);
    return s_cfg_desc;
}

/* ---- HID class callbacks ---- */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;   /* no input reports fetched over the control pipe */
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id;
    /* Host's keyboard-LED output report - the cheap host-present / caps-lock
     * signal BadUSB surfaces through api->hid_host(). */
    if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize >= 1)
        s_hid_host_leds = buffer[0];
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
    "Fantasi Keyboard",              /* 7: HID keyboard interface */
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
