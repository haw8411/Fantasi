#ifndef FANTASI_TUSB_CONFIG_H
#define FANTASI_TUSB_CONFIG_H

/* CFG_TUSB_MCU is expected to come from the platform's Makefile
 * (e.g. -DCFG_TUSB_MCU=OPT_MCU_STM32WB) so this file stays platform-agnostic. */
#ifndef CFG_TUSB_MCU
#  error "CFG_TUSB_MCU must be defined by the platform build"
#endif

#define CFG_TUSB_OS             OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUSB_DEBUG          0

/* Endpoint 0 size - 64 works on all USB-FS peripherals we target. */
#define CFG_TUD_ENDPOINT0_SIZE  64

/* CDC (the CLI) is always enumerated. HID is the BadUSB keyboard: compiled in and
 * enumerated by default (persistent mode), staying inert until an app drives it.
 * The opt-in `hid=switch` setting instead adds/removes the HID interface on the
 * fly, so it only appears while a payload has it armed (the config descriptor is
 * assembled per-enumeration in usb_descriptors.c). */
#define CFG_TUD_CDC             1
#define CFG_TUD_HID             1
#define CFG_TUD_HID_EP_BUFSIZE  16
#define CFG_TUD_MIDI            0

/* Vendor interface carries the protobuf transport (WebUSB). Always present on
 * the composite targets (FZ/CU) alongside CDC + MSC. */
#define CFG_TUD_VENDOR          1
#define CFG_TUD_VENDOR_RX_BUFSIZE 1024
#define CFG_TUD_VENDOR_TX_BUFSIZE 1024
#define CFG_TUD_VENDOR_EPSIZE     64

#ifdef FANTASI_ENABLE_MSC
#define CFG_TUD_MSC             1
#define CFG_TUD_MSC_EP_BUFSIZE  512
#else
#define CFG_TUD_MSC             0
#endif

#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  256
#define CFG_TUD_CDC_EP_BUFSIZE  64

#endif
