/* Fantasi / Proxmark3 TinyUSB configuration.
 *
 * A shadow of hal/tinyusb/tusb_config.h with the same class layout
 * (CDC only, 256-byte TX/RX fifos). Separate file so we can tune
 * stack sizes and keep SAM7S-specific assumptions here; build picks
 * this up by putting platforms/proxmark3 ahead of the common tree
 * on the include path. */
#ifndef FANTASI_PM3_TUSB_CONFIG_H
#define FANTASI_PM3_TUSB_CONFIG_H

#ifndef CFG_TUSB_MCU
#  error "CFG_TUSB_MCU must be defined by the platform Makefile"
#endif

#define CFG_TUSB_OS             OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUSB_DEBUG          0

/* SAM7S UDP: EP0 is 8 bytes by hardware, but setting this to 64 here
 * affects only the device descriptor's bMaxPacketSize0 field. We
 * configure the actual EP0 hardware bank in dcd_at91sam7s.c to match
 * whichever value this is. Real SAM7S EP0 FIFO is 8 bytes - stick to 8. */
#define CFG_TUD_ENDPOINT0_SIZE  8

#define CFG_TUD_CDC             1
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0

/* Vendor interface (WebUSB protobuf), reached via switch-mode (see hal.c). Small
 * FIFOs - SAM7S has only 64 KB SRAM, and one framed chunk is ~490 B. */
#define CFG_TUD_VENDOR          1
#define CFG_TUD_VENDOR_RX_BUFSIZE 512
#define CFG_TUD_VENDOR_TX_BUFSIZE 512
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
