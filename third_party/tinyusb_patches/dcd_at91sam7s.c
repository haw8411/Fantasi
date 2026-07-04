/* TinyUSB device-controller driver for Atmel AT91SAM7S UDP.
 *
 * Derived from dcd_samg.c (Copyright 2018, hathach, MIT licence).
 * Adapted for the AT91SAM7S/ARM7TDMI variant used on the Proxmark3:
 *   - Register names translated from SAMG (<sam.h>) to AT91SAM7S
 *     (<at91sam7s512.h>): UDP_CSR_xxx_Msk → AT91C_UDP_xxx etc.
 *   - Interrupt controller is the Atmel AIC, not the Cortex-M NVIC.
 *   - USB pull-up is driven by a GPIO pin (PA24 on PM3 Easy), not
 *     UDP_TXVC_PUON — SAM7S doesn't have an integrated software pull-up.
 *   - Peripheral clock enable runs through PMC_PCER and PMC_SCER;
 *     the PLL USBDIV must be configured for 48 MHz UDP clock.
 *
 * SAM7S UDP has 8 endpoints (EP0-EP7). EP0 is 8 bytes, EP1/2 are 64
 * bytes with ping-pong, EP3 is 64 bytes single-bank, EP4/5 are 256
 * bytes with ping-pong. We treat EP count as 8 here; TinyUSB's CDC
 * class uses EP0/EP1 (notif)/EP2 (BULK) which fits trivially. */

#include "tusb_option.h"

#if CFG_TUSB_MCU == OPT_MCU_AT91SAM7S

#include "at91sam7s512.h"
#include "device/dcd.h"

#ifndef GPIO_USB_PU
#  define GPIO_USB_PU  AT91C_PIO_PA24     /* PM3 Easy USB_DP_PUP control */
#endif

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

/* SAM7S UDP has 4 endpoints (EP0-EP3); the at91sam7s512.h vendor
 * header only declares UDP_CSR[4]/UDP_FDR[4] for that reason. CDC
 * maps to EP0 control, EP1 interrupt IN (notification), EP2 bulk
 * OUT, EP3 bulk IN — matching the endpoint numbers we use in the
 * PM3 descriptor override. */
#define EP_COUNT    4
#define UDP         AT91C_BASE_UDP

typedef struct {
    uint8_t*  buffer;
    uint16_t  total_len;
    volatile uint16_t actual_len;
    uint16_t  epsize;
} xfer_desc_t;

static xfer_desc_t _dcd_xfer[EP_COUNT];

static void xfer_epsize_set(xfer_desc_t* xfer, uint16_t epsize) { xfer->epsize = epsize; }

static void xfer_begin(xfer_desc_t* xfer, uint8_t* buffer, uint16_t total_bytes) {
    xfer->buffer = buffer;
    xfer->total_len = total_bytes;
    xfer->actual_len = 0;
}

static void xfer_end(xfer_desc_t* xfer) {
    xfer->buffer = NULL;
    xfer->total_len = 0;
    xfer->actual_len = 0;
}

static uint16_t xfer_packet_len(xfer_desc_t* xfer) {
    return tu_min16(xfer->total_len - xfer->actual_len, xfer->epsize);
}

static void xfer_packet_done(xfer_desc_t* xfer) {
    uint16_t const xact_len = xfer_packet_len(xfer);
    xfer->buffer += xact_len;
    xfer->actual_len += xact_len;
}

static void xact_ep_write(uint8_t epnum, uint8_t* buffer, uint16_t xact_len) {
    for (uint16_t i = 0; i < xact_len; i++)
        UDP->UDP_FDR[epnum] = (uint32_t)buffer[i];
}

static void xact_ep_read(uint8_t epnum, uint8_t* buffer, uint16_t xact_len) {
    for (uint16_t i = 0; i < xact_len; i++)
        buffer[i] = (uint8_t)UDP->UDP_FDR[epnum];
}

/* Bits in UDP_CSR that must be written as 1 to "have no effect" —
 * i.e. writing 0 to them would clear an event flag. */
#define CSR_NO_EFFECT_1_ALL \
    (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1 | \
     AT91C_UDP_STALLSENT   | AT91C_UDP_RXSETUP    | AT91C_UDP_TXCOMP)

/* UDP_CSR writes are asynchronous: the peripheral's MCK-domain write
 * completes several cycles after the bus access. The datasheet
 * requires polling the register after each write ("wait for the end
 * of the write operation before executing another write by polling
 * the bits which are being written"). A fixed cycle delay is
 * unreliable because read-after-write latency depends on pipeline
 * state and AMBA bridge buffering.
 *
 * Subtlety: csr_set/csr_clear use the RMW pattern with
 * CSR_NO_EFFECT_1_ALL, which ORs in bits like TXCOMP. Writing 1 to
 * TXCOMP is by design a no-op (only hardware sets it, only software
 * clears via 0). So the "I just wrote" value won't match readback
 * on those bits. The poll therefore compares only the specific bits
 * the caller asked to set or clear — those are the ones software
 * expects to see commit. */
#define CSR_WAIT_LIMIT   512u

static inline void csr_sync_bits(uint8_t epnum, uint32_t mask, uint32_t want) {
    for (uint32_t n = 0; n < CSR_WAIT_LIMIT; n++) {
        if ((UDP->UDP_CSR[epnum] & mask) == want) return;
    }
}

/* Raw write (no poll) — caller provides its own sync if it needs one. */
static inline void csr_write(uint8_t epnum, uint32_t value) {
    UDP->UDP_CSR[epnum] = value;
    /* The R/W bits DIR/FORCESTALL/EPTYPE/EPEDS will latch the
     * written value directly. Poll on those so a subsequent read
     * sees the new state. */
    uint32_t const rw_mask = AT91C_UDP_DIR | AT91C_UDP_FORCESTALL |
                             AT91C_UDP_EPTYPE | AT91C_UDP_EPEDS;
    csr_sync_bits(epnum, rw_mask, value & rw_mask);
}

static inline void csr_set(uint8_t epnum, uint32_t mask) {
    UDP->UDP_CSR[epnum] = UDP->UDP_CSR[epnum] | CSR_NO_EFFECT_1_ALL | mask;
    /* Expect the set bits to now read back as 1. Status bits aren't
     * software-settable (writing 1 is no-op), so csr_set is meaningful
     * only for the non-status bits: DIR, FORCESTALL, TXPKTRDY, EPEDS
     * and EPTYPE. A caller using csr_set with a status bit will
     * correctly skip the check because expected == current for bits
     * that never change on write. */
    csr_sync_bits(epnum, mask, mask);
}

static inline void csr_clear(uint8_t epnum, uint32_t mask) {
    UDP->UDP_CSR[epnum] = (UDP->UDP_CSR[epnum] | CSR_NO_EFFECT_1_ALL) & ~mask;
    /* Expect every bit in mask to read back as 0 after write
     * commits. Works for both status (write-0-clears) and R/W bits. */
    csr_sync_bits(epnum, mask, 0);
}

/*------------------------------------------------------------------*/
/* Device API                                                       */
/*------------------------------------------------------------------*/

static void bus_reset(void) {
    tu_memclr(_dcd_xfer, sizeof(_dcd_xfer));
    xfer_epsize_set(&_dcd_xfer[0], CFG_TUD_ENDPOINT0_SIZE);

    /* Reset all endpoints, then enable EP0 as control. A brief
     * deliberate delay between assert and release gives UDP_RSTEP's
     * minimum-pulse requirement (≥2 MCK cycles per datasheet) some
     * slack under AMBA write-buffering. */
    UDP->UDP_RSTEP  = 0xFFFFFFFFU;
    for (volatile uint32_t n = 0; n < 32; n++) { __asm volatile("nop"); }
    UDP->UDP_RSTEP  = 0;
    UDP->UDP_FADDR  = AT91C_UDP_FEN;
    csr_write(0, AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);

    /* Re-assert the full IER set — including ENDBUSRES so a second
     * bus reset (host re-enumerate) is still caught, and EPINT0 so
     * the first SETUP after reset generates a status bit. The
     * observed 0x1200-only IMR in the earlier hardware trace is
     * defended against here: on every bus reset, IER is rewritten
     * with all the bits we care about. */
    UDP->UDP_IER = AT91C_UDP_EPINT0    | AT91C_UDP_RXSUSP |
                   AT91C_UDP_RXRSM     | AT91C_UDP_WAKEUP |
                   AT91C_UDP_ENDBUSRES;

    /* Make sure the UDP transceiver is on (clear TXVDIS). */
    UDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;
}

void dcd_init(uint8_t rhport) {
    (void)rhport;

    /* Program the PLL's USB divider to /1 so UDPCK = 48 MHz. The PM3
     * bootrom programs the main PLL for 96 MHz, and this CKGR_USBDIV
     * field lives in the top bits of CKGR_PLLR — OR it in rather than
     * overwriting so we preserve the MUL/DIV values. */
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1;

    /* Bring up the peripheral clock (PMC_PCER bit 11) and the USB
     * system clock (PMC_SCER.UDP = bit 7). Both are needed before
     * any UDP register access. */
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_UDP);
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;

    /* Reset UDP state — FADDR = 0 (unaddressed), GLBSTATE = 0
     * (unconfigured, no FADDEN yet). Matches stock PM3's usb_enable()
     * sequence; some hosts refuse a "fresh" device if we come up with
     * FADDEN already asserted from a previous configuration. */
    UDP->UDP_FADDR    = 0;
    UDP->UDP_GLBSTATE = 0;

    tu_memclr(_dcd_xfer, sizeof(_dcd_xfer));
    /* Set EP0 epsize now so the DCD can respond correctly to a SETUP
     * that arrives before the first ENDBUSRES (unusual but possible
     * on warm attach when pullup is already up). bus_reset() will
     * tu_memclr the array again on ENDBUSRES and re-set this. */
    xfer_epsize_set(&_dcd_xfer[0], CFG_TUD_ENDPOINT0_SIZE);

    /* Park the pull-up GPIO low (PIO-mode, output, clear) so the host
     * sees a clean disconnect. dcd_connect() drives it high at end of
     * init. */
    AT91C_BASE_PIOA->PIO_PER  = GPIO_USB_PU;
    AT91C_BASE_PIOA->PIO_OER  = GPIO_USB_PU;
    AT91C_BASE_PIOA->PIO_CODR = GPIO_USB_PU;

    /* Transceiver disabled until we fully configure below. */
    UDP->UDP_TXVC = AT91C_UDP_TXVDIS;

    /* Initial EP0 state: enabled as control endpoint. bus_reset()
     * re-asserts this on every ENDBUSRES so the re-enumerate path is
     * covered too, but we need it set now for the first SETUP. */
    csr_write(0, AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);
    UDP->UDP_IER = AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM |
                   AT91C_UDP_WAKEUP | AT91C_UDP_ENDBUSRES;

    dcd_connect(rhport);
}

/* Enable/disable the UDP IRQ at the AIC. We *must* service the DCD
 * from interrupt context: Linux xhci tolerates only ~150 µs of NAK
 * bursts before failing a control transfer with EPROTO (-71). A
 * FreeRTOS task polling at the 1 ms tick rate is way too slow —
 * the SETUP arrives, hardware NAKs every IN token until the next
 * poll, and xhci aborts before we ever see the first IN. The AIC
 * trampoline configured in hal.c dispatches to dcd_int_handler(). */
void dcd_int_enable(uint8_t rhport) {
    (void)rhport;
    AT91C_BASE_AIC->AIC_IECR = (1u << AT91C_ID_UDP);
}
void dcd_int_disable(uint8_t rhport) {
    (void)rhport;
    AT91C_BASE_AIC->AIC_IDCR = (1u << AT91C_ID_UDP);
}

/* USB spec: address change takes effect only AFTER the host ACKs
 * the status-stage ZLP. The SAMG DCD pattern defers the FADDR write
 * to dcd_edpt0_status_complete, which TinyUSB's usbd_control calls
 * after the IN ZLP transfer completes. We do the same, and we hold
 * on to dev_addr in a file-static so the status-complete callback
 * has the value (it receives only the setup request, which we could
 * also decode — keeping the cached value is simpler and matches the
 * SAMG pattern). */
static uint8_t _pending_dev_addr;
void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
    _pending_dev_addr = dev_addr;
    dcd_edpt_xfer(rhport, 0x80, NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport) {
    (void)rhport;
    /* SAM7S UDP supports remote wakeup via UDP_GLBSTATE.ESR; leaving
     * unimplemented until a host actually needs it. */
}

void dcd_connect(uint8_t rhport) {
    (void)rhport;
    /* Enable the transceiver and drive PA24 high to attach the 1.5 kΩ
     * pull-up on D+. Host will see a full-speed device attach event. */
    UDP->UDP_TXVC = 0;                     /* TXVDIS = 0 → transceiver active */
    AT91C_BASE_PIOA->PIO_SODR = GPIO_USB_PU;
}

void dcd_disconnect(uint8_t rhport) {
    (void)rhport;
    AT91C_BASE_PIOA->PIO_CODR = GPIO_USB_PU;
    UDP->UDP_TXVC = AT91C_UDP_TXVDIS;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
    (void)rhport; (void)en;
    /* SOF interrupts aren't routed to usbd; leave unimplemented. */
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const* request) {
    (void)rhport;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
        if (request->bRequest == TUSB_REQ_SET_ADDRESS) {
            uint8_t const dev_addr = (uint8_t)request->wValue;
            /* Belt-and-braces: enable FADDEN first, then write the new
             * address. The datasheet allows these in either order as
             * long as both are done before the next SETUP. Do GLBSTATE
             * first so FEN + addr in one FADDR write is observed with
             * FADDEN already latched. */
            UDP->UDP_GLBSTATE |= AT91C_UDP_FADDEN;
            UDP->UDP_FADDR = AT91C_UDP_FEN | (dev_addr & 0x7F);
            (void)_pending_dev_addr;
        } else if (request->bRequest == TUSB_REQ_SET_CONFIGURATION) {
            UDP->UDP_GLBSTATE |= AT91C_UDP_CONFG;
        }
    }
}

/* SAM7S, like SAMG, can't have the same EP number as both IN and OUT
 * at the same time — the CSR EPTYPE field encodes direction. */
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* ep_desc) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_desc->bEndpointAddress);
    uint8_t const dir   = tu_edpt_dir(ep_desc->bEndpointAddress);

    TU_VERIFY(ep_desc->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS);
    TU_VERIFY(epnum < EP_COUNT);
    TU_ASSERT((UDP->UDP_CSR[epnum] & AT91C_UDP_EPEDS) == 0);

    xfer_epsize_set(&_dcd_xfer[epnum], tu_edpt_packet_size(ep_desc));

    /* EPTYPE field: {0,1,2,3} = CTRL/ISO_OUT/BULK_OUT/INT_OUT,
     *               {5,6,7}   = ISO_IN/BULK_IN/INT_IN (bit 10 = dir). */
    uint32_t const eptype = (ep_desc->bmAttributes.xfer + (4u * dir)) << 8;
    csr_write(epnum, AT91C_UDP_EPEDS | eptype);

    /* IN endpoints: enable CSR interrupt up-front so TXCOMP is caught.
     * OUT endpoints: interrupt is enabled on demand in dcd_edpt_xfer(). */
    if (dir == TUSB_DIR_IN) UDP->UDP_IER = (1u << epnum);

    return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport; (void)ep_addr;
    /* usbd only calls this on device reset; bus_reset() already
     * clears all endpoints so nothing to do here. */
}

void dcd_edpt_close_all(uint8_t rhport) {
    (void)rhport;
    for (uint8_t i = 1; i < EP_COUNT; i++) {
        csr_write(i, 0);
        UDP->UDP_IDR = (1u << i);
    }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);

    xfer_desc_t* xfer = &_dcd_xfer[epnum];
    xfer_begin(xfer, buffer, total_bytes);

    if (dir == TUSB_DIR_OUT) {
        if (epnum != 0) UDP->UDP_IER = (1u << epnum);
    } else {
        /* Gate the UDP interrupt during FDR writes to prevent the ISR
         * from reading a different endpoint's FDR concurrently — the
         * shared DPRAM bus causes byte leakage between FIFOs. */
        AT91C_BASE_AIC->AIC_IDCR = (1u << AT91C_ID_UDP);
        xact_ep_write(epnum, xfer->buffer, xfer_packet_len(xfer));
        csr_set(epnum, AT91C_UDP_TXPKTRDY);
        AT91C_BASE_AIC->AIC_IECR = (1u << AT91C_ID_UDP);
    }
    return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;
    /* usbd pairs EP0 IN+OUT stalls; handle only one side to match SAMG. */
    if (ep_addr == tu_edpt_addr(0, TUSB_DIR_IN_MASK)) return;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    csr_set(epnum, AT91C_UDP_FORCESTALL);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    csr_clear(epnum, AT91C_UDP_FORCESTALL);
    /* Pulse RSTEP to reset the data-toggle back to DATA0. */
    UDP->UDP_RSTEP |=  (1u << epnum);
    UDP->UDP_RSTEP &= ~(1u << epnum);
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport) {
    /* NOTE: gated on UDP_ISR directly, NOT UDP_ISR & UDP_IMR. The
     * earlier hardware trace showed UDP_IMR sometimes losing bits
     * between tud_init's UDP_IER write and the first interrupt, which
     * then caused us to silently skip SETUP packets. This handler is
     * called from both AIC IRQ context AND our 1 ms polling loop, so
     * it must re-discover pending work from the register state alone.
     *
     * UDP_ISR's bits reflect hardware-visible conditions regardless
     * of masking; UDP_IMR only controls whether those conditions
     * drive NIRQ. Using ISR directly means a lost IMR bit degrades
     * latency but not correctness. */
    uint32_t const intr_status = UDP->UDP_ISR;

    /* Clear the system-level status bits we service. The per-EP
     * bits (0..3) are read-only through ICR; they clear only when
     * the corresponding CSR event bits clear. */
    UDP->UDP_ICR = intr_status & (AT91C_UDP_ENDBUSRES | AT91C_UDP_RXSUSP |
                                  AT91C_UDP_RXRSM | AT91C_UDP_WAKEUP |
                                  AT91C_UDP_SOFINT);

    if (intr_status & AT91C_UDP_ENDBUSRES) {
        bus_reset();
        dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
    }
    if (intr_status & AT91C_UDP_RXSUSP)  dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
    if (intr_status & AT91C_UDP_RXRSM)   dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
    if (intr_status & AT91C_UDP_WAKEUP)  dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);

    /* EP0 SETUP: check CSR[0] directly rather than gating on
     * intr_status's EPINT0 bit. Per datasheet, DIR must be configured
     * while RXSETUP is still asserted and BEFORE it is cleared, so
     * csr_set(DIR) must be read-back-synchronised before csr_clear
     * RXSETUP runs — csr_write handles that. */
    if (UDP->UDP_CSR[0] & AT91C_UDP_RXSETUP) {
        uint8_t setup[8];
        for (uint8_t i = 0; i < 8; i++) setup[i] = (uint8_t)UDP->UDP_FDR[0];

        dcd_event_setup_received(rhport, setup, true);

        /* DIR for the data stage. Only SET DIR for IN requests;
         * never clear it — matches PM3's stock usb_cdc.c which is
         * known to enumerate reliably on the same silicon. Clearing
         * DIR empirically worsens enumeration (more -110 timeouts,
         * fewer -71 which means at least the hardware was answering
         * something). The datasheet's DIR description doesn't
         * document a hardware auto-clear on RXSETUP, but the PM3
         * working reference suggests the state machine is quite
         * forgiving if DIR is only *raised* for IN stages and left
         * alone otherwise. */
        if (tu_edpt_dir(setup[0])) csr_set(0, AT91C_UDP_DIR);

        csr_clear(0, AT91C_UDP_RXSETUP | AT91C_UDP_TXPKTRDY |
                     AT91C_UDP_TXCOMP  | AT91C_UDP_RX_DATA_BK0 |
                     AT91C_UDP_RX_DATA_BK1 | AT91C_UDP_STALLSENT |
                     AT91C_UDP_FORCESTALL);
    }

    /* Process OUT (FDR reads) and IN (FDR writes) in separate passes.
     * The AT91SAM7S UDP's DPRAM bus exhibits byte leakage between
     * endpoint FIFOs when reads and writes to different FDR[] indices
     * occur back-to-back without settling time. */

    /* Pass 1: OUT endpoints — read from FDRs */
    for (uint8_t epnum = 0; epnum < EP_COUNT; epnum++) {
        uint32_t const csr = UDP->UDP_CSR[epnum];
        xfer_desc_t* xfer = &_dcd_xfer[epnum];

        uint32_t const banks_complete = csr & (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1);
        if (banks_complete) {
            uint16_t const xact_len =
                (uint16_t)((csr & AT91C_UDP_RXBYTECNT) >> 16);

            if (epnum == 0 && xact_len == 0 &&
                xfer->total_len > 0 && xfer->actual_len < xfer->total_len) {
                dcd_event_xfer_complete(rhport, epnum | TUSB_DIR_IN_MASK,
                                        xfer->actual_len, XFER_RESULT_SUCCESS, true);
                xfer_end(xfer);
                csr_clear(epnum, banks_complete);
                continue;
            }

            xact_ep_read(epnum, xfer->buffer, xact_len);
            xfer->buffer    += xact_len;
            xfer->actual_len += xact_len;

            if (xact_len < xfer->epsize || xfer->actual_len >= xfer->total_len) {
                if (epnum != 0) UDP->UDP_IDR = (1u << epnum);
                dcd_event_xfer_complete(rhport, epnum, xfer->actual_len,
                                        XFER_RESULT_SUCCESS, true);
                xfer_end(xfer);
            }
            csr_clear(epnum, banks_complete);
        }

        if (csr & AT91C_UDP_STALLSENT) {
            csr_clear(epnum, AT91C_UDP_STALLSENT);
        }
    }

    /* DPRAM settling — a read of a non-FDR register forces the bus to
     * complete any pending FIFO access before we switch direction. */
    (void)UDP->UDP_NUM;

    /* Pass 2: IN endpoints — write to FDRs */
    for (uint8_t epnum = 0; epnum < EP_COUNT; epnum++) {
        uint32_t const csr = UDP->UDP_CSR[epnum];
        xfer_desc_t* xfer = &_dcd_xfer[epnum];

        if (csr & AT91C_UDP_TXCOMP) {
            xfer_packet_done(xfer);
            uint16_t const xact_len = xfer_packet_len(xfer);

            if (xact_len) {
                xact_ep_write(epnum, xfer->buffer, xact_len);
                csr_set(epnum, AT91C_UDP_TXPKTRDY);
            } else {
                dcd_event_xfer_complete(rhport, epnum | TUSB_DIR_IN_MASK,
                                        xfer->actual_len, XFER_RESULT_SUCCESS, true);
                xfer_end(xfer);
            }
            csr_clear(epnum, AT91C_UDP_TXCOMP);
        }
    }
}

#endif /* CFG_TUSB_MCU == OPT_MCU_AT91SAM7S */
