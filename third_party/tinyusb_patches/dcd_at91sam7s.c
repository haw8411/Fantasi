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
    uint8_t   active;
    uint8_t   dir;
} xfer_desc_t;

_Static_assert(sizeof(xfer_desc_t) == 12,
               "SAM7S transfer direction must fit existing struct padding");

/* EP0 needs more detail than a generic active bit because a new SETUP can
 * precede task-level completion of the current transfer. Store the phase in
 * existing structure padding. */
enum {
    XFER_IDLE = 0,
    XFER_ACTIVE,
    EP0_WAIT,
    EP0_DATA_OUT,
    EP0_DATA_IN,
    EP0_STATUS_OUT,
    EP0_STATUS_IN,
    EP0_DATA_OUT_DONE,
    EP0_DATA_IN_DONE,
    EP0_STATUS_OUT_DONE,
    EP0_STATUS_IN_DONE,
    EP0_PENDING_DATA_OUT,
    EP0_PENDING_STATUS_OUT,
    EP0_DATA_OUT_EARLY,
    EP0_STATUS_OUT_EARLY,
};

static xfer_desc_t _dcd_xfer[EP_COUNT];

/* EP1 and EP2 have ping-pong receive FIFOs.  When both banks are full the
 * SAM7S exposes both RX_DATA_BK bits, but provides no indication of which bank
 * was filled first.  The datasheet therefore requires software to remember
 * the alternating consume order.  One bit per endpoint is enough; keeping the
 * mask here costs one fixed byte for the controller, not per transfer/session. */
static uint8_t _out_bank1_mask;
static volatile uint8_t _queue_deferred_needed;

/* Stage an EP0 OUT packet that arrives before TinyUSB publishes its buffer.
 * Releasing the bank and keeping EP0 enabled allows a superseding SETUP to be
 * serviced immediately. Every v2 data stage fits this fixed buffer. */
static uint8_t _ep0_stage[8];
static uint8_t _ep0_stage_len;

extern bool fantasi_usbd_event_queue_has_space(uint8_t count);
/* Number of free queue entries required before a retained UDP condition may
 * be serviced.  Keeping the exact requirement closes two races that a boolean
 * cannot: one dequeue may still leave too little room, and normal EP0 arming
 * must not unmask the AIC while backpressure is active. */

/* A SETUP packet aborts the preceding EP0 transaction, but completion events
 * for that transaction may already be waiting in TinyUSB's task queue. Keep a
 * seven-bit transaction epoch and pack it into otherwise-unused bits of
 * xfer_desc_t.dir and EP0 completion lengths. This is two fixed DCD bytes,
 * never per-session state. */
#define EP0_EPOCH_MASK          0x7fu
#define EP0_EPOCH_LEN_SHIFT     24u
#define EP0_TASK_EPOCH_INVALID  0xffu

static volatile uint8_t _ep0_setup_epoch;
static volatile uint8_t _ep0_task_epoch = EP0_TASK_EPOCH_INVALID;

static void xfer_epsize_set(xfer_desc_t* xfer, uint16_t epsize) { xfer->epsize = epsize; }

static uint8_t xfer_dir(xfer_desc_t const* xfer) {
    return xfer->dir & 1u;
}

static uint8_t xfer_epoch(xfer_desc_t const* xfer) {
    return (xfer->dir >> 1) & EP0_EPOCH_MASK;
}

static void ep0_context_set(xfer_desc_t* xfer, uint8_t dir) {
    xfer->dir = (dir & 1u) | (uint8_t)(_ep0_setup_epoch << 1);
}

static void ep0_epoch_invalidate(void) {
    _ep0_setup_epoch = (uint8_t)((_ep0_setup_epoch + 1u) & EP0_EPOCH_MASK);
    _ep0_task_epoch = EP0_TASK_EPOCH_INVALID;
}

/* usbd.c calls this while its queued-SETUP count is protected. A later SETUP
 * ISR changes _ep0_setup_epoch, so a stale task callback cannot arm a phase
 * of the replacement transaction. */
void fantasi_dcd_control_task_sync(void) {
    _ep0_task_epoch = _ep0_setup_epoch;
}

bool fantasi_dcd_control_event_matches_task(uint8_t epoch) {
    /* A real completion for the transaction the task is retiring remains
     * meaningful even if the ISR has already received the next SETUP. The
     * task callback must run to advance TinyUSB and commit status effects;
     * dcd_edpt_xfer() separately rejects any phase it tries to arm against
     * the newer hardware epoch. */
    return epoch == _ep0_task_epoch;
}

static void ep0_event_xfer_complete(uint8_t rhport, uint8_t ep_addr,
                                    uint32_t len, xfer_desc_t const* xfer) {
    uint32_t const tagged_len = (len & 0xffffu) |
                                ((uint32_t)xfer_epoch(xfer) << EP0_EPOCH_LEN_SHIFT);
    dcd_event_xfer_complete(rhport, ep_addr, tagged_len,
                            XFER_RESULT_SUCCESS, true);
}

/* Publish an EP0 completion from USB-task context (early-consumed phases are
 * finished inside dcd_edpt_xfer). The FreeRTOS ARM7 port's FromISR queue
 * primitives are unsafe from a task, and the blocking task-side send would
 * deadlock the queue's own consumer if it were ever full - so this uses a
 * zero-timeout send provided by usbd.c and reports failure instead. */
bool fantasi_usbd_queue_event_task(dcd_event_t const* event);
static bool ep0_event_publish_task(uint8_t rhport, uint8_t ep_addr,
                                   uint32_t len, xfer_desc_t const* xfer) {
    dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_XFER_COMPLETE };
    event.xfer_complete.ep_addr = ep_addr;
    event.xfer_complete.len = (len & 0xffffu) |
                              ((uint32_t)xfer_epoch(xfer) << EP0_EPOCH_LEN_SHIFT);
    event.xfer_complete.result = XFER_RESULT_SUCCESS;
    return fantasi_usbd_queue_event_task(&event);
}

/* Keep the hardware condition asserted until TinyUSB has queue capacity for
 * every event the DCD is about to publish.  Masking the AIC source prevents a
 * level-interrupt storm; the USB task calls fantasi_dcd_event_queue_space()
 * immediately after it removes an event, so the retained condition is retried
 * without a polling-tick delay. */
static bool event_queue_defer(uint8_t needed) {
    if (fantasi_usbd_event_queue_has_space(needed)) return false;
    if (needed > _queue_deferred_needed) _queue_deferred_needed = needed;
    AT91C_BASE_AIC->AIC_IDCR = (1u << AT91C_ID_UDP);
    return true;
}

void fantasi_dcd_event_queue_space(void) {
    uint8_t const needed = _queue_deferred_needed;
    if (!needed || !fantasi_usbd_event_queue_has_space(needed)) return;
    _queue_deferred_needed = 0;
    AT91C_BASE_AIC->AIC_IECR = (1u << AT91C_ID_UDP);
}

static void udp_irq_enable_if_ready(void) {
    if (!_queue_deferred_needed)
        AT91C_BASE_AIC->AIC_IECR = (1u << AT91C_ID_UDP);
}

/* IER/IDR command writes also cross into the UDP clock domain. A plain write
 * can fail to update IMR before the CPU continues.  For EP0 that loses a SETUP;
 * for a ping-pong bulk OUT endpoint a late IDR lets the retained second bank
 * re-enter the ISR after its descriptor completed, so it is consumed into the
 * idle descriptor.  Confirm every endpoint mask transition synchronously. */
#define UDP_MASK_WAIT_LIMIT 512u
static void endpoint_irq_enable(uint8_t epnum) {
    uint32_t const mask = 1u << epnum;
    for (uint32_t n = 0; n < UDP_MASK_WAIT_LIMIT; n++) {
        UDP->UDP_IER = mask;
        if (UDP->UDP_IMR & mask) return;
    }
}

static void endpoint_irq_disable(uint8_t epnum) {
    uint32_t const mask = 1u << epnum;
    for (uint32_t n = 0; n < UDP_MASK_WAIT_LIMIT; n++) {
        UDP->UDP_IDR = mask;
        if (!(UDP->UDP_IMR & mask)) return;
    }
}

static void ep0_irq_enable(void)  { endpoint_irq_enable(0); }
static void ep0_irq_disable(void) { endpoint_irq_disable(0); }

/* The USB task calls this after draining an event (or its bounded idle wait).
 * It is not a reset: it only restores an interrupt gate if a retained SETUP or
 * an already-serviced queue-backpressure condition was left masked. This is
 * what makes the single-bank EP0 self-livening. */
void fantasi_dcd_poll(void) {
    fantasi_dcd_event_queue_space();

    uint32_t const csr = UDP->UDP_CSR[0];
    uint32_t const udp_imr = UDP->UDP_IMR;
    uint32_t const aic_imr = AT91C_BASE_AIC->AIC_IMR;
    bool const setup_masked = (csr & AT91C_UDP_RXSETUP) &&
                              !(udp_imr & AT91C_UDP_EPINT0);
    bool const irq_masked = !_queue_deferred_needed &&
                            !(aic_imr & (1u << AT91C_ID_UDP));
    if (!setup_masked && !irq_masked) return;

    if (setup_masked) ep0_irq_enable();
    if (irq_masked)   AT91C_BASE_AIC->AIC_IECR = (1u << AT91C_ID_UDP);
}

static void xfer_begin(xfer_desc_t* xfer, uint8_t* buffer, uint16_t total_bytes,
                       uint8_t dir) {
    xfer->buffer = buffer;
    xfer->total_len = total_bytes;
    xfer->actual_len = 0;
    xfer->active = XFER_ACTIVE;
    xfer->dir = dir;
}

static void xfer_end(xfer_desc_t* xfer) {
    xfer->buffer = NULL;
    xfer->total_len = 0;
    xfer->actual_len = 0;
    xfer->active = XFER_IDLE;
}

static void xfer_complete(uint8_t epnum, xfer_desc_t* xfer) {
    xfer->buffer = NULL;
    xfer->total_len = 0;
    xfer->actual_len = 0;
    if (epnum != 0) {
        xfer->active = XFER_IDLE;
        return;
    }

    switch (xfer->active) {
        case EP0_DATA_OUT:   xfer->active = EP0_DATA_OUT_DONE; break;
        case EP0_DATA_IN:    xfer->active = EP0_DATA_IN_DONE; break;
        case EP0_STATUS_OUT: xfer->active = EP0_STATUS_OUT_DONE; break;
        case EP0_STATUS_IN:  xfer->active = EP0_STATUS_IN_DONE; break;
        default:             xfer->active = XFER_IDLE; break;
    }
}

static bool ep0_out_active(uint8_t phase) {
    return phase == EP0_DATA_OUT || phase == EP0_STATUS_OUT;
}

static bool ep0_in_active(uint8_t phase) {
    return phase == EP0_DATA_IN || phase == EP0_STATUS_IN;
}

static bool ep0_out_pending(uint8_t phase) {
    return phase == EP0_PENDING_DATA_OUT ||
           phase == EP0_PENDING_STATUS_OUT;
}

static uint16_t xfer_packet_len(xfer_desc_t* xfer) {
    return tu_min16(xfer->total_len - xfer->actual_len, xfer->epsize);
}

static void xfer_packet_done(xfer_desc_t* xfer) {
    uint16_t const xact_len = xfer_packet_len(xfer);
    xfer->buffer += xact_len;
    xfer->actual_len += xact_len;
}

static bool endpoint_is_ping_pong(uint8_t epnum) {
    return epnum == 1 || epnum == 2;
}

static uint32_t out_bank_expected(uint8_t epnum) {
    return endpoint_is_ping_pong(epnum) && (_out_bank1_mask & (1u << epnum))
         ? AT91C_UDP_RX_DATA_BK1 : AT91C_UDP_RX_DATA_BK0;
}

static void out_bank_advance(uint8_t epnum) {
    if (endpoint_is_ping_pong(epnum)) _out_bank1_mask ^= (uint8_t)(1u << epnum);
}

static void out_bank_reset(uint8_t epnum) {
    _out_bank1_mask &= (uint8_t)~(1u << epnum);
}

static void endpoint_fifo_reset(uint8_t epnum) {
    uint32_t const mask = 1u << epnum;
    UDP->UDP_RSTEP |= mask;
    for (volatile uint32_t n = 0; n < 32; n++) __asm volatile("nop");
    UDP->UDP_RSTEP &= ~mask;
    out_bank_reset(epnum);
}

/* SAM7S requires three UDPCK plus three peripheral-clock cycles between a
 * RX_DATA_BKx/TXPKTRDY transition and any DPR access. The endpoint interrupt
 * can arrive before that interval has elapsed, so polling the CSR bit alone is
 * insufficient and occasionally exposes the preceding packet's FIFO bytes.
 * MCK and UDPCK are both 48 MHz on PM3; twenty instruction cycles comfortably
 * cover the required crossing without adding state or scheduler latency. */
static inline void dpr_sync_delay(void) {
    for (volatile uint32_t n = 0; n < 20; n++) __asm volatile("nop");
}

static void xact_ep_write(uint8_t epnum, uint8_t* buffer, uint16_t xact_len) {
    dpr_sync_delay();
    for (uint16_t i = 0; i < xact_len; i++)
        UDP->UDP_FDR[epnum] = (uint32_t)buffer[i];
}

static void xact_ep_read(uint8_t epnum, uint8_t* buffer, uint16_t xact_len,
                         uint16_t copy_len) {
    dpr_sync_delay();
    for (uint16_t i = 0; i < xact_len; i++) {
        uint8_t const value = (uint8_t)UDP->UDP_FDR[epnum];
        if (i < copy_len) buffer[i] = value;
    }
}

/* Bits in UDP_CSR that must be written as 1 to "have no effect" —
 * i.e. writing 0 to them would clear an event flag. */
#define CSR_NO_EFFECT_1_ALL \
    (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1 | \
     AT91C_UDP_STALLSENT   | AT91C_UDP_RXSETUP    | AT91C_UDP_TXCOMP)

#define UDP_ENDPOINT_INTS  ((1u << EP_COUNT) - 1u)
#define UDP_SYSTEM_INTS    (AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | \
                            AT91C_UDP_WAKEUP | AT91C_UDP_ENDBUSRES | \
                            AT91C_UDP_SOFINT)
#define UDP_USED_INTS      (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | \
                            AT91C_UDP_RXRSM | AT91C_UDP_WAKEUP | \
                            AT91C_UDP_ENDBUSRES)

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

/* RSTEP resets FIFO/toggle state but does not disable EPEDS. Always follow it
 * by clearing every CSR so a new TinyUSB personality starts with no endpoint
 * type, direction, enable, or latched transaction inherited from the old one. */
static void endpoints_reset_disable(void) {
    UDP->UDP_RSTEP = UDP_ENDPOINT_INTS;
    for (volatile uint32_t n = 0; n < 32; n++) { __asm volatile("nop"); }
    UDP->UDP_RSTEP = 0;
    _out_bank1_mask = 0;
    for (uint8_t epnum = 0; epnum < EP_COUNT; epnum++) csr_write(epnum, 0);
}

/*------------------------------------------------------------------*/
/* Device API                                                       */
/*------------------------------------------------------------------*/

static void bus_reset(void) {
    _queue_deferred_needed = 0;
    _ep0_stage_len = 0;
    ep0_epoch_invalidate();
    tu_memclr(_dcd_xfer, sizeof(_dcd_xfer));
    xfer_epsize_set(&_dcd_xfer[0], CFG_TUD_ENDPOINT0_SIZE);

    UDP->UDP_IDR = UDP_ENDPOINT_INTS;
    endpoints_reset_disable();
    UDP->UDP_FADDR  = AT91C_UDP_FEN;
    csr_write(0, AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);

    /* Restore the full interrupt set, including bus reset and EP0. */
    UDP->UDP_IER = UDP_USED_INTS;

    /* Make sure the UDP transceiver is on (clear TXVDIS). */
    UDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;
}

void dcd_init(uint8_t rhport) {
    (void)rhport;

    _queue_deferred_needed = 0;
    ep0_epoch_invalidate();

    /* The PM3 bootloader normally leaves the 96 MHz PLL and /2 USB divider
     * configured. Do not rewrite a live PLL on every USB personality switch.
     * If a foreign bootloader left another divider, change only that field and
     * wait until both the PLL and master clock are ready before touching UDP. */
    uint32_t const pllr = AT91C_BASE_CKGR->CKGR_PLLR;
    if ((pllr & AT91C_CKGR_USBDIV) != AT91C_CKGR_USBDIV_1) {
        AT91C_BASE_CKGR->CKGR_PLLR =
            (pllr & ~AT91C_CKGR_USBDIV) | AT91C_CKGR_USBDIV_1;
        while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK)) {}
        while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY)) {}
    }

    /* Bring up the peripheral clock (PMC_PCER bit 11) and the USB
     * system clock (PMC_SCER.UDP = bit 7). Both are needed before
     * any UDP register access. */
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_UDP);
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;

    /* Establish a genuinely detached, interrupt-silent controller before
     * touching address or endpoint state. This path is used both at boot and
     * by an in-place TinyUSB personality restart. */
    AT91C_BASE_PIOA->PIO_PER  = GPIO_USB_PU;
    AT91C_BASE_PIOA->PIO_OER  = GPIO_USB_PU;
    AT91C_BASE_PIOA->PIO_CODR = GPIO_USB_PU;
    UDP->UDP_TXVC = AT91C_UDP_TXVDIS;
    UDP->UDP_IDR = UDP_ENDPOINT_INTS | UDP_SYSTEM_INTS;
    UDP->UDP_ICR = UDP_SYSTEM_INTS;
    endpoints_reset_disable();

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

    /* Initial EP0 state: enabled as control endpoint. bus_reset()
     * re-asserts this on every ENDBUSRES so the re-enumerate path is
     * covered too, but we need it set now for the first SETUP. */
    csr_write(0, AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);
    UDP->UDP_IER = UDP_USED_INTS;

    dcd_connect(rhport);
}

bool dcd_deinit(uint8_t rhport) {
    (void)rhport;
    /* tud_deinit() has already disabled the AIC and called dcd_disconnect(). */
    return true;
}

/* Service the DCD from interrupt context. Tick-rate task polling cannot meet
 * control-transfer latency. The AIC trampoline in hal.c dispatches here. */
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
    if (_ep0_task_epoch != _ep0_setup_epoch) return;
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
    AT91C_BASE_PIOA->PIO_PER = GPIO_USB_PU;
    AT91C_BASE_PIOA->PIO_OER = GPIO_USB_PU;
    UDP->UDP_TXVC = 0;                     /* TXVDIS = 0 → transceiver active */
    AT91C_BASE_PIOA->PIO_SODR = GPIO_USB_PU;
    (void)AT91C_BASE_PIOA->PIO_PDSR;       /* complete the bridge write */
}

void dcd_disconnect(uint8_t rhport) {
    (void)rhport;
    _queue_deferred_needed = 0;
    _ep0_stage_len = 0;
    ep0_epoch_invalidate();
    AT91C_BASE_PIOA->PIO_CODR = GPIO_USB_PU;
    UDP->UDP_TXVC = AT91C_UDP_TXVDIS;

    /* Quiesce every level source and endpoint before TinyUSB deletes its event
     * queue. dcd_init() re-enables the complete interrupt set before the next
     * attach, so no detached-period status needs to be retained. */
    UDP->UDP_IDR = UDP_ENDPOINT_INTS | UDP_SYSTEM_INTS;
    UDP->UDP_ICR = UDP_SYSTEM_INTS;
    endpoints_reset_disable();
    for (uint8_t epnum = 0; epnum < EP_COUNT; epnum++) {
        xfer_end(&_dcd_xfer[epnum]);
    }
    UDP->UDP_FADDR = 0;
    UDP->UDP_GLBSTATE = 0;
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
    out_bank_reset(epnum);

    /* EPTYPE field: {0,1,2,3} = CTRL/ISO_OUT/BULK_OUT/INT_OUT,
     *               {5,6,7}   = ISO_IN/BULK_IN/INT_IN (bit 10 = dir). */
    uint32_t const eptype = (ep_desc->bmAttributes.xfer + (4u * dir)) << 8;
    csr_write(epnum, AT91C_UDP_EPEDS | eptype);

    /* IN endpoints: enable CSR interrupt up-front so TXCOMP is caught.
     * OUT endpoints: interrupt is enabled on demand in dcd_edpt_xfer(). */
    if (dir == TUSB_DIR_IN) endpoint_irq_enable(epnum);

    return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport; (void)ep_addr;
    /* usbd only calls this on device reset; bus_reset() already
     * clears all endpoints so nothing to do here. */
}

void dcd_edpt_close_all(uint8_t rhport) {
    (void)rhport;
    _out_bank1_mask = 0;
    for (uint8_t i = 1; i < EP_COUNT; i++) {
        csr_write(i, 0);
        endpoint_irq_disable(i);
    }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);

    xfer_desc_t* xfer = &_dcd_xfer[epnum];
    /* EP0 can receive the next stage immediately. Make observing a held bank
     * and publishing its descriptor atomic with respect to the USB IRQ. */
    if (epnum == 0) {
        AT91C_BASE_AIC->AIC_IDCR = (1u << AT91C_ID_UDP);
        /* The task may have been pre-empted by the next SETUP after checking
         * a completion. The epoch comparison rejects that stale arm. Do not
         * test RXSETUP here: its clear crosses asynchronously from UDPCK and
         * can remain observable briefly after the ISR published this valid
         * SETUP to the task. */
        if (_ep0_task_epoch != _ep0_setup_epoch) {
            udp_irq_enable_if_ready();
            return true;
        }
    }
    bool ep0_pending = false;
    uint8_t ep0_prior = XFER_IDLE;
    if (epnum == 0) {
        ep0_prior = xfer->active;
        ep0_pending = ep0_out_pending(ep0_prior);
        uint8_t const setup_dir = xfer_dir(xfer);
        xfer->buffer = buffer;
        xfer->total_len = total_bytes;
        xfer->actual_len = 0;
        if (dir == setup_dir) {
            xfer->active = dir == TUSB_DIR_IN ? EP0_DATA_IN : EP0_DATA_OUT;
        } else {
            xfer->active = dir == TUSB_DIR_IN ? EP0_STATUS_IN : EP0_STATUS_OUT;
        }
    } else {
        xfer_begin(xfer, buffer, total_bytes, dir);
    }

    if (epnum == 0 && xfer->active == EP0_DATA_OUT &&
        ep0_prior == EP0_DATA_OUT_EARLY) {
        /* The data packet already arrived and was staged by the ISR. Deliver
         * it and finish the phase now; no hardware arming is involved. The
         * completion event is published with the current epoch, so the normal
         * task path retires it. */
        uint16_t const copy = tu_min16(_ep0_stage_len, total_bytes);
        for (uint16_t i = 0; i < copy; i++) buffer[i] = _ep0_stage[i];
        xfer->actual_len = copy;
        _ep0_stage_len = 0;
        /* Queue full (the task is its own consumer, so effectively
         * unreachable) leaves the transaction to die here; the host's timeout
         * retry issues a fresh SETUP which resets this state cleanly. */
        if (ep0_event_publish_task(rhport, 0, copy, xfer))
            xfer_complete(0, xfer);
        udp_irq_enable_if_ready();
        return true;
    }
    if (epnum == 0 && xfer->active == EP0_STATUS_OUT &&
        ep0_prior == EP0_STATUS_OUT_EARLY) {
        /* The status ZLP already completed on the wire. */
        if (ep0_event_publish_task(rhport, 0, 0, xfer))
            xfer_complete(0, xfer);
        udp_irq_enable_if_ready();
        return true;
    }

    if (ep0_pending) {
        /* Fallback hold only (staging occupied): the packet is still in FDR.
         * With the descriptor now active, exposing its level interrupt makes
         * the normal OUT path copy and release it. */
        ep0_irq_enable();
    }

    if (dir == TUSB_DIR_OUT) {
        if (epnum != 0) endpoint_irq_enable(epnum);
        else            udp_irq_enable_if_ready();
    } else {
        /* Gate the UDP interrupt during FDR writes to prevent the ISR
         * from reading a different endpoint's FDR concurrently — the
         * shared DPRAM bus causes byte leakage between FIFOs. */
        if (epnum != 0) AT91C_BASE_AIC->AIC_IDCR = (1u << AT91C_ID_UDP);
        xact_ep_write(epnum, xfer->buffer, xfer_packet_len(xfer));
        csr_set(epnum, AT91C_UDP_TXPKTRDY);
        udp_irq_enable_if_ready();
    }
    return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    if (epnum == 0 && _ep0_task_epoch != _ep0_setup_epoch) return;
    /* usbd pairs EP0 IN+OUT stalls; handle only one side to match SAMG. */
    if (ep_addr == tu_edpt_addr(0, TUSB_DIR_IN_MASK)) return;
    /* Drop queued bulk-OUT packets before publishing a stall so data from the
     * rejected phase cannot be consumed as the next CBW. */
    if (epnum != 0 && tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
        endpoint_irq_disable(epnum);
        endpoint_fifo_reset(epnum);
    }
    csr_set(epnum, AT91C_UDP_FORCESTALL);

    if (epnum == 0) {
        /* A completed control OUT bank is deliberately held until the USB task
         * arms the status phase. If the class callback rejects that data,
         * TinyUSB stalls instead of arming status; without this release the
         * bank remains full and EP0 remains interrupt-masked forever, so even
         * the next SETUP cannot clear the stall. FORCESTALL is already
         * published, therefore releasing the old data bank is safe: the host's
         * status token receives STALL, and a following SETUP can interrupt and
         * start a clean retry. */
        csr_clear(0, AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1);
        ep0_irq_enable();
        udp_irq_enable_if_ready();
    }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    if (epnum == 0 && _ep0_task_epoch != _ep0_setup_epoch) return;
    csr_clear(epnum, AT91C_UDP_FORCESTALL);
    /* Pulse RSTEP to reset the data-toggle back to DATA0. */
    endpoint_fifo_reset(epnum);
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport) {
    /* A full OUT bank is deliberately interrupt-masked while TinyUSB's task
     * consumes its completion and arms the next EP0 phase. UDP_ISR still
     * reports that level while it is masked. Scanning raw ISR bits when an
     * unrelated endpoint interrupts would therefore pop the held bank a
     * second time into a replacement descriptor. Only service sources that
     * are enabled in IMR; dcd_edpt_xfer() re-enables a held endpoint after it
     * has published the descriptor that makes the packet safe to consume. */
    uint32_t const intr_status = UDP->UDP_ISR & UDP->UDP_IMR;

    uint8_t system_events = 0;
    if (intr_status & AT91C_UDP_ENDBUSRES) system_events++;
    if (intr_status & AT91C_UDP_RXSUSP)    system_events++;
    if (intr_status & AT91C_UDP_RXRSM)     system_events++;
    if (intr_status & AT91C_UDP_WAKEUP)    system_events++;
    if (system_events && event_queue_defer(system_events)) return;

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

    /* Per datasheet, DIR must be configured while RXSETUP is still asserted
     * and before it is cleared, so csr_set(DIR) must be read-back-synchronised
     * before csr_clear(RXSETUP) runs. */
    if ((intr_status & AT91C_UDP_EPINT0) &&
        (UDP->UDP_CSR[0] & AT91C_UDP_RXSETUP)) {
        xfer_desc_t *prior = &_dcd_xfer[0];
        uint8_t setup_events = 1; /* the SETUP itself */
        switch (prior->active) {
            case EP0_DATA_IN:
                setup_events += 2;
                break;
            case EP0_DATA_IN_DONE:
            case EP0_STATUS_OUT:
            case EP0_PENDING_STATUS_OUT:
            case EP0_STATUS_OUT_EARLY:
            case EP0_STATUS_IN:
                setup_events++;
                break;
            default:
                break;
        }
        /* RXSETUP remains asserted, NAK-gating the ensuing data phase, until
         * the prior transaction can be retired and this SETUP can be queued
         * atomically. Prior events carry the prior epoch, so callbacks can
         * advance TinyUSB without arming hardware phases over this SETUP. */
        if (event_queue_defer(setup_events)) return;

        uint8_t setup[8];
        dpr_sync_delay();
        for (uint8_t i = 0; i < 8; i++) setup[i] = (uint8_t)UDP->UDP_FDR[0];

        /* A new SETUP proves that the host finished (or deliberately aborted)
         * the preceding control transfer. If its final interrupt has not been
         * published yet, synthesize only the phases implied by that SETUP.
         * They are tagged with prior's epoch before the descriptor is reused. */
        switch (prior->active) {
            case EP0_DATA_IN:
                ep0_event_xfer_complete(rhport, TUSB_DIR_IN_MASK,
                                        prior->total_len, prior);
                ep0_event_xfer_complete(rhport, 0, 0, prior);
                break;
            case EP0_DATA_IN_DONE:
            case EP0_STATUS_OUT:
            case EP0_PENDING_STATUS_OUT:
            case EP0_STATUS_OUT_EARLY:
                ep0_event_xfer_complete(rhport, 0, 0, prior);
                break;
            case EP0_STATUS_IN:
                ep0_event_xfer_complete(rhport, TUSB_DIR_IN_MASK, 0, prior);
                break;
            default:
                break;
        }

        /* A new SETUP aborts any remaining transfer. Its FIFO contents
         * supersede a bank deliberately held between OUT packets, and any
         * staged-but-undelivered early data belongs to the dead transaction. */
        _ep0_setup_epoch = (uint8_t)((_ep0_setup_epoch + 1u) & EP0_EPOCH_MASK);
        _ep0_stage_len = 0;
        xfer_end(prior);
        prior->active = EP0_WAIT;
        ep0_context_set(prior, tu_edpt_dir(setup[0]));
        ep0_irq_enable();

        dcd_event_setup_received(rhport, setup, true);

        /* Retire every bit of the preceding transaction while RXSETUP still
         * NAK-gates the new data phase. Do not combine any of these clears with
         * releasing RXSETUP: CSR writes cross asynchronously into the UDP
         * clock domain. */
        uint32_t stale = UDP->UDP_CSR[0] &
            (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1);
        stale |= AT91C_UDP_TXPKTRDY | AT91C_UDP_TXCOMP |
                 AT91C_UDP_STALLSENT | AT91C_UDP_FORCESTALL;
        csr_clear(0, stale);

        /* Program DIR for every SETUP while RXSETUP is still asserted, as
         * required by the SAM7S UDP and TinyUSB's reference SAMG driver. In
         * particular, an IN control request (the mux OPEN) leaves DIR set;
         * failing to clear it for the following OUT request makes the FDR
         * expose the old IN bank. */
        if (tu_edpt_dir(setup[0])) csr_set(0, AT91C_UDP_DIR);
        else                       csr_clear(0, AT91C_UDP_DIR);

        csr_clear(0, AT91C_UDP_RXSETUP);
    }

    /* Process OUT (FDR reads) and IN (FDR writes) in separate passes.
     * The AT91SAM7S UDP's DPRAM bus exhibits byte leakage between
     * endpoint FIFOs when reads and writes to different FDR[] indices
     * occur back-to-back without settling time. */

    /* Pass 1: OUT endpoints — read from FDRs */
    for (uint8_t epnum = 0; epnum < EP_COUNT; epnum++) {
        if (!(intr_status & (1u << epnum))) continue;
        uint32_t csr = UDP->UDP_CSR[epnum];
        xfer_desc_t* xfer = &_dcd_xfer[epnum];

        for (;;) {
            uint32_t const banks_complete =
                csr & (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1);
            /* EP0 is single-bank.  For every other endpoint, consume only the
             * bank software expects next.  Clearing both flags at once loses a
             * 64-byte packet whenever a ping-pong pair fills before the ISR is
             * serviced (MSC then waits forever for the discarded bytes). */
            uint32_t const bank_complete = epnum == 0
                                         ? banks_complete
                                         : banks_complete & out_bank_expected(epnum);
            if (!bank_complete) break;

            uint16_t const xact_len =
                (uint16_t)((csr & AT91C_UDP_RXBYTECNT) >> 16);

            if (epnum == 0 && !ep0_out_active(xfer->active)) {
                /* OUT/status can beat the queued setup/completion event to the
                 * TinyUSB task. If it is the early status phase of an IN
                 * transfer, first finish the IN transaction so TinyUSB will
                 * arm that status descriptor. */
                if (xfer->active == EP0_DATA_IN && xact_len == 0) {
                    if (event_queue_defer(1)) return;
                    ep0_event_xfer_complete(rhport,
                                            epnum | TUSB_DIR_IN_MASK,
                                            xfer->total_len, xfer);
                    xfer_complete(epnum, xfer);
                }
                /* Consume the early packet now and release the bank. Keeping
                 * the bank full with EP0 masked NAKs further data, but the UDP
                 * still ACKs a new SETUP from another host process - and a
                 * masked EP0 hides that SETUP until the 10 ms safety poll,
                 * which xHCI does not tolerate. A held status ZLP carries
                 * nothing; a held data packet fits the fixed staging buffer.
                 * dcd_edpt_xfer() finishes the phase when the task arms it. */
                if (xfer_dir(xfer) == TUSB_DIR_IN) {
                    xfer->active = EP0_STATUS_OUT_EARLY;
                    csr_clear(epnum, bank_complete);
                } else if (xfer->active != EP0_DATA_OUT_EARLY &&
                           xact_len <= sizeof(_ep0_stage)) {
                    if (xact_len)
                        xact_ep_read(epnum, _ep0_stage, xact_len, xact_len);
                    _ep0_stage_len = (uint8_t)xact_len;
                    xfer->active = EP0_DATA_OUT_EARLY;
                    csr_clear(epnum, bank_complete);
                } else {
                    /* Staging already occupied (multi-packet control OUT that
                     * outran the task) - fall back to the bounded hold. */
                    xfer->active = EP0_PENDING_DATA_OUT;
                    ep0_irq_disable();
                }
                break;
            }

            /* RXBYTECNT describes the hardware bank, not necessarily the
             * descriptor TinyUSB armed. A stale full EP0 bank has occasionally
             * appeared while a seven-byte short transfer was active. Never let
             * that overrun _usbd_ctrl_buf or make TinyUSB's remaining-length
             * subtraction wrap; drain the whole FIFO but copy/report only the
             * bytes that fit the active descriptor. */
            uint16_t const remaining = xfer->actual_len < xfer->total_len
                                     ? xfer->total_len - xfer->actual_len : 0;
            uint16_t const copy_len = tu_min16(xact_len, remaining);
            bool const completes = xact_len < xfer->epsize ||
                                   (uint16_t)(xfer->actual_len + copy_len) >= xfer->total_len;
            if (completes && event_queue_defer(1)) return;
            if (xact_len) {
                xact_ep_read(epnum, xfer->buffer, xact_len, copy_len);
                xfer->buffer += copy_len;
            }
            xfer->actual_len += copy_len;

            if (completes) {
                if (epnum != 0) endpoint_irq_disable(epnum);
                if (epnum == 0) {
                    ep0_event_xfer_complete(rhport, epnum,
                                            xfer->actual_len, xfer);
                } else {
                    dcd_event_xfer_complete(rhport, epnum, xfer->actual_len,
                                            XFER_RESULT_SUCCESS, true);
                }
                /* No flow-control hold here: a completed v2 data stage is a
                 * single packet, the next token is the status IN (which needs
                 * no bank), and holding the bank with EP0 masked would hide a
                 * superseding SETUP from another process. */
                xfer_complete(epnum, xfer);
            }
            csr_clear(epnum, bank_complete);
            if (epnum != 0) out_bank_advance(epnum);

            if (completes) break;
            csr = UDP->UDP_CSR[epnum];
        }

        csr = UDP->UDP_CSR[epnum];
        if (csr & AT91C_UDP_STALLSENT) {
            /* A control-endpoint stall rejects only the current request.  The
             * SAM7S does not clear FORCESTALL for us after sending it; leaving
             * that bit set makes every following SETUP fail with EPIPE and the
             * endpoint cannot recover until a much later bus event happens to
             * disturb the CSR.  Stock Proxmark3's AT91F_USB_SendStall() clears
             * FORCESTALL together with STALLSENT for exactly this reason.
             * Non-control endpoint halts remain latched until CLEAR_FEATURE. */
            uint32_t clear = AT91C_UDP_STALLSENT;
            if (epnum == 0) clear |= AT91C_UDP_FORCESTALL;
            csr_clear(epnum, clear);
        }
    }

    /* DPRAM settling — a read of a non-FDR register forces the bus to
     * complete any pending FIFO access before we switch direction. */
    (void)UDP->UDP_NUM;

    /* Pass 2: IN endpoints — write to FDRs */
    for (uint8_t epnum = 0; epnum < EP_COUNT; epnum++) {
        if (!(intr_status & (1u << epnum))) continue;
        uint32_t const csr = UDP->UDP_CSR[epnum];
        xfer_desc_t* xfer = &_dcd_xfer[epnum];

        if (csr & AT91C_UDP_TXCOMP) {
            /* EP0 is bidirectional and uses one descriptor. A completed IN can
             * remain pending until after the next OUT has been armed; never let
             * that stale flag finish or advance the replacement descriptor. */
            if (epnum == 0 && !ep0_in_active(xfer->active)) {
                csr_clear(epnum, AT91C_UDP_TXCOMP);
                continue;
            }
            uint16_t const completed_packet_len = xfer_packet_len(xfer);
            bool const completes =
                (uint16_t)(xfer->actual_len + completed_packet_len) >= xfer->total_len;
            if (completes && event_queue_defer(1)) return;
            xfer_packet_done(xfer);
            uint16_t const xact_len = xfer_packet_len(xfer);

            if (xact_len) {
                xact_ep_write(epnum, xfer->buffer, xact_len);
                csr_set(epnum, AT91C_UDP_TXPKTRDY);
            } else {
                if (epnum == 0) {
                    ep0_event_xfer_complete(rhport,
                                            epnum | TUSB_DIR_IN_MASK,
                                            xfer->actual_len, xfer);
                } else {
                    dcd_event_xfer_complete(rhport,
                                            epnum | TUSB_DIR_IN_MASK,
                                            xfer->actual_len,
                                            XFER_RESULT_SUCCESS, true);
                }
                xfer_complete(epnum, xfer);
            }
            csr_clear(epnum, AT91C_UDP_TXCOMP);
        }
    }
}

#endif /* CFG_TUSB_MCU == OPT_MCU_AT91SAM7S */
