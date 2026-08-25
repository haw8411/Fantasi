#include "usb_transport.h"
#include "usb_mux.h"

#include <libusb-1.0/libusb.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define FANTASI_VID 0x1209
#define FANTASI_PID 0x0001

static libusb_context       *s_ctx;
static libusb_device_handle *s_dev;
static int      s_itf   = -1;
static uint8_t  s_ep_in;
static uint8_t  s_ep_out;
static bool     s_mux;
static uint32_t s_session;
/* Offset within the current response frame's device mailbox. Mux READs are
 * stateless on the device: the host addresses the byte it wants via wIndex, so
 * a retried/aborted READ can never duplicate or skip response bytes. Reset to 0
 * at the start of each response frame; advanced by bytes actually received. */
static uint16_t s_read_offset;
static uint8_t  s_ep0_size;
static uint8_t  s_mux_version;
static uint8_t  s_frame_token;
static uint64_t s_next_ping_ms;
static char     s_want_name[64]; /* restrict to this device name (iSerial), for multi-device setups (empty = any) */

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* Randomize control-transfer retries so independent CLI processes do not
 * remain synchronized under EP0 contention. */
static unsigned s_backoff_seed;
static void mux_backoff(unsigned attempt)
{
    (void)attempt;
    /* getpid() differentiates retries before OPEN assigns a session ID. */
    if (!s_backoff_seed)
        s_backoff_seed = ((unsigned)getpid() * 2654435761u) | 1u;
    /* Short and desynchronized: keep retries fast (a long delay only widens the
     * contention window and adds EP0 load) while the per-process jitter breaks
     * two clients out of lockstep. */
    unsigned jitter = rand_r(&s_backoff_seed) % 1200u;      /* 0..1.2 ms */
    usleep(300u + jitter);
}

/* Idle WebUSB clients only need to renew the firmware's 15-second lease. Keep
 * their EP0 heartbeats sparse and spread them by the device-issued session ID;
 * otherwise many CLI processes opened together all probe at the same 500 ms
 * readline tick and needlessly burst the small ARM7 control-event queue. */
static void defer_mux_ping(void)
{
    uint64_t const now = monotonic_ms();
    /* 4-6 s, staggered by session id. Comfortably under the device's 15 s
     * lease so a lost heartbeat has several more tries before the session
     * could be reaped, while spreading many CLIs' probes so they don't all
     * burst the small ARM7 control-event queue on the same readline tick. */
    s_next_ping_ms = now ? now + 4000u + (s_session % 2001u) : 0;
}

static uint8_t mux_crc8_update(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (unsigned bit = 0; bit < 8; bit++) {
        uint8_t next = (uint8_t)(crc << 1);
        crc = (crc & 0x80u) ? (uint8_t)(next ^ 0x07u) : next;
    }
    return crc;
}

static uint8_t mux_chunk_crc(uint32_t session, const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (unsigned shift = 0; shift < 32; shift += 8)
        crc = mux_crc8_update(crc, (uint8_t)(session >> shift));
    for (size_t i = 0; i < len; i++) crc = mux_crc8_update(crc, data[i]);
    return crc;
}

static uint16_t mux_frame_crc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xffffu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; bit++) {
            uint16_t next = (uint16_t)(crc << 1);
            crc = (crc & 0x8000u) ? (uint16_t)(next ^ 0x1021u) : next;
        }
    }
    return crc;
}

/* Send one EP0-sized, checksummed control transfer at a time. A failed packet
 * aborts this attempt; the caller restarts from START so device reassembly can
 * never retain an ambiguous prefix. */
static int mux_chunk_write_once(const uint8_t *data, size_t len,
                                uint8_t frame_token)
{
    if (len > FANTASI_USB_MUX_CHUNK_START_DATA_MAX +
              255u * FANTASI_USB_MUX_CHUNK_DATA_MAX)
        return LIBUSB_ERROR_INVALID_PARAM;

    uint16_t const frame_crc = mux_frame_crc(data, len);
    size_t offset = 0;
    uint8_t sequence = 0;
    do {
        uint8_t packet[FANTASI_USB_MUX_CHUNK_MAX];
        size_t take;
        size_t packet_len;
        if (sequence == 0) {
            take = 0;
            packet[0] = FANTASI_USB_MUX_CHUNK_START | (uint8_t)take;
            packet[1] = frame_token;
            packet[2] = (uint8_t)len;
            packet[3] = (uint8_t)(len >> 8);
            packet[4] = (uint8_t)frame_crc;
            packet[5] = (uint8_t)(frame_crc >> 8);
            if (take) packet[6] = data[0];
            packet_len = 7u + take;
        } else {
            size_t remain = len - offset;
            take = remain < FANTASI_USB_MUX_CHUNK_DATA_MAX
                 ? remain : FANTASI_USB_MUX_CHUNK_DATA_MAX;
            packet[0] = (uint8_t)take;
            packet[1] = sequence;
            if (take) memcpy(packet + 2, data + offset, take);
            packet_len = 3u + take;
        }
        if (offset + take == len) packet[0] |= FANTASI_USB_MUX_CHUNK_END;
        packet[packet_len - 1] = mux_chunk_crc(s_session, packet, packet_len - 1);

        /* Retry this single chunk before giving up on the frame. The device
         * treats a resent previous-sequence chunk as an idempotent no-op, so a
         * lost packet costs one chunk retransmit instead of restarting the
         * whole (~120-chunk) frame from START - essential for concurrent
         * uploads to make progress on the contended 8-byte EP0. */
        int r = LIBUSB_ERROR_IO;
        for (unsigned catt = 0; catt < 8; catt++) {
            r = libusb_control_transfer(
                s_dev, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                       LIBUSB_RECIPIENT_DEVICE,
                FANTASI_USB_MUX_CHUNK,
                (uint16_t)s_session, 0,
                packet, (uint16_t)packet_len, 1000);
            if (r == (int)packet_len) break;
            if (r == LIBUSB_ERROR_NO_DEVICE) return r;
            mux_backoff(catt);
        }
        if (r != (int)packet_len) return r < 0 ? r : LIBUSB_ERROR_IO;
        offset += take;
        sequence++;
    } while (offset < len);
    return 0;
}

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
        s_ep0_size = dd.bMaxPacketSize0;
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

        /* The force-bulk switch is intentionally undocumented and exists for
         * controller diagnostics: bulk remains usable when a broken EP0 state
         * prevents us from reading the device's retained trace. */
        if (!getenv("FANTASI_USB_FORCE_BULK")) {
        /* Multiplexed firmware allocates a device-owned session over EP0.
         * Device control requests do not require claiming the vendor interface,
         * so every CLI process can do this independently. A PIPE/short response
         * selects the legacy exclusive bulk fallback. */
        unsigned char reply[FANTASI_USB_MUX_OPEN_REPLY_SIZE];
        /* Per-connection OPEN idempotence nonce, carried in wValue/wIndex (there
         * is no session id yet at OPEN). A retried OPEN with the same nonce
         * returns the same device session instead of orphaning one when an OPEN
         * reply is lost after the device already created the session. */
        if (!s_backoff_seed)
            s_backoff_seed = ((unsigned)getpid() * 2654435761u) | 1u;
        uint32_t const open_nonce =
            ((uint32_t)rand_r(&s_backoff_seed) << 17) ^
            ((uint32_t)rand_r(&s_backoff_seed) << 1) ^ (uint32_t)getpid() ^ 1u;
        uint16_t const nonce_lo = (uint16_t)open_nonce;
        uint16_t const nonce_hi = (uint16_t)(open_nonce >> 16);
        /* This one mux protocol is used on every target. Small-EP0 targets
         * additionally use CHUNK writes, while normal EP0 targets retain
         * whole-frame WRITE requests. Session addressing and stateless READs
         * are identical in both cases. */
        int opened = LIBUSB_ERROR_NOT_SUPPORTED;
        for (unsigned attempt = 0; attempt < 8; attempt++) {
            opened = libusb_control_transfer(
                s_dev,
                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                FANTASI_USB_MUX_OPEN, nonce_lo, nonce_hi, reply,
                FANTASI_USB_MUX_OPEN_REPLY_SIZE, 1000);
            if (opened == FANTASI_USB_MUX_OPEN_REPLY_SIZE &&
                reply[0] == FANTASI_USB_MUX_MAGIC_0 &&
                reply[1] == FANTASI_USB_MUX_MAGIC_1 &&
                reply[2] == FANTASI_USB_MUX_VERSION) {
                s_session = (uint32_t)reply[3] | ((uint32_t)reply[4] << 8) |
                            ((uint32_t)reply[5] << 16) | ((uint32_t)reply[6] << 24);
                if (s_session) {
                    s_mux = true;
                    s_mux_version = FANTASI_USB_MUX_VERSION;
                    s_frame_token = 0;
                    defer_mux_ping();
                    rc = 0;
                    break;
                }
            }
            if (opened == LIBUSB_ERROR_NO_DEVICE) break;
            mux_backoff(attempt);
        }
        if (s_mux) break;

        /* Firmware predating multiplexed OPEN takes the established exclusive
         * bulk fallback below. */
        }
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
    if (rc == 0 && !s_mux) {
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
        if (s_mux && s_session) {
            /* Retry CLOSE: a single failed CLOSE transfer under EP0 contention
             * would leave the device session resident until its (disabled)
             * lease, showing up as "closed sessions remained". CLOSE is
             * idempotent on the device (a second CLOSE for an already-closing
             * peer is a no-op), so retrying is safe. */
            for (unsigned attempt = 0; attempt < 12; attempt++) {
                int r = libusb_control_transfer(
                    s_dev, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                           LIBUSB_RECIPIENT_DEVICE,
                    FANTASI_USB_MUX_CLOSE,
                    (uint16_t)s_session, 0,
                    NULL, 0, 500);
                if (r >= 0 || r == LIBUSB_ERROR_NO_DEVICE) break;
                mux_backoff(attempt);
            }
        } else if (s_itf >= 0) {
            libusb_release_interface(s_dev, s_itf);
        }
        libusb_close(s_dev);
        s_dev = NULL;
    }
    if (s_ctx) { libusb_exit(s_ctx); s_ctx = NULL; }
    s_itf = -1;
    s_mux = false;
    s_session = 0;
    s_ep0_size = 0;
    s_mux_version = 0;
    s_frame_token = 0;
    s_next_ping_ms = 0;
}

bool usb_transport_connected(void) { return s_dev != NULL; }
uint32_t usb_transport_session_id(void) { return s_mux ? s_session : 0; }
bool usb_transport_multiplexed(void) { return s_mux; }

/* True when the connected device has the tiny control endpoint that forces the
 * chunked mux path (the SAM7S PM3's 8-byte EP0). It is exactly this device whose
 * dual-bank OUT overruns under pipelined uploads, so it identifies a device that
 * must be paced to one in-flight chunk - a hardware fact read from the device
 * descriptor, independent of which connection route reached it. */
bool usb_transport_constrained_ep0(void)
{
    return s_ep0_size != 0 && s_ep0_size <= FANTASI_USB_MUX_EP0_SIZE_MAX;
}

/* Device is gone: close the handle so usb_transport_connected() reads false
 * and a later usb_transport_open() rescans. Keep s_ctx so reopen is cheap. */
static void usb_transport_drop(void)
{
    if (s_dev) { libusb_close(s_dev); s_dev = NULL; }
    s_itf = -1;
    s_mux = false;
    s_session = 0;
    s_ep0_size = 0;
    s_mux_version = 0;
    s_frame_token = 0;
    s_next_ping_ms = 0;
}

/* Probe the device-owned session. An empty mailbox returns a successful
 * zero-length READ, while a vanished session stalls both READ and PING. */
static bool mux_ping_now(unsigned attempts)
{
    unsigned char magic[4];
    for (unsigned attempt = 0; attempt < attempts; attempt++) {
        int r = libusb_control_transfer(
            s_dev, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                   LIBUSB_RECIPIENT_DEVICE,
            FANTASI_USB_MUX_PING,
            (uint16_t)s_session, 0,
            magic, sizeof(magic), 500);
        if (r == 4 && magic[0] == FANTASI_USB_MUX_MAGIC_0 &&
            magic[1] == FANTASI_USB_MUX_MAGIC_1 &&
            magic[2] == FANTASI_USB_MUX_MAGIC_2 &&
            magic[3] == s_mux_version) {
            defer_mux_ping();
            return true;
        }
        if (r == LIBUSB_ERROR_NO_DEVICE) break;
        mux_backoff(attempt);
    }
    return false;
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
    if (s_mux) {
        uint64_t const now = monotonic_ms();
        if (now && s_next_ping_ms && now < s_next_ping_ms) return true;
        /* Retry the lease renewal: a single PING lost to EP0 contention must
         * not let the device reap this (idle-holder) session before its next
         * heartbeat. The device renews `touched` on any PING that lands. */
        if (!mux_ping_now(12)) {
            /* The device is still present but this lease is not. Drop the old
             * handle so the reconnect path performs a fresh OPEN instead of
             * repeatedly reusing an expired session. */
            usb_transport_drop();
            return false;
        }
    }
    return true;
}

ssize_t usb_transport_read(void *buf, size_t len)
{
    if (!s_dev) return -1;
    if (s_mux) {
        size_t request_len = len > UINT16_MAX ? UINT16_MAX : len;
        if (s_ep0_size <= FANTASI_USB_MUX_EP0_SIZE_MAX &&
            s_mux_version >= FANTASI_USB_MUX_VERSION &&
            request_len > FANTASI_USB_MUX_CHUNK_MAX)
            request_len = FANTASI_USB_MUX_CHUNK_MAX;
        int r = LIBUSB_ERROR_OTHER;
        for (unsigned attempt = 0; attempt < 16; attempt++) {
            r = libusb_control_transfer(
                s_dev, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                       LIBUSB_RECIPIENT_DEVICE,
                FANTASI_USB_MUX_READ,
                (uint16_t)s_session, s_read_offset,
                buf, (uint16_t)request_len, 500);
            if (r >= 0 || r == LIBUSB_ERROR_NO_DEVICE) break;
            /* A short-lived EP0 stall is a rejected transaction, not a lost
             * session. The stateless offset read is idempotent, so retry with a
             * desynchronized backoff (see mux_backoff). Timeouts are retried
             * too: an empty mailbox is a successful zero-length transfer, so a
             * negative timeout never means "no data". */
            mux_backoff(attempt);
        }
        if (r == LIBUSB_ERROR_NO_DEVICE) { usb_transport_drop(); return -1; }
        if (r > 0) { s_read_offset = (uint16_t)(s_read_offset + r); return r; }
        if (r < 0 && !mux_ping_now(12)) {
            /* All READ retries plus an explicit PING rejected this SID.  It is
             * not an empty mailbox (which is a successful zero-byte READ), so
             * stop the command instead of spinning forever on EPIPE after a
             * device-side lease loss. */
            usb_transport_drop();
            return -1;
        }
        /* A transient read error/timeout is not a lost response: the offset
         * read is idempotent, so report "no data yet" and let the caller's
         * poll loop retry from the same offset. Returning a hard error here
         * would abandon a response that is still readable, failing the command
         * on a momentary EP0 stall under contention. Only device-gone (above)
         * is fatal. */
        return 0;
    }
    int got = 0;
    int r = libusb_bulk_transfer(s_dev, s_ep_in, buf, (int)len, &got, 50);
    if (r == 0 || r == LIBUSB_ERROR_TIMEOUT) return got;
    if (r == LIBUSB_ERROR_NO_DEVICE) { usb_transport_drop(); return -1; }
    return got;   /* other transient errors: report what we got (0) */
}

/* Rewind the per-frame read offset without any I/O (used before a fresh
 * request, where the prior response was already drained/released). */
void usb_transport_read_reset(void) { s_read_offset = 0; }

/* Acknowledge a fully-received response frame: a stateless past-end READ tells
 * the device the host has the whole frame, releasing its mailbox so the worker
 * can emit the next streamed frame. Resets the per-frame read offset. No-op on
 * the legacy bulk path. Called by the framing layer after each decoded frame. */
void usb_transport_frame_consumed(void)
{
    if (s_dev && s_mux && s_session &&
        s_mux_version >= FANTASI_USB_MUX_VERSION) {
        unsigned char scratch[FANTASI_USB_MUX_CHUNK_MAX];
        for (unsigned attempt = 0; attempt < 10; attempt++) {
            int r = libusb_control_transfer(
                s_dev, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                       LIBUSB_RECIPIENT_DEVICE,
                FANTASI_USB_MUX_READ,
                (uint16_t)s_session, s_read_offset,
                scratch, (uint16_t)sizeof(scratch), 500);
            if (r >= 0 || r == LIBUSB_ERROR_NO_DEVICE) break;
            usleep(2000);
        }
    }
    s_read_offset = 0;
}

ssize_t usb_transport_write(const void *buf, size_t len)
{
    if (!s_dev) return -1;
    if (s_mux) {
        if (len > UINT16_MAX) return -1;
        if (s_ep0_size <= FANTASI_USB_MUX_EP0_SIZE_MAX &&
            s_mux_version >= FANTASI_USB_MUX_VERSION) {
            int r = LIBUSB_ERROR_OTHER;
            uint8_t const frame_token = ++s_frame_token;
            /* START retries with the same token are idempotent. Randomized
             * retries allow concurrent uploaders to share EP0. */
            for (unsigned attempt = 0; attempt < 14; attempt++) {
                r = mux_chunk_write_once(buf, len, frame_token);
                if (r == 0) return (ssize_t)len;
                if (r == LIBUSB_ERROR_NO_DEVICE) {
                    usb_transport_drop();
                    return -1;
                }
                mux_backoff(attempt);
            }
            return -1;
        }
        int r = libusb_control_transfer(
            s_dev, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                   LIBUSB_RECIPIENT_DEVICE,
            FANTASI_USB_MUX_WRITE,
            (uint16_t)s_session, 0,
            (unsigned char *)(uintptr_t)buf, (uint16_t)len, 1000);
        if (r == LIBUSB_ERROR_NO_DEVICE) { usb_transport_drop(); return -1; }
        return r < 0 ? -1 : r;
    }
    int sent = 0;
    int r = libusb_bulk_transfer(s_dev, s_ep_out, (unsigned char *)buf, (int)len, &sent, 1000);
    if (r == LIBUSB_ERROR_NO_DEVICE) { usb_transport_drop(); return -1; }
    if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) return -1;
    return sent;
}
