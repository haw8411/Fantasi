/* WebUSB protobuf transports.
 *
 * The legacy vendor bulk pipe is one implicit, ordered session. New clients
 * use control transfers to create and address independent sessions. */
#include "usb_proto.h"

#include "log.h"
#include "cli.h"
#include "proto.h"
#include "../proto/usb_mux.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

typedef struct mux_peer {
    struct mux_peer *next;
    uint8_t *mail;                /* owned by the emitting proto task */
    uint32_t session;
    uint32_t nonce;              /* host-chosen OPEN idempotence key */
    uint16_t mail_len;
    uint8_t version;
    uint8_t last_token;
    uint8_t flags;
    uint8_t refs;
} mux_peer_t;

enum {
    MUX_PEER_HAS_LAST_TOKEN = 1u << 0,
    MUX_PEER_CLOSING        = 1u << 1,
};

_Static_assert(sizeof(mux_peer_t) <= 24,
               "WebUSB mailbox metadata grew beyond its RAM budget");

static mux_peer_t *s_peers;
static SemaphoreHandle_t s_peer_lock;

/* TinyUSB serializes EP0 transfers. These values describe the currently
 * staged mux request until CONTROL_STAGE_ACK. */
typedef enum {
    CTRL_NONE,
    CTRL_OPEN,
    CTRL_CLOSE,
    CTRL_WRITE,
    CTRL_CHUNK,
    CTRL_READ,
    CTRL_PING,
} control_kind_t;

static control_kind_t s_control_kind;
static mux_peer_t *s_control_peer;
static uint16_t s_control_read_len;
static uint8_t s_control_reply[FANTASI_USB_MUX_OPEN_REPLY_SIZE];
#if CFG_TUD_ENDPOINT0_SIZE <= FANTASI_USB_MUX_EP0_SIZE_MAX
static uint8_t s_control_write[FANTASI_USB_MUX_CHUNK_MAX];
#else
static uint8_t s_control_write[FANTASI_PROTO_FRAME_MAX];
#endif

static const fantasi_proto_transport_t s_mux_transport;
static mux_peer_t *peer_find_locked(uint32_t session);

static void control_reset(void)
{
    s_control_kind = CTRL_NONE;
    s_control_peer = NULL;
    s_control_read_len = 0;
}

static void peer_take(void)
{
    if (s_peer_lock) xSemaphoreTake(s_peer_lock, portMAX_DELAY);
}

static void peer_give(void)
{
    if (s_peer_lock) xSemaphoreGive(s_peer_lock);
}

static bool peer_init(void)
{
    if (!s_peer_lock) s_peer_lock = xSemaphoreCreateMutex();
    return s_peer_lock != NULL;
}

static bool peer_closing(const mux_peer_t *peer)
{
    return (peer->flags & MUX_PEER_CLOSING) != 0;
}

/* Called with s_peer_lock held. The registry owns one reference; an emitter
 * temporarily owns another while it waits for the host to consume its frame.
 * This lets CLOSE wake that emitter without freeing the mailbox underneath it. */
static bool peer_put_locked(mux_peer_t *peer)
{
    configASSERT(peer->refs > 0);
    return --peer->refs == 0;
}

#if CFG_TUD_ENDPOINT0_SIZE <= FANTASI_USB_MUX_EP0_SIZE_MAX
typedef struct mux_partial {
    struct mux_partial *next;
    uint32_t session;
    uint16_t total;
    uint16_t received;
    uint16_t expected_crc;
    uint8_t next_sequence;
    uint8_t token;
    uint8_t frame[];
} mux_partial_t;

_Static_assert(sizeof(mux_partial_t) <= 16,
               "WebUSB active reassembly metadata grew beyond its RAM budget");

static mux_partial_t *s_partials;

static mux_partial_t **partial_slot_locked(uint32_t session)
{
    mux_partial_t **slot = &s_partials;
    while (*slot && (*slot)->session != session) slot = &(*slot)->next;
    return slot;
}

static mux_partial_t *partial_take_locked(uint32_t session)
{
    mux_partial_t **slot = partial_slot_locked(session);
    mux_partial_t *part = *slot;
    if (part) *slot = part->next;
    return part;
}

static void partial_drop(uint32_t session)
{
    peer_take();
    mux_partial_t *part = partial_take_locked(session);
    peer_give();
    if (part) vPortFree(part);
}

static uint8_t chunk_crc8_update(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (unsigned bit = 0; bit < 8; bit++) {
        uint8_t next = (uint8_t)(crc << 1);
        crc = (crc & 0x80u) ? (uint8_t)(next ^ 0x07u) : next;
    }
    return crc;
}

static uint8_t chunk_crc8(uint32_t session, const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (unsigned shift = 0; shift < 32; shift += 8)
        crc = chunk_crc8_update(crc, (uint8_t)(session >> shift));
    for (size_t i = 0; i < len; i++) crc = chunk_crc8_update(crc, data[i]);
    return crc;
}

static uint16_t frame_crc16(const uint8_t *data, size_t len)
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


/* Consume one independently checked EP0 packet. Only an active request owns a
 * reassembly allocation; idle sessions retain their 40+24-byte core/peer
 * footprint. Invalid or missing chunks discard the partial frame so the host
 * can restart unambiguously from sequence zero. */
static bool partial_accept(uint32_t session, const uint8_t *packet, size_t len)
{
    if (len < 3 || len > FANTASI_USB_MUX_CHUNK_MAX) {
        partial_drop(session);
        return false;
    }
    if (chunk_crc8(session, packet, len - 1) != packet[len - 1]) {
        partial_drop(session);
        return false;
    }

    uint8_t const flags = packet[0];
    uint8_t const payload_len = flags & FANTASI_USB_MUX_CHUNK_LEN_MASK;
    bool const start = (flags & FANTASI_USB_MUX_CHUNK_START) != 0;
    bool const end = (flags & FANTASI_USB_MUX_CHUNK_END) != 0;
    if (flags & ~(FANTASI_USB_MUX_CHUNK_START |
                  FANTASI_USB_MUX_CHUNK_END |
                  FANTASI_USB_MUX_CHUNK_LEN_MASK)) {
        partial_drop(session);
        return false;
    }

    mux_partial_t *complete = NULL;
    mux_partial_t *discard = NULL;
    bool valid = true;
    peer_take();
    if (start) {
        if (payload_len > FANTASI_USB_MUX_CHUNK_START_DATA_MAX ||
            len != (size_t)(7u + payload_len)) {
            valid = false;
        } else {
            uint16_t total = (uint16_t)packet[2] | ((uint16_t)packet[3] << 8);
            uint16_t crc = (uint16_t)packet[4] | ((uint16_t)packet[5] << 8);
            if (total < 3 || total > FANTASI_PROTO_FRAME_MAX ||
                payload_len > total || end != (payload_len == total)) {
                valid = false;
            } else {
                discard = partial_take_locked(session);
                mux_partial_t *part = pvPortMalloc(sizeof(*part) + total);
                if (!part) {
                    valid = false;
                } else {
                    part->next = s_partials;
                    part->session = session;
                    part->total = total;
                    part->received = payload_len;
                    part->expected_crc = crc;
                    part->next_sequence = 1;
                    part->token = packet[1];
                    if (payload_len) memcpy(part->frame, packet + 6, payload_len);
                    s_partials = part;
                    if (end) complete = partial_take_locked(session);
                }
            }
        }
    } else {
        mux_partial_t **slot = partial_slot_locked(session);
        mux_partial_t *part = *slot;
        if (part && len == (size_t)(3u + payload_len) &&
            payload_len <= FANTASI_USB_MUX_CHUNK_DATA_MAX &&
            part->next_sequence >= 2 &&
            packet[1] == (uint8_t)(part->next_sequence - 1u)) {
            /* Accept retransmission of the last continuation chunk. Its status
             * stage may be lost after the data is appended; the frame CRC still
             * validates the final assembly. */
            peer_give();
            return true;
        }
        if (!part || packet[1] != part->next_sequence ||
            payload_len > FANTASI_USB_MUX_CHUNK_DATA_MAX ||
            len != (size_t)(3u + payload_len) ||
            payload_len > (uint16_t)(part->total - part->received) ||
            end != (part->received + payload_len == part->total)) {
            valid = false;
            discard = partial_take_locked(session);
        } else {
            if (payload_len)
                memcpy(part->frame + part->received, packet + 2, payload_len);
            part->received += payload_len;
            part->next_sequence++;
            if (end) {
                *slot = part->next;
                complete = part;
            }
        }
    }
    peer_give();
    if (discard) vPortFree(discard);
    if (!valid) {
        if (!discard) partial_drop(session);
        return false;
    }
    if (!complete) return true;

    uint16_t const message_len = (uint16_t)complete->frame[0] |
                                 ((uint16_t)complete->frame[1] << 8);
    bool ok = message_len && message_len <= CliRequest_size &&
              complete->total == (uint16_t)(message_len + 2) &&
              frame_crc16(complete->frame, complete->total) == complete->expected_crc;
    if (ok) {
        bool duplicate = false;
        peer_take();
        mux_peer_t *peer = peer_find_locked(session);
        if (peer)
            duplicate = (peer->flags & MUX_PEER_HAS_LAST_TOKEN) &&
                        peer->last_token == complete->token;
        peer_give();
        if (!duplicate) {
            ok = fantasi_proto_session_submit(session, &s_mux_transport, 0,
                                              complete->frame + 2, message_len);
            if (ok) {
                peer_take();
                peer = peer_find_locked(session);
                if (peer) {
                    peer->last_token = complete->token;
                    peer->flags |= MUX_PEER_HAS_LAST_TOKEN;
                }
                peer_give();
            }
        }
    }
    vPortFree(complete);
    return ok;
}
#else
static void partial_drop(uint32_t session) { (void)session; }
#endif

static mux_peer_t *peer_find_locked(uint32_t session)
{
    for (mux_peer_t *p = s_peers; p; p = p->next)
        if (p->session == session && !peer_closing(p)) return p;
    return NULL;
}

/* Mux addressing: the session id is 16-bit and travels in wValue, leaving
 * wIndex free to carry the READ response offset (see the READ handler). Session
 * ids are capped to 16 bits at allocation (core/proto.c). */
static uint32_t request_session(tusb_control_request_t const *request)
{
    return (uint32_t)request->wValue;
}

static void put_u32le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static void put_magic(uint8_t *out, uint8_t version)
{
    out[0] = FANTASI_USB_MUX_MAGIC_0;
    out[1] = FANTASI_USB_MUX_MAGIC_1;
    out[2] = FANTASI_USB_MUX_MAGIC_2;
    out[3] = version;
}

/* ---- Control-session transport vtable ---------------------------------- */

static size_t mux_emit(uint32_t session, uint32_t link,
                       const uint8_t *frame, size_t len)
{
    (void)link;
    mux_peer_t *mine = NULL;

    /* Wait for the preceding response on this session to be consumed. This is
     * a one-frame rendezvous, not a queue: a stalled host can consume at most
     * one response frame's transient RAM, and it backpressures only its worker. */
    for (;;) {
        peer_take();
        mux_peer_t *p = peer_find_locked(session);
        if (!p || peer_closing(p)) {
            peer_give();
            return 0;
        }
        if (!p->mail) {
            p->mail = (uint8_t *)(uintptr_t)frame;
            p->mail_len = (uint16_t)len;
            p->refs++;                 /* retained until this emitter returns */
            mine = p;
            peer_give();
            break;
        }
        peer_give();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    /* Bound the mailbox wait so an abandoned session cannot retain the worker
     * stack indefinitely. */
    TickType_t const deadline = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
    for (;;) {
        peer_take();
        bool done = mine->mail == NULL;
        bool closed = peer_closing(mine);
        bool expired = !done && (int32_t)(xTaskGetTickCount() - deadline) >= 0;
        if (expired) { mine->mail = NULL; done = true; }
        bool destroy = done && peer_put_locked(mine);
        peer_give();
        if (done) {
            if (destroy) vPortFree(mine);
            return (closed || expired) ? 0 : len;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static bool mux_connected(uint32_t session, uint32_t link)
{
    (void)link;
    peer_take();
    bool connected = peer_find_locked(session) != NULL && tud_mounted();
    peer_give();
    return connected;
}

static void mux_release(uint32_t session, uint32_t link)
{
    (void)link;
    mux_peer_t *released = NULL;
    bool destroy = false;
    peer_take();
    mux_peer_t **pp = &s_peers;
    while (*pp) {
        if ((*pp)->session == session) {
            released = *pp;
            *pp = released->next;
            released->flags |= MUX_PEER_CLOSING;
            released->mail = NULL;       /* wakes an emitter; it owns the frame */
            destroy = peer_put_locked(released); /* registry reference */
            break;
        }
        pp = &(*pp)->next;
    }
    peer_give();
    partial_drop(session);
    if (destroy) vPortFree(released);
}

/* Reassembly and unread mail keep the session active for lease reaping. */
static bool mux_has_pending(uint32_t session, uint32_t link)
{
    (void)link;
    peer_take();
    bool pending = false;
#if CFG_TUD_ENDPOINT0_SIZE <= FANTASI_USB_MUX_EP0_SIZE_MAX
    pending = *partial_slot_locked(session) != NULL;
#endif
    if (!pending) {
        mux_peer_t *p = peer_find_locked(session);
        pending = p && p->mail != NULL;
    }
    peer_give();
    return pending;
}

static const fantasi_proto_transport_t s_mux_transport = {
    .name = "webusb",
    .emit = mux_emit,
    .connected = mux_connected,
    .release = mux_release,
    .has_pending = mux_has_pending,
    .ingress_can_emit = false,
    .allow_envelope_open = false,
    /* The lease exceeds the 4-6 s host heartbeat interval. Workers, queued
     * jobs, reassembly, and unread mail suppress reaping during active work. */
    .lease_ms = FANTASI_USB_MUX_LEASE_MS,
};

/* Idempotent OPEN: if a live peer already exists for this host nonce, return it
 * instead of allocating a new session. This makes OPEN retriable - when an OPEN
 * reply transfer is lost after the device already created the session, the
 * host's retry (same nonce) gets the same SID rather than orphaning the first
 * one. nonce 0 means "no idempotence key" (legacy host); always allocate. */
static mux_peer_t *open_peer(uint8_t version, uint32_t nonce)
{
    if (nonce) {
        peer_take();
        for (mux_peer_t *p = s_peers; p; p = p->next) {
            if (p->nonce == nonce && !peer_closing(p)) {
                uint32_t session = p->session;
                peer_give();
                /* OPEN retries are transport activity too. The lease belongs
                 * to the core session registry; keeping a second peer-local
                 * timestamp made successful PINGs invisible to its reaper. */
                fantasi_proto_session_touch(session, &s_mux_transport, 0);
                return p;         /* same session for a retried OPEN */
            }
        }
        peer_give();
    }
    mux_peer_t *peer = pvPortMalloc(sizeof(*peer));
    if (!peer) return NULL;
    memset(peer, 0, sizeof(*peer));
    peer->version = version;
    peer->nonce = nonce;
    peer->refs = 1;                       /* registry reference */
    peer->session = fantasi_proto_session_open(&s_mux_transport, 0);
    if (!peer->session) { vPortFree(peer); return NULL; }
    peer_take();
    peer->next = s_peers;
    s_peers = peer;
    peer_give();
    return peer;
}

static void close_peer(mux_peer_t *peer)
{
    if (!peer) return;
    uint32_t session;
    peer_take();
    if (peer_closing(peer)) { peer_give(); return; }
    peer->flags |= MUX_PEER_CLOSING;
    peer->mail = NULL;
    session = peer->session;
    peer_give();
    fantasi_proto_session_close(session, &s_mux_transport, 0);
}

void usb_proto_transport_down(void)
{
    if (!peer_init()) return;
    control_reset();
    for (;;) {
        uint32_t session = 0;
        peer_take();
        for (mux_peer_t *p = s_peers; p; p = p->next) {
            if (peer_closing(p)) continue;
            p->flags |= MUX_PEER_CLOSING;
            p->mail = NULL;
            session = p->session;
            break;
        }
        peer_give();
        if (!session) break;
        fantasi_proto_session_close(session, &s_mux_transport, 0);
    }
}

/* ---- TinyUSB control callback ------------------------------------------ */

bool usb_proto_control_xfer(uint8_t rhport, uint8_t stage,
                            tusb_control_request_t const *request)
{
    if (!peer_init()) return false;

    if (stage == CONTROL_STAGE_SETUP) {
        control_reset();

        if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
            request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_DEVICE)
            return false;

        bool direction_in = request->bmRequestType_bit.direction == TUSB_DIR_IN;
        switch (request->bRequest) {
        case FANTASI_USB_MUX_OPEN: {
            if (!direction_in || request->wLength < FANTASI_USB_MUX_OPEN_REPLY_SIZE)
                return false;
            mux_peer_t *peer = open_peer(FANTASI_USB_MUX_VERSION,
                                         (uint32_t)request->wValue |
                                         ((uint32_t)request->wIndex << 16));
            if (!peer) return false;
            s_control_reply[0] = FANTASI_USB_MUX_MAGIC_0;
            s_control_reply[1] = FANTASI_USB_MUX_MAGIC_1;
            s_control_reply[2] = FANTASI_USB_MUX_VERSION;
            put_u32le(s_control_reply + 3, peer->session);
            s_control_kind = CTRL_OPEN;
            s_control_peer = peer;
            bool ok = tud_control_xfer(rhport, request, s_control_reply,
                                       FANTASI_USB_MUX_OPEN_REPLY_SIZE);
            if (!ok) {
                control_reset();
                close_peer(peer);
            }
            return ok;
        }
        case FANTASI_USB_MUX_CLOSE: {
            if (direction_in || request->wLength != 0) return false;
            peer_take();
            mux_peer_t *peer = peer_find_locked(request_session(request));
            peer_give();
            if (!peer) return false;
            /* Close now, not in CONTROL_STAGE_ACK. The status-stage completion
             * that would carry a deferred close can be dropped when a
             * concurrent process's SETUP supersedes this transfer's epoch, and
             * a lost close leaves the session lingering ("closed sessions
             * remained in w"). close_peer is idempotent, so acting at SETUP is
             * safe even if the host retries the CLOSE. */
            close_peer(peer);
            s_control_kind = CTRL_NONE;
            s_control_peer = NULL;
            bool ok = tud_control_status(rhport, request);
            if (!ok) control_reset();
            return ok;
        }
        case FANTASI_USB_MUX_WRITE: {
            if (direction_in || request->wLength < 3 ||
                request->wLength > sizeof(s_control_write)) return false;
            uint32_t session = request_session(request);
            peer_take();
            mux_peer_t *peer = peer_find_locked(session);
            peer_give();
            if (!peer) return false;
            s_control_kind = CTRL_WRITE;
            s_control_peer = peer;
            bool ok = tud_control_xfer(rhport, request, s_control_write,
                                       request->wLength);
            if (!ok) control_reset();
            return ok;
        }
#if CFG_TUD_ENDPOINT0_SIZE <= FANTASI_USB_MUX_EP0_SIZE_MAX
        case FANTASI_USB_MUX_CHUNK: {
            if (direction_in || request->wLength < 3 ||
                request->wLength > FANTASI_USB_MUX_CHUNK_MAX)
                return false;    /* malformed SETUP -> device STALL -> host EPIPE */
            uint32_t session = request_session(request);
            /* Each chunk renews the lease because one frame may span many
             * controls and restart before reaching the session queue. */
            if (!fantasi_proto_session_touch(session, &s_mux_transport, 0))
                return false;
            peer_take();
            mux_peer_t *peer = peer_find_locked(session);
            peer_give();
            if (!peer) {
                return false;
            }
            if (peer->version != FANTASI_USB_MUX_VERSION) return false;
            /* Do not release an unread response here. File uploads pipeline
             * several requests before reading their ACKs; the one-slot mailbox
             * must retain the oldest ACK and backpressure this session's worker
             * while later requests accumulate in its bounded job queue. The
             * host explicitly releases each decoded frame with a past-end READ. */
            s_control_kind = CTRL_CHUNK;
            s_control_peer = peer;
            bool ok = tud_control_xfer(rhport, request, s_control_write,
                                       request->wLength);
            if (!ok) control_reset();
            return ok;
        }
#endif
        case FANTASI_USB_MUX_READ: {
            if (!direction_in || request->wLength == 0) return false;
            uint32_t session = request_session(request);
            if (!fantasi_proto_session_touch(session, &s_mux_transport, 0))
                return false;
            peer_take();
            mux_peer_t *peer = peer_find_locked(session);
            if (!peer) { peer_give(); return false; }
            /* READ is stateless: wIndex selects the response offset. The
             * mailbox remains unchanged until the complete frame is
             * acknowledged. */
            uint16_t offset = request->wIndex;
            /* A past-end read acknowledges the frame and releases its mail. */
            if (peer->mail && offset >= peer->mail_len) peer->mail = NULL;
            uint16_t available = (peer->mail && offset < peer->mail_len)
                               ? (uint16_t)(peer->mail_len - offset) : 0;
            uint16_t amount = available < request->wLength ? available : request->wLength;
            uint8_t *data = amount ? peer->mail + offset : NULL;
            s_control_kind = CTRL_READ;
            s_control_peer = peer;
            s_control_read_len = amount;
            bool ok = tud_control_xfer(rhport, request, data, amount);
            peer_give();
            if (!ok) control_reset();
            return ok;
        }
        case FANTASI_USB_MUX_PING: {
            if (!direction_in || request->wLength < 4) return false;
            uint32_t session = request_session(request);
            /* PING renews the core session lease without a protobuf request. */
            if (!fantasi_proto_session_touch(session, &s_mux_transport, 0))
                return false;
            peer_take();
            mux_peer_t *peer = peer_find_locked(session);
            peer_give();
            if (!peer) return false;
            put_magic(s_control_reply, peer->version);
            s_control_kind = CTRL_PING;
            s_control_peer = peer;
            bool ok = tud_control_xfer(rhport, request, s_control_reply, 4);
            if (!ok) control_reset();
            return ok;
        }
        default:
            return false;
        }
    }

    if (stage == CONTROL_STAGE_DATA && s_control_kind == CTRL_CHUNK) {
#if CFG_TUD_ENDPOINT0_SIZE <= FANTASI_USB_MUX_EP0_SIZE_MAX
        bool ok = s_control_peer &&
                  partial_accept(s_control_peer->session, s_control_write,
                                 request->wLength);
        if (!ok) control_reset();
        return ok;
#else
        control_reset();
        return false;
#endif
    }

#if CFG_TUD_ENDPOINT0_SIZE <= FANTASI_USB_MUX_EP0_SIZE_MAX
    if (stage == CONTROL_STAGE_DATA &&
        request->bRequest == FANTASI_USB_MUX_CHUNK &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        s_control_kind != CTRL_CHUNK) {
        /* A CHUNK's data stage arrived but the staged transfer context was
         * lost or replaced between the setup and data callbacks. Falling
         * through to the final `return true` acknowledges data that reached
         * no session - the host advances, the reassembly does not, and the
         * whole frame later dies on a sequence gap. Refuse the status stage
         * so the host's retry machinery recovers. */
        control_reset();
        return false;
    }
#endif

    if (stage == CONTROL_STAGE_DATA && s_control_kind == CTRL_WRITE) {
        uint16_t frame_len = request->wLength;
        uint16_t message_len = (uint16_t)s_control_write[0] |
                               ((uint16_t)s_control_write[1] << 8);
        if (message_len == 0 || message_len > CliRequest_size ||
            frame_len != (uint16_t)(message_len + 2)) {
            control_reset();
            return false;
        }
        bool ok = fantasi_proto_session_submit(s_control_peer->session,
                                               &s_mux_transport, 0,
                                               s_control_write + 2, message_len);
        if (!ok) control_reset();
        return ok;
    }

    if (stage == CONTROL_STAGE_ACK) {
        /* READ commits nothing (stateless offset reads); CLOSE is handled at
         * SETUP time (see the CLOSE case). Just clear the staged context. */
        control_reset();
    }
    return true;
}

/* ---- Legacy vendor bulk endpoint --------------------------------------- */

static size_t bulk_emit(uint32_t session, uint32_t link,
                        const uint8_t *frame, size_t len)
{
    (void)session; (void)link;
    size_t sent = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(250);
    while (sent < len && tud_vendor_mounted()) {
        uint32_t n = tud_vendor_write(frame + sent, len - sent);
        if (!n) {
            tud_vendor_write_flush();
            if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) break;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        deadline = xTaskGetTickCount() + pdMS_TO_TICKS(250);
        sent += n;
    }
    tud_vendor_write_flush();
    return sent;
}

static bool bulk_connected(uint32_t session, uint32_t link)
{
    (void)session; (void)link;
    return tud_vendor_mounted();
}

static const fantasi_proto_transport_t s_bulk_transport = {
    .name = "webusb-bulk",
    .emit = bulk_emit,
    .connected = bulk_connected,
    .release = NULL,
    .ingress_can_emit = true,
    .allow_envelope_open = false,
    .lease_ms = 0,
};

static volatile TaskHandle_t s_vendor_waiter;

static void vendor_wait(uint32_t timeout_ms)
{
    s_vendor_waiter = xTaskGetCurrentTaskHandle();
    if (!tud_vendor_available())
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    s_vendor_waiter = NULL;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf; (void)buffer; (void)bufsize;
    TaskHandle_t task = s_vendor_waiter;
    if (task) xTaskNotifyGive(task);
}

void usb_proto_task(void *arg)
{
    (void)arg;
    peer_init();
    static uint8_t frame[FANTASI_PROTO_FRAME_MAX];
    fantasi_proto_endpoint_t endpoint;
    fantasi_proto_endpoint_init(&endpoint, &s_bulk_transport, 0,
                                frame, sizeof(frame));
    bool was_mounted = false;

    for (;;) {
        bool mounted = tud_vendor_mounted();
        if (!mounted && was_mounted) fantasi_proto_endpoint_down(&endpoint);
        was_mounted = mounted;

        uint8_t input[256];
        size_t n = mounted && tud_vendor_available()
                 ? (size_t)tud_vendor_read(input, sizeof(input)) : 0;
        if (n) fantasi_proto_endpoint_rx(&endpoint, input, n);
        else vendor_wait(100);
        /* Reap only sessions with no worker, queued work, reassembly, or
         * unread mail. */
        fantasi_proto_reap();
    }
}
