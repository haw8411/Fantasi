#ifndef FANTASI_PROTO_H
#define FANTASI_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fantasi.pb.h"

/* A protobuf transport supplies one immutable vtable. `link` identifies the
 * physical route within that transport (zero for the current single-link BLE
 * and legacy WebUSB paths). `session` is zero for an implicit legacy stream or
 * the allocated SID for an explicit session. emit() must keep a complete framed
 * response atomic with respect to other sessions on the same physical link. */
typedef struct fantasi_proto_transport {
    const char *name;
    size_t (*emit)(uint32_t session, uint32_t link,
                   const uint8_t *frame, size_t len);
    bool   (*connected)(uint32_t session, uint32_t link);
    /* Called once, after a closed session's worker/app references are gone.
     * WebUSB uses it to release the session mailbox; BLE has no side object. */
    void   (*release)(uint32_t session, uint32_t link);
    /* Optional: report whether the transport still holds in-flight state for
     * this session (a partial request being reassembled, or an unread response
     * mailbox). The lease reaper treats such a session as active and never
     * reaps it, so a request stalled mid-transfer under contention is not
     * mistaken for an abandoned session. NULL means "no such state". */
    bool   (*has_pending)(uint32_t session, uint32_t link);
    /* Admission failures happen on the physical RX task. Stream transports may
     * report them immediately; a rendezvous transport must not, because its
     * emitter waits for a read that cannot begin until the write completes. */
    bool ingress_can_emit;
    bool allow_envelope_open;
    uint32_t lease_ms;             /* zero: transport owns cleanup / no lease */
} fantasi_proto_transport_t;

typedef struct fantasi_proto_session fantasi_proto_session_t;

/* A physical framed byte stream owns one endpoint/parser and one implicit
 * legacy session. The parser storage is supplied by the transport so there is
 * no fixed per-logical-session receive buffer. */
#define FANTASI_PROTO_FRAME_MAX (2u + CliRequest_size)
typedef struct fantasi_proto_endpoint {
    const fantasi_proto_transport_t *transport;
    uint32_t link;
    fantasi_proto_session_t *legacy;
    uint8_t *frame;
    uint16_t frame_len;
    uint16_t frame_cap;
} fantasi_proto_endpoint_t;

void fantasi_proto_init(void);

void fantasi_proto_endpoint_init(fantasi_proto_endpoint_t *ep,
                                 const fantasi_proto_transport_t *transport,
                                 uint32_t link, uint8_t *frame,
                                 uint16_t frame_cap);
void fantasi_proto_endpoint_rx(fantasi_proto_endpoint_t *ep,
                               const uint8_t *data, size_t len);
/* Close the implicit session and every explicit session on this route. */
void fantasi_proto_endpoint_down(fantasi_proto_endpoint_t *ep);

/* Direct session API used by the independent WebUSB control-endpoint
 * transport. OPEN is performed by the USB control request itself; subsequent
 * protobuf requests must carry the returned session value. */
uint32_t fantasi_proto_session_open(const fantasi_proto_transport_t *transport,
                                    uint32_t link);
bool fantasi_proto_session_submit(uint32_t session,
                                  const fantasi_proto_transport_t *transport,
                                  uint32_t link,
                                  const uint8_t *message, uint16_t message_len);
/* Renew an explicit session's inactivity lease without submitting work. Used
 * by out-of-band transport heartbeats such as WebUSB PING. Returns false when
 * the addressed session no longer exists. */
bool fantasi_proto_session_touch(uint32_t session,
                                 const fantasi_proto_transport_t *transport,
                                 uint32_t link);
void fantasi_proto_session_close(uint32_t session,
                                 const fantasi_proto_transport_t *transport,
                                 uint32_t link);

/* Used by the `w` command. It walks the dynamic registry without a fixed-size
 * snapshot and never holds the registry lock while writing output. */
void fantasi_proto_write_sessions(void);
void fantasi_proto_reap(void);

/* BLE notification payload negotiation. Kept as the platform callback entry
 * point; the transport uses it to fragment an atomic response frame. */
void proto_set_mtu(uint16_t att_mtu);

/* BLE receive/poll task entry. */
void proto_task(void *arg);

#endif
