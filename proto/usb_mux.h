#ifndef FANTASI_USB_MUX_H
#define FANTASI_USB_MUX_H

/* Device-recipient vendor control requests used by the independent WebUSB
 * session transport. These intentionally do not claim the bulk vendor
 * interface, so several host processes can open their own device-owned session
 * at once. Requests 1 and 2 remain reserved for WebUSB/MS OS descriptors.
 *
 * Addressing: the session id is 16-bit and travels in wValue; wIndex is 0 for
 * every request except READ, where it carries the response byte offset (see
 * below). Session ids are allocated 16-bit (core/proto.c) precisely so wIndex
 * is free for the READ offset.
 *
 * READ is stateless and idempotent: the host asks for the bytes at a given
 * offset of the current response frame, and the device returns them as a pure
 * function of (session, offset) without mutating any per-read state. A retried
 * or aborted READ therefore can never duplicate or drop response bytes - the
 * mirror of the CHUNK write path's token/sequence idempotence. The device
 * releases a response frame's mailbox only when the host reads at or past its
 * end (the host's per-frame ACK, which also lets a streamed multi-frame
 * response advance). A later pipelined request must not discard an earlier
 * unread response. */
#define FANTASI_USB_MUX_OPEN   0x30
#define FANTASI_USB_MUX_CLOSE  0x31
#define FANTASI_USB_MUX_WRITE  0x32
#define FANTASI_USB_MUX_READ   0x33
#define FANTASI_USB_MUX_PING   0x34
#define FANTASI_USB_MUX_CHUNK  0x35

/* SAM7S exposes an eight-byte EP0 and cannot reliably defer a multi-packet
 * control-OUT transfer. The host therefore sends one independently checked
 * short packet per CHUNK request on such devices. Byte 0 carries START/END and
 * the protobuf-frame payload count. Byte 1 is a per-frame retry token in START
 * and a monotonically increasing chunk sequence thereafter. A START packet
 * additionally carries uint16 total length + uint16 frame CRC and no frame
 * data. Other packets carry up to four bytes. The final byte is a packet CRC-8.
 * Hosts also cap mux READ requests to seven bytes, so neither direction uses a
 * full-size or multi-packet SAM7S control data stage. Reassembly exists only
 * while a request is in flight. */
#define FANTASI_USB_MUX_EP0_SIZE_MAX    8u
#define FANTASI_USB_MUX_CHUNK_MAX       7u
#define FANTASI_USB_MUX_CHUNK_START     0x40u
#define FANTASI_USB_MUX_CHUNK_END       0x80u
#define FANTASI_USB_MUX_CHUNK_LEN_MASK  0x07u
#define FANTASI_USB_MUX_CHUNK_DATA_MAX  4u
#define FANTASI_USB_MUX_CHUNK_START_DATA_MAX 0u

/* This is the first and only multiplexed WebUSB wire version. OPEN returns a
 * deliberately short seven-byte reply: "FM", version, and a uint32 LE session
 * ID. PING returns "FMX", version. Keeping OPEN below one full SAM7S EP0 packet
 * avoids the controller's unreliable exact-max-packet transfer case. */
#define FANTASI_USB_MUX_VERSION 1u
#define FANTASI_USB_MUX_MAGIC_0 'F'
#define FANTASI_USB_MUX_MAGIC_1 'M'
#define FANTASI_USB_MUX_MAGIC_2 'X'
#define FANTASI_USB_MUX_OPEN_REPLY_SIZE 7u

/* A dead host cannot leave a worker/mailbox resident forever. Interactive host
 * CLIs ping from their readline event hook; active commands poll READ. */
#define FANTASI_USB_MUX_LEASE_MS 15000u

#endif
