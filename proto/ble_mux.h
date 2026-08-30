#ifndef FANTASI_BLE_MUX_H
#define FANTASI_BLE_MUX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* A logical-session request may span several ATT writes.  Every new client
 * wraps each fragment in this small datagram header so writes from independent
 * host processes can be interleaved without interleaving their protobuf byte
 * streams.  The GATT receive path preserves each ATT write as one message.
 *
 *   magic[4], session_le32, message_len_le16, offset_le16, payload...
 *
 * The magic, interpreted as a legacy little-endian frame length, is far above
 * CliRequest_size.  Legacy clients therefore remain unambiguous. */
#define FANTASI_BLE_MUX_MAGIC_0 0xF3u
#define FANTASI_BLE_MUX_MAGIC_1 0xA5u
#define FANTASI_BLE_MUX_MAGIC_2 0x6Du
#define FANTASI_BLE_MUX_MAGIC_3 0x58u

#define FANTASI_BLE_MUX_HEADER_SIZE 12u

/* Keep each envelope within one STM32WB HCI vendor event. Although an ATT
 * MTU of 256 permits a 253-byte characteristic value, the coprocessor must
 * split values above 245 bytes across ACI_GATT_ATTRIBUTE_MODIFIED events
 * (the event itself has an eight-byte payload header). Preserving one ATT
 * write as one message is part of the mux wire contract, so use the common
 * 244-byte data-channel ceiling on every platform. */
#define FANTASI_BLE_MUX_PACKET_MAX 244u

/* Device-to-host notifications need their own envelope as well. BlueZ emits a
 * GattCharacteristic1 Value signal once per StartNotify owner, so two wholly
 * independent CLI processes can each observe every physical notification
 * twice. Raw byte-stream framing cannot distinguish those duplicates, and a
 * duplicated middle ATT fragment corrupts a multi-notification protobuf frame.
 *
 * A globally monotonic frame sequence (the device serializes complete response
 * frames on the physical link) plus the original frame length and fragment
 * offset lets every host process reassemble exactly one copy. The SID permits
 * routing before protobuf decoding, including the OPEN response where the host
 * does not know its new SID yet.
 *
 *   magic[4], session_le32, sequence_le16, frame_len_le16, offset_le16,
 *   payload...
 */
#define FANTASI_BLE_MUX_RESPONSE_MAGIC_0 0xF3u
#define FANTASI_BLE_MUX_RESPONSE_MAGIC_1 0xA5u
#define FANTASI_BLE_MUX_RESPONSE_MAGIC_2 0x6Du
#define FANTASI_BLE_MUX_RESPONSE_MAGIC_3 0x59u

#define FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE 14u

typedef enum {
    FANTASI_BLE_MUX_RESPONSE_RAW = 0,
    FANTASI_BLE_MUX_RESPONSE_CONSUMED,
    FANTASI_BLE_MUX_RESPONSE_COMPLETE,
} fantasi_ble_mux_response_result_t;

typedef struct {
    uint32_t session;
    uint16_t total;
    uint16_t received;
    uint16_t sequence;
    uint16_t last_sequence;
    bool active;
    bool last_valid;
} fantasi_ble_mux_response_rx_t;

static inline uint16_t fantasi_ble_mux_u16le(const uint8_t *in)
{
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

static inline uint32_t fantasi_ble_mux_u32le(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

/* Consume one complete ATT notification. `frame` is caller-owned scratch with
 * capacity for one length-prefixed CliResponse. On COMPLETE, `complete_len`
 * names the exact bytes to append to the normal protobuf receive stream. */
static inline fantasi_ble_mux_response_result_t
fantasi_ble_mux_response_accept(fantasi_ble_mux_response_rx_t *state,
                                uint8_t *frame, size_t frame_cap,
                                const uint8_t *packet, size_t packet_len,
                                size_t *complete_len)
{
    if (complete_len) *complete_len = 0;
    if (packet_len < 4 ||
        packet[0] != FANTASI_BLE_MUX_RESPONSE_MAGIC_0 ||
        packet[1] != FANTASI_BLE_MUX_RESPONSE_MAGIC_1 ||
        packet[2] != FANTASI_BLE_MUX_RESPONSE_MAGIC_2 ||
        packet[3] != FANTASI_BLE_MUX_RESPONSE_MAGIC_3)
        return FANTASI_BLE_MUX_RESPONSE_RAW;

    /* Recognized-but-malformed packets are consumed instead of poisoning the
     * backwards-compatible raw response stream. */
    if (!state || !frame || packet_len <= FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE)
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;

    uint32_t session = fantasi_ble_mux_u32le(packet + 4);
    uint16_t sequence = fantasi_ble_mux_u16le(packet + 8);
    uint16_t total = fantasi_ble_mux_u16le(packet + 10);
    uint16_t offset = fantasi_ble_mux_u16le(packet + 12);
    size_t payload_len = packet_len - FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE;
    const uint8_t *payload = packet + FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE;

    if (!session || total < 3 || total > frame_cap || offset >= total) {
        state->active = false;
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;
    }

    if (payload_len > (size_t)(total - offset)) {
        state->active = false;
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;
    }

    /* Notifications for different sessions can be interspersed before BlueZ
     * delivers a delayed duplicate. Sequence numbers are global, so retain a
     * global high-water mark rather than only the preceding (SID, sequence)
     * pair. Signed subtraction gives the usual wrap-safe 16-bit ordering as
     * long as a live reader does not fall more than half the sequence space
     * behind. A fragment of the active (newer) frame still passes this test. */
    if (state->last_valid &&
        (int16_t)(sequence - state->last_sequence) <= 0)
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;

    if (offset == 0) {
        if (state->active && state->session == session &&
            state->sequence == sequence && state->total == total &&
            payload_len <= state->received &&
            memcmp(frame, payload, payload_len) == 0)
            return FANTASI_BLE_MUX_RESPONSE_CONSUMED; /* duplicate first part */

        state->session = session;
        state->sequence = sequence;
        state->total = total;
        state->received = 0;
        state->active = true;
    } else if (!state->active || state->session != session ||
               state->sequence != sequence || state->total != total) {
        /* A subscriber may join halfway through a frame. Wait for the next
         * offset-zero packet instead of treating the suffix as raw bytes. */
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;
    }

    if (offset < state->received) {
        if ((size_t)offset + payload_len <= state->received &&
            memcmp(frame + offset, payload, payload_len) == 0)
            return FANTASI_BLE_MUX_RESPONSE_CONSUMED; /* duplicate middle part */
        state->active = false;
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;
    }
    if (offset != state->received) {
        state->active = false;                 /* gap or out-of-order fragment */
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;
    }

    memcpy(frame + offset, payload, payload_len);
    state->received += (uint16_t)payload_len;
    if (state->received != state->total)
        return FANTASI_BLE_MUX_RESPONSE_CONSUMED;

    state->active = false;
    state->last_valid = true;
    state->last_sequence = sequence;
    if (complete_len) *complete_len = state->total;
    return FANTASI_BLE_MUX_RESPONSE_COMPLETE;
}

#endif
