#!/usr/bin/env python3
"""Hardware-free wire-contract test for protobuf, BLE, and WebUSB muxing."""

import os
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pb_decode.h"
#include "pb_encode.h"
#include "fantasi.pb.h"
#include "ble_mux.h"
#include "usb_mux.h"

static uint16_t u16(const uint8_t *p)
{ return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void p16(uint8_t *p, uint16_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void p32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

static uint8_t usb_crc8_update(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (unsigned bit = 0; bit < 8; bit++)
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                            : (uint8_t)(crc << 1);
    return crc;
}

static uint8_t usb_packet_crc(uint32_t sid, const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (unsigned shift = 0; shift < 32; shift += 8)
        crc = usb_crc8_update(crc, (uint8_t)(sid >> shift));
    for (size_t i = 0; i < len; i++) crc = usb_crc8_update(crc, data[i]);
    return crc;
}

static uint16_t usb_frame_crc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xffffu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

typedef struct {
    uint32_t sid;
    uint16_t total, received, crc;
    uint8_t next_sequence, token;
    bool complete;
    uint8_t data[2 + CliRequest_size];
} usb_assembly_t;

static size_t usb_chunk(uint8_t *packet, uint32_t sid, uint8_t token,
                        uint8_t sequence, const uint8_t *frame, uint16_t total,
                        uint16_t offset)
{
    size_t take, len;
    if (sequence == 0) {
        take = 0;
        packet[0] = FANTASI_USB_MUX_CHUNK_START;
        packet[1] = token;
        p16(packet + 2, total);
        p16(packet + 4, usb_frame_crc(frame, total));
        len = 7;
    } else {
        take = total - offset;
        if (take > FANTASI_USB_MUX_CHUNK_DATA_MAX)
            take = FANTASI_USB_MUX_CHUNK_DATA_MAX;
        packet[0] = (uint8_t)take;
        packet[1] = sequence;
        memcpy(packet + 2, frame + offset, take);
        len = 3 + take;
    }
    if (offset + take == total) packet[0] |= FANTASI_USB_MUX_CHUNK_END;
    packet[len - 1] = usb_packet_crc(sid, packet, len - 1);
    assert(len <= FANTASI_USB_MUX_CHUNK_MAX);
    return len;
}

static bool usb_accept(usb_assembly_t *a, uint32_t sid,
                       const uint8_t *packet, size_t len)
{
    if (len < 3 || len > FANTASI_USB_MUX_CHUNK_MAX ||
        usb_packet_crc(sid, packet, len - 1) != packet[len - 1])
        return false;
    uint8_t flags = packet[0];
    uint8_t amount = flags & FANTASI_USB_MUX_CHUNK_LEN_MASK;
    bool start = (flags & FANTASI_USB_MUX_CHUNK_START) != 0;
    bool end = (flags & FANTASI_USB_MUX_CHUNK_END) != 0;
    if (start) {
        if (len != 7 || amount != 0) return false;
        memset(a, 0, sizeof(*a));
        a->sid = sid; a->total = u16(packet + 2); a->crc = u16(packet + 4);
        a->token = packet[1]; a->next_sequence = 1;
        return a->total >= 3 && a->total <= sizeof(a->data) && !end;
    }
    if (a->sid != sid || packet[1] != a->next_sequence ||
        amount > FANTASI_USB_MUX_CHUNK_DATA_MAX || len != 3u + amount ||
        a->received + amount > a->total ||
        end != (a->received + amount == a->total))
        return false;
    memcpy(a->data + a->received, packet + 2, amount);
    a->received += amount;
    a->next_sequence++;
    if (end) a->complete = usb_frame_crc(a->data, a->total) == a->crc;
    return true;
}

static size_t encode_req(const CliRequest *req, uint8_t *out)
{
    pb_ostream_t s = pb_ostream_from_buffer(out, CliRequest_size);
    assert(pb_encode(&s, CliRequest_fields, req));
    return s.bytes_written;
}

typedef struct {
    uint32_t sid;
    uint16_t total, received;
    uint8_t data[CliRequest_size];
} assembly_t;

static void fragment_into(assembly_t *a, uint32_t sid,
                          const uint8_t *message, uint16_t total,
                          uint16_t offset, uint16_t amount)
{
    uint8_t packet[FANTASI_BLE_MUX_HEADER_SIZE + 17];
    assert(amount <= 17 && offset + amount <= total);
    packet[0] = FANTASI_BLE_MUX_MAGIC_0;
    packet[1] = FANTASI_BLE_MUX_MAGIC_1;
    packet[2] = FANTASI_BLE_MUX_MAGIC_2;
    packet[3] = FANTASI_BLE_MUX_MAGIC_3;
    p32(packet + 4, sid); p16(packet + 8, total); p16(packet + 10, offset);
    memcpy(packet + FANTASI_BLE_MUX_HEADER_SIZE, message + offset, amount);

    assert(u32(packet + 4) == sid && u16(packet + 8) == total);
    if (offset == 0) { a->sid = sid; a->total = total; a->received = 0; }
    assert(a->sid == sid && a->total == total && a->received == offset);
    memcpy(a->data + offset, packet + FANTASI_BLE_MUX_HEADER_SIZE, amount);
    a->received += amount;
}

static fantasi_ble_mux_response_result_t response_fragment(
    fantasi_ble_mux_response_rx_t *state, uint8_t *assembled, size_t cap,
    uint32_t sid, uint16_t sequence, const uint8_t *frame, uint16_t total,
    uint16_t offset, uint16_t amount, size_t *complete_len)
{
    uint8_t packet[FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE + 19];
    assert(amount <= 19 && offset + amount <= total);
    packet[0] = FANTASI_BLE_MUX_RESPONSE_MAGIC_0;
    packet[1] = FANTASI_BLE_MUX_RESPONSE_MAGIC_1;
    packet[2] = FANTASI_BLE_MUX_RESPONSE_MAGIC_2;
    packet[3] = FANTASI_BLE_MUX_RESPONSE_MAGIC_3;
    p32(packet + 4, sid);
    p16(packet + 8, sequence);
    p16(packet + 10, total);
    p16(packet + 12, offset);
    memcpy(packet + FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE,
           frame + offset, amount);
    return fantasi_ble_mux_response_accept(
        state, assembled, cap, packet,
        FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE + amount, complete_len);
}

int main(void)
{
    _Static_assert(CliRequest_session_open_tag == 13, "wire tag drift");
    _Static_assert(CliRequest_session_close_tag == 14, "wire tag drift");
    _Static_assert(CliRequest_cancel_tag == 15, "wire tag drift");
    _Static_assert(CliRequest_session_ping_tag == 16, "wire tag drift");
    _Static_assert(CliRequest_size < UINT16_MAX, "BLE length field too small");
    _Static_assert(FANTASI_BLE_MUX_PACKET_MAX <= 245,
                   "STM32WB would split one mux datagram across HCI events");
    _Static_assert(FANTASI_USB_MUX_OPEN != FANTASI_USB_MUX_CLOSE, "USB op collision");
    _Static_assert(FANTASI_USB_MUX_WRITE != FANTASI_USB_MUX_READ, "USB op collision");
    _Static_assert(FANTASI_USB_MUX_OPEN != FANTASI_USB_MUX_CHUNK, "USB mux op collision");
    _Static_assert(FANTASI_USB_MUX_VERSION == 1, "USB mux wire version drift");
    _Static_assert(FANTASI_USB_MUX_CHUNK_MAX < FANTASI_USB_MUX_EP0_SIZE_MAX,
                   "SAM7 chunks must be short EP0 packets");
    _Static_assert(FANTASI_USB_MUX_OPEN_REPLY_SIZE < FANTASI_USB_MUX_EP0_SIZE_MAX,
                   "SAM7 OPEN must be a short EP0 packet");

    /* The BLE marker can never be mistaken for a valid legacy frame length. */
    assert(((uint16_t)FANTASI_BLE_MUX_MAGIC_0 |
            ((uint16_t)FANTASI_BLE_MUX_MAGIC_1 << 8)) > CliRequest_size);
    assert(((uint16_t)FANTASI_BLE_MUX_RESPONSE_MAGIC_0 |
            ((uint16_t)FANTASI_BLE_MUX_RESPONSE_MAGIC_1 << 8)) > CliResponse_size);

    CliRequest open = CliRequest_init_zero;
    open.id = 0x10203040u;
    open.which_payload = CliRequest_session_open_tag;
    open.payload.session_open = true;
    uint8_t wire[CliRequest_size];
    size_t n = encode_req(&open, wire);
    CliRequest decoded = CliRequest_init_zero;
    pb_istream_t in = pb_istream_from_buffer(wire, n);
    assert(pb_decode(&in, CliRequest_fields, &decoded));
    assert(decoded.id == open.id && !decoded.has_session &&
           decoded.which_payload == CliRequest_session_open_tag);

    CliRequest req[2] = { CliRequest_init_zero, CliRequest_init_zero };
    uint8_t encoded[2][CliRequest_size];
    size_t lengths[2];
    for (unsigned i = 0; i < 2; i++) {
        req[i].id = 100 + i;
        req[i].has_session = true;
        req[i].session = 0xA0B00000u + i;
        req[i].which_payload = CliRequest_file_write_tag;
        strcpy(req[i].payload.file_write.path, i ? "/ramfs/b" : "/ramfs/a");
        req[i].payload.file_write.offset = 0;
        req[i].payload.file_write.last = true;
        req[i].payload.file_write.data.size = 97;
        memset(req[i].payload.file_write.data.bytes, (int)('A' + i), 97);
        lengths[i] = encode_req(&req[i], encoded[i]);
    }

    /* Round-robin fragments model independent BlueZ writers. Reassembly is by
     * SID, so interleaving changes neither request. */
    assembly_t a[2] = {{0}};
    size_t off[2] = {0, 0};
    while (off[0] < lengths[0] || off[1] < lengths[1]) {
        for (unsigned i = 0; i < 2; i++) {
            if (off[i] >= lengths[i]) continue;
            size_t amount = lengths[i] - off[i];
            if (amount > 17) amount = 17;
            fragment_into(&a[i], req[i].session, encoded[i],
                          (uint16_t)lengths[i], (uint16_t)off[i], (uint16_t)amount);
            off[i] += amount;
        }
    }
    for (unsigned i = 0; i < 2; i++) {
        assert(a[i].received == lengths[i]);
        assert(memcmp(a[i].data, encoded[i], lengths[i]) == 0);
        CliRequest roundtrip = CliRequest_init_zero;
        in = pb_istream_from_buffer(a[i].data, a[i].received);
        assert(pb_decode(&in, CliRequest_fields, &roundtrip));
        assert(roundtrip.has_session && roundtrip.session == req[i].session);
        assert(roundtrip.payload.file_write.data.size == 97);
    }

    /* The SAM7 WebUSB path uses only short, independently checked controls.
     * Interleave two device-owned sessions packet-by-packet and prove both
     * framed protobuf requests survive without sharing transport state. */
    uint8_t usb_frame[2][2 + CliRequest_size];
    uint16_t usb_len[2];
    usb_assembly_t usb_a[2] = {{0}};
    uint16_t usb_off[2] = {0, 0};
    uint8_t usb_seq[2] = {0, 0};
    uint8_t packet[FANTASI_USB_MUX_CHUNK_MAX];
    for (unsigned i = 0; i < 2; i++) {
        usb_len[i] = (uint16_t)(lengths[i] + 2);
        p16(usb_frame[i], (uint16_t)lengths[i]);
        memcpy(usb_frame[i] + 2, encoded[i], lengths[i]);
        size_t plen = usb_chunk(packet, req[i].session, (uint8_t)(9 + i),
                                0, usb_frame[i], usb_len[i], 0);
        assert(usb_accept(&usb_a[i], req[i].session, packet, plen));
        usb_seq[i] = 1;
    }
    while (usb_off[0] < usb_len[0] || usb_off[1] < usb_len[1]) {
        for (unsigned i = 0; i < 2; i++) {
            if (usb_off[i] >= usb_len[i]) continue;
            size_t plen = usb_chunk(packet, req[i].session, (uint8_t)(9 + i),
                                    usb_seq[i]++, usb_frame[i], usb_len[i],
                                    usb_off[i]);
            uint8_t amount = packet[0] & FANTASI_USB_MUX_CHUNK_LEN_MASK;
            assert(usb_accept(&usb_a[i], req[i].session, packet, plen));
            usb_off[i] += amount;
        }
    }
    for (unsigned i = 0; i < 2; i++) {
        assert(usb_a[i].complete && usb_a[i].received == usb_len[i]);
        assert(usb_a[i].token == 9 + i);
        assert(memcmp(usb_a[i].data, usb_frame[i], usb_len[i]) == 0);
    }

    /* A stale/corrupt FIFO byte is rejected by the per-packet checksum before
     * it can advance reassembly; the frame CRC independently guards END. */
    size_t bad_len = usb_chunk(packet, req[0].session, 33, 0,
                               usb_frame[0], usb_len[0], 0);
    packet[4] ^= 0x40;
    assert(!usb_accept(&usb_a[0], req[0].session, packet, bad_len));

    CliRequest cancel = CliRequest_init_zero;
    cancel.id = 9; cancel.has_session = true; cancel.session = 44;
    cancel.which_payload = CliRequest_cancel_tag;
    cancel.payload.cancel.request_id = 8;
    n = encode_req(&cancel, wire);
    decoded = (CliRequest)CliRequest_init_zero;
    in = pb_istream_from_buffer(wire, n);
    assert(pb_decode(&in, CliRequest_fields, &decoded));
    assert(decoded.session == 44 && decoded.payload.cancel.request_id == 8);

    CliRequest close = CliRequest_init_zero;
    close.id = 10; close.has_session = true; close.session = 44;
    close.which_payload = CliRequest_session_close_tag;
    close.payload.session_close = true;
    n = encode_req(&close, wire);
    decoded = (CliRequest)CliRequest_init_zero;
    in = pb_istream_from_buffer(wire, n);
    assert(pb_decode(&in, CliRequest_fields, &decoded));
    assert(decoded.has_session && decoded.session == 44 &&
           decoded.which_payload == CliRequest_session_close_tag &&
           decoded.payload.session_close);

    CliResponse response = CliResponse_init_zero;
    response.id = 77; response.has_next = false;
    response.has_session = true; response.session = 44;
    response.which_payload = CliResponse_output_tag;
    strcpy(response.payload.output, "ok");
    uint8_t response_wire[CliResponse_size];
    pb_ostream_t out = pb_ostream_from_buffer(response_wire, sizeof(response_wire));
    assert(pb_encode(&out, CliResponse_fields, &response));
    CliResponse response_decoded = CliResponse_init_zero;
    in = pb_istream_from_buffer(response_wire, out.bytes_written);
    assert(pb_decode(&in, CliResponse_fields, &response_decoded));
    assert(response_decoded.has_session && response_decoded.session == 44);

    /* BlueZ can deliver each ATT Value update once per independent
     * StartNotify owner. Duplicate every fragment and prove the receiver emits
     * exactly one intact protobuf frame, including when the whole frame fits in
     * a single notification. A new sequence remains a legitimate frame even
     * when its bytes are identical to the preceding one. */
    uint8_t framed[2 + CliResponse_size];
    p16(framed, (uint16_t)out.bytes_written);
    memcpy(framed + 2, response_wire, out.bytes_written);
    uint16_t framed_len = (uint16_t)(out.bytes_written + 2);
    uint8_t assembled[sizeof(framed)];
    fantasi_ble_mux_response_rx_t response_rx = {0};
    unsigned completed = 0;
    for (uint16_t off = 0; off < framed_len; ) {
        uint16_t amount = framed_len - off;
        if (amount > 7) amount = 7;
        size_t complete_len = 0;
        fantasi_ble_mux_response_result_t r = response_fragment(
            &response_rx, assembled, sizeof(assembled), 44, 41,
            framed, framed_len, off, amount, &complete_len);
        if (r == FANTASI_BLE_MUX_RESPONSE_COMPLETE) {
            completed++;
            assert(complete_len == framed_len);
        } else {
            assert(r == FANTASI_BLE_MUX_RESPONSE_CONSUMED);
        }
        /* The duplicate notification must never advance or complete again. */
        complete_len = 0;
        assert(response_fragment(&response_rx, assembled, sizeof(assembled),
                                 44, 41, framed, framed_len, off, amount,
                                 &complete_len) ==
               FANTASI_BLE_MUX_RESPONSE_CONSUMED);
        assert(complete_len == 0);
        off += amount;
    }
    assert(completed == 1 && memcmp(assembled, framed, framed_len) == 0);

    size_t complete_len = 0;
    assert(response_fragment(&response_rx, assembled, sizeof(assembled),
                             44, 42, framed, framed_len, 0, framed_len,
                             &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_COMPLETE);
    assert(complete_len == framed_len);
    complete_len = 0;
    assert(response_fragment(&response_rx, assembled, sizeof(assembled),
                             44, 42, framed, framed_len, 0, framed_len,
                             &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_CONSUMED);

    /* A duplicate can be delayed until after a response for another session.
     * The device sequence is global, so the older frame must still be dropped. */
    complete_len = 0;
    assert(response_fragment(&response_rx, assembled, sizeof(assembled),
                             99, 43, framed, framed_len, 0, framed_len,
                             &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_COMPLETE);
    assert(response_fragment(&response_rx, assembled, sizeof(assembled),
                             44, 42, framed, framed_len, 0, framed_len,
                             &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_CONSUMED);

    /* Accept zero padding to the negotiated ATT payload for compatibility;
     * non-zero excess remains malformed. */
    uint8_t padded[FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE + 64] = {0};
    padded[0] = FANTASI_BLE_MUX_RESPONSE_MAGIC_0;
    padded[1] = FANTASI_BLE_MUX_RESPONSE_MAGIC_1;
    padded[2] = FANTASI_BLE_MUX_RESPONSE_MAGIC_2;
    padded[3] = FANTASI_BLE_MUX_RESPONSE_MAGIC_3;
    p32(padded + 4, 44); p16(padded + 8, 44);
    p16(padded + 10, framed_len); p16(padded + 12, 0);
    memcpy(padded + FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE, framed, framed_len);
    complete_len = 0;
    assert(fantasi_ble_mux_response_accept(&response_rx, assembled,
                                           sizeof(assembled), padded,
                                           sizeof(padded), &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_COMPLETE);
    assert(complete_len == framed_len &&
           memcmp(assembled, framed, framed_len) == 0);

    p16(padded + 8, 45);
    padded[FANTASI_BLE_MUX_RESPONSE_HEADER_SIZE + framed_len] = 1;
    complete_len = 0;
    assert(fantasi_ble_mux_response_accept(&response_rx, assembled,
                                           sizeof(assembled), padded,
                                           sizeof(padded), &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_CONSUMED);
    assert(complete_len == 0);

    uint8_t legacy[] = { 1, 0, 0x08 };
    assert(fantasi_ble_mux_response_accept(&response_rx, assembled,
                                           sizeof(assembled), legacy,
                                           sizeof(legacy), &complete_len) ==
           FANTASI_BLE_MUX_RESPONSE_RAW);

    puts("PASS: multiplex wire contract and duplicate-safe BLE fragments");
    return 0;
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="fantasi-mux-unit-") as td:
        source = os.path.join(td, "mux_test.c")
        binary = os.path.join(td, "mux_test")
        with open(source, "w", encoding="utf-8") as f:
            f.write(HARNESS)
        cmd = [
            os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", os.path.join(REPO_ROOT, "third_party/nanopb"),
            "-I", os.path.join(REPO_ROOT, "proto"),
            source,
            os.path.join(REPO_ROOT, "proto/fantasi.pb.c"),
            os.path.join(REPO_ROOT, "third_party/nanopb/pb_encode.c"),
            os.path.join(REPO_ROOT, "third_party/nanopb/pb_decode.c"),
            os.path.join(REPO_ROOT, "third_party/nanopb/pb_common.c"),
            "-o", binary,
        ]
        built = subprocess.run(cmd, capture_output=True, text=True)
        if built.returncode:
            print("FAIL: mux harness did not compile")
            print(built.stdout + built.stderr)
            return 1
        ran = subprocess.run([binary], capture_output=True, text=True)
        sys.stdout.write(ran.stdout)
        if ran.returncode:
            print(ran.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
