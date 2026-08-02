/* Fantasi app RFID API - the app-facing RFID surface.
 *
 * RFID is device- and build-specific, so it is not part of the core
 * fantasi_api_t table. Instead an app obtains a versioned function table through
 * a single loader-resolved symbol, fantasi_rfid(), which the ELF loader patches
 * to the firmware entry when the app references it (the same mechanism Berry
 * uses). fantasi_rfid() returns NULL when the device/build has no RFID.
 *
 * The struct is defined here once and #included by the firmware (core/rfid/
 * rfid_api.c) so both sides share one layout. Every field below ships in this
 * first release (ABI 1); per-platform capability is signalled by a NULL function
 * pointer, not the version. New members are only ever appended, so a post-v1
 * addition bumps FANTASI_RFID_ABI and its user gates on `->abi >= N` (to guard
 * against a freshly-flashed firmware paired with a stale host-side app ELF). */
#ifndef FANTASI_APP_RFID_H
#define FANTASI_APP_RFID_H

#include <stdint.h>

#define FANTASI_RFID_ABI 1

/* Scratch size an hf_sniff_capture caller must provide (decode text + DMA ring +
 * frame buffers). The module allocates this ephemerally for the duration of a
 * sniff and frees it after - no permanent buffer, no capture file. */
#define FANTASI_RFID_SNIFF_BUFSZ 5760

/* Capability bits (mirror hal_rfid.h). */
#define FANTASI_RFID_CAP_HF_READ  (1u << 0)
#define FANTASI_RFID_CAP_HF_EMU   (1u << 1)
#define FANTASI_RFID_CAP_LF_READ  (1u << 2)
#define FANTASI_RFID_CAP_LF_EMU   (1u << 3)

/* Radio modes (mirror rfid_mode_t). */
#define FANTASI_RFID_OFF        0
#define FANTASI_RFID_HF_READER  1
#define FANTASI_RFID_LF_READER  2
#define FANTASI_RFID_HF_EMU     3
#define FANTASI_RFID_LF_EMU     4

/* hf_transceive flags (mirror hal_rfid.h). */
#define FANTASI_RFID_HF_CRC_TX     (1u << 0)
#define FANTASI_RFID_HF_CRC_RX     (1u << 1)
#define FANTASI_RFID_HF_NO_PARITY  (1u << 2)

typedef struct fantasi_rfid {
    uint16_t abi;                       /* FANTASI_RFID_ABI */

    uint32_t (*caps)(void);             /* FANTASI_RFID_CAP_* bitmask */
    int  (*set_mode)(int mode);         /* FANTASI_RFID_* ; 0 or -1 */
    void (*field)(int on);

    /* One framed HF transceive; returns rx bit count or <0 (see hal_rfid.h). */
    int  (*hf_transceive)(const uint8_t *tx, int tx_bits, uint32_t flags,
                          uint8_t *rx, int rx_cap, uint32_t timeout_us);

    /* Custom-parity HF transceive for encrypted 14443-A frames (MIFARE Crypto1): send `nbytes` with the
     * caller's parity bit per byte (par[i]), no CRC; if `rx_par` is non-NULL it receives the tag's per-byte
     * parity bit (same indexing as rx, >= rx_cap) for the nested-nonce attack, else pass NULL. Returns rx
     * bit count or <0 (see hal_rfid.h). */
    int  (*hf_transceive_par)(const uint8_t *tx, int nbytes, const uint8_t *par,
                              uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us);

    /* Full ISO14443-A activation. Fills uid[0..*uid_len-1], *sak, atqa[0..1],
     * *cascade. Returns 0, or <0 (-1 == no tag). Any out-pointer may be NULL. */
    int  (*iso14443a_select)(uint8_t uid[10], int *uid_len,
                             uint8_t *sak, uint8_t atqa[2], int *cascade);

    /* Low-level frontend id/version probe (e.g. MFRC522 VersionReg). */
    int  (*hf_probe)(void);

    /* Self-contained LF EM4100 read: powers the LF field, captures, decodes, and
     * powers down. Fills the 5-byte UID; returns the bit rate (64/32/16), or <0
     * (-1 == no tag). No set_mode/field needed around it. */
    int  (*lf_em4100)(uint8_t uid[5]);

    /* Loadable FPGA gateware (Proxmark3). fpga_resource returns the bitstream
     * basename a mode needs (under /fpga, no extension), or NULL where the device
     * has no FPGA - the app provisions "/fpga/<name>.bit.z" from the host, then
     * set_mode loads it. fpga_load drives an arbitrary compressed bitstream from a
     * VFS path into the gateware (custom designs). Both NULL/unsupported off-FPGA. */
    const char *(*fpga_resource)(int mode);
    int         (*fpga_load)(const char *path);

    /* Passive HF sniff: capture a live reader<->card 13.56 MHz exchange with no field
     * generated, writing a capture/summary to dump_path (waits up to timeout_ms for the
     * field). Returns sample count or <0. NULL/unsupported where the frontend can't
     * passively demodulate. */
    int (*hf_sniff)(const char *dump_path, uint32_t timeout_ms);

    /* Passive HF sniff into a caller-provided buffer (>= FANTASI_RFID_SNIFF_BUFSZ), with no
     * capture file: the HAL runs the timing-critical DMA capture + Miller/Manchester decode and
     * leaves the decoded trace text (L-header + frame lines) at buf[0..return]. Returns the text
     * byte length, 0 if the field was idle, or <0 on error. quiet_ms ends one capture once that
     * long passes with no new frame; max_ms hard-caps one active capture. Lets a sniff module
     * own an ephemeral buffer and print the trace directly - so the frontend keeps no static
     * sniff buffer. NULL where the device sniffs via hf_sniff (file) instead. */
    int (*hf_sniff_capture)(uint8_t *buf, uint32_t cap, uint32_t quiet_ms, uint32_t max_ms);

    /* LF reader->tag downlink: `bits` is one byte per bit (0/1, MSB-first as the caller packed the
     * command), `nbits` long. Runs the T5577 "fixed bit length" gap timing and holds the field for
     * the EEPROM commit; self-contained given set_mode(LF_READER) first. Returns 0, or <0. NULL
     * where the device has no LF transmit. Used by the t5577 module for `raw`/`write`. */
    int (*lf_modulate)(const uint8_t *bits, int nbits, uint32_t opts);

    /* LF reader<->tag round-trip capture (the RX primitive; RX counterpart to lf_modulate's TX):
     * optionally gap-modulate a downlink command (`cmd`/`nbits`, one byte per bit MSB-first; pass
     * NULL/0 for capture-only), hold the field, and stream the raw 125 kHz envelope - one 8-bit
     * sample per carrier cycle - into the caller's `buf` (>= a few bit-periods). Returns the sample
     * count, or <0. This is deliberately just the analog capture: LF has no hardware demod (unlike
     * hf_transceive), so the DSP - clock recovery, ASK/Manchester demod, framing - lives in the
     * calling module, which owns the ephemeral sample buffer and is deleted after use. Reusable by
     * any LF read protocol. Self-contained given set_mode(LF_READER) first. NULL where no LF read. */
    int (*lf_transceive)(const uint8_t *cmd, int nbits, uint8_t *buf, int cap);

    /* ---- appended in later ABIs: emulation ---- */

    /* HF ISO14443-A tag emulation (card side): the real-time bit-I/O + FDT mirror of hf_transceive. After
     * set_mode(FANTASI_RFID_HF_EMU): hf_emu_recv Miller-decodes one reader command into rx[]/rx_par[] (returns
     * byte count, 0 on timeout_ms idle, <0 error); hf_emu_send load-modulates a reply back at the frame delay
     * time - `tosend` is one byte per subcarrier symbol, which the module Manchester-encodes (leading
     * correction template, start bit, per-bit SEC_D/E + parity, stop). The FPGA enforces the 1172-cycle FDT;
     * the 1236 correction comes from the last recv's parity. All protocol logic (anticollision, Crypto1,
     * card image) stays in the module. NULL where the device can't tag-emulate. */
    int (*hf_emu_recv)(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms);
    int (*hf_emu_send)(const uint8_t *tosend, int len);

    /* Streaming tag reply for low-FDT crypto answers: instead of a pre-built buffer, the HAL fetches each
     * subcarrier symbol on demand via next(ctx) - one per call - and streams them at the frame delay time.
     * A module thus encrypts+encodes a Crypto1 reply bit-by-bit during the paced ~9.4us/symbol feed (one
     * c1_bit hides in each gap), so the first symbol - hence the FDT - lands at ~a real card's timing even
     * for an 18-byte encrypted READ, instead of waiting for the whole ciphertext up front. `nsymbols` is the
     * exact symbol count next() will produce (start bit + 9 per byte + stop). NULL where no tag emulation. */
    int (*hf_emu_send_stream)(uint8_t (*next)(void *ctx), void *ctx, int nsymbols);

    /* Buffer form of the crypto reply, for a frontend that can't overlap the producer with transmission (e.g. a
     * CPU-bit-banged reply, where computing a symbol mid-TX would smear the subcarrier): the module pre-encodes
     * the whole reply with its tight in-place loop (iso14a_tag_encode format - one byte per symbol) and hands the
     * finished buffer here, never pointer-cached (the ciphertext changes every auth). A module uses this only when
     * hf_emu_send_stream returns <0 (the frontend doesn't stream), so streaming frontends keep their low-FDT
     * overlap and non-streaming ones still get a fast tight-loop encode. NULL where no tag emulation. */
    int (*hf_emu_send_stream_buf)(const uint8_t *tosend, int len);
} fantasi_rfid_t;

/* Resolved by the loader; NULL when this device/build has no RFID. */
extern const fantasi_rfid_t *fantasi_rfid(void);

#endif /* FANTASI_APP_RFID_H */
