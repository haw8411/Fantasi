#ifndef FANTASI_HAL_RFID_H
#define FANTASI_HAL_RFID_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* RFID HAL contract.
 *
 * There is no common register model across targets - the Proxmark3 drives a
 * Spartan-II FPGA over SSC/DMA, the Flipper an ST25R3916 over SPI, the Chameleon
 * an MFRC522 (HF read) / nRF NFCT (HF emulate) / analog LF chain. The only
 * portable abstraction is a logical one: HF is a framed *transceive*, LF is a
 * raw *acquire* + *modulate*. All protocol logic (anticollision, EM4100 decode,
 * CRC) lives above this in core/rfid/, written once against these primitives.
 *
 * Every entry weak-defaults to "unsupported" in core/rfid/rfid_hal_weak.c, so a
 * platform links before its driver exists and callers see hal_rfid_caps() == 0
 * (mirrors the hal_hid_* pattern). See docs/rfid.md. */

/* ---- Capability bits (advertised per device via hal_rfid_caps) ---- */
#define RFID_CAP_HF_READ   (1u << 0)
#define RFID_CAP_HF_EMU    (1u << 1)
#define RFID_CAP_LF_READ   (1u << 2)
#define RFID_CAP_LF_EMU    (1u << 3)

/* Returns the OR of RFID_CAP_* this device supports (0 when none). */
uint32_t hal_rfid_caps(void);

/* ---- Exclusive radio mode ----
 * The radio is a single, contended resource: on the Chameleon one antenna mux +
 * power rail switch reader vs emulator and PWM0 is shared; on the Proxmark3 the
 * FPGA holds exactly one bitstream. The HAL owns the mode and arbitrates the
 * transition (mux, power, bitstream swap, settle delay) internally. */
typedef enum {
    RFID_OFF = 0,
    RFID_HF_READER,
    RFID_LF_READER,
    RFID_HF_EMU,
    RFID_LF_EMU,
} rfid_mode_t;

/* Enter a mode. Returns 0 on success, -1 if the device lacks it. Idempotent. */
int  hal_rfid_set_mode(rfid_mode_t mode);

/* Energise / de-energise the carrier for the current reader mode. */
void hal_rfid_field(bool on);

/* Periodic hook (call from a task loop): auto-parks the reader carrier once it
 * has been left on and idle, so no op or host command can leave the field
 * driving the antenna. No-op on platforms that don't define it. */
void hal_rfid_field_tick(void);

/* ---- Loadable gateware (FPGA-backed platforms; weak-default NULL/unsupported) ----
 * The Proxmark3's Spartan-II holds exactly one bitstream at a time and there is
 * not enough app-region flash to embed every protocol's, so bitstreams live
 * compressed under /fpga (streamed there from the host on first use) and set_mode
 * loads the one it needs. These let an app provision that file ahead of time and
 * let a custom app drive its own design. Chip-based platforms return NULL / do
 * nothing (set_mode reconfigures a fixed frontend, no bitstream involved). */

/* Bitstream basename a mode needs (under /fpga, no extension), or NULL if the
 * platform needs none. The app provisions "/fpga/<name>.bit.z" before set_mode. */
const char *hal_rfid_fpga_resource(rfid_mode_t mode);

/* Load a compressed bitstream (tools/fpga_lzss.py format) from a VFS path into
 * the gateware. Returns 0, or RFID_ERR_UNSUPP (no FPGA / missing file / failed). */
int  hal_rfid_fpga_load(const char *path);

/* ---- HF reader: one framed transceive ----
 * Send `tx_bits` bits from `tx` (a partial final byte is allowed - e.g. the
 * 7-bit REQA/WUPA short frame, or a bit-count during anticollision), then
 * receive the tag response into `rx` (up to `rx_cap` bytes). Returns the number
 * of *bits* received, or <0 on timeout / error (see RFID_ERR_*).
 *
 * flags:
 *   RFID_HF_CRC_TX  append a CRC_A to the transmitted frame (hardware or core)
 *   RFID_HF_CRC_RX  validate + strip the received CRC_A
 *   RFID_HF_NO_PARITY  suppress the ISO14443-A parity bit (anticollision frames)
 * Timeout is in microseconds (0 = the driver default). */
#define RFID_HF_CRC_TX     (1u << 0)
#define RFID_HF_CRC_RX     (1u << 1)
#define RFID_HF_NO_PARITY  (1u << 2)

#define RFID_ERR_TIMEOUT   (-1)   /* no response within the timeout (no tag) */
#define RFID_ERR_FRAMING   (-2)   /* parity / protocol / framing error */
#define RFID_ERR_CRC       (-3)   /* CRC_A mismatch on RX */
#define RFID_ERR_COLLISION (-4)   /* bit collision (>1 tag) */
#define RFID_ERR_OVERFLOW  (-5)   /* response exceeded rx_cap */
#define RFID_ERR_UNSUPP    (-6)   /* device can't do this */

int hal_rfid_hf_transceive(const uint8_t *tx, int tx_bits, uint32_t flags,
                           uint8_t *rx, int rx_cap, uint32_t timeout_us);

/* Custom-parity transceive for encrypted ISO14443-A frames (MIFARE Crypto1): send `nbytes` from `tx`
 * with the caller-supplied parity bit per byte (par[i] for byte i), no CRC appended; receive the tag
 * answer into `rx` (up to rx_cap bytes). If `rx_par` is non-NULL it receives the tag's per-byte parity
 * bit (one byte per RX byte, same indexing as `rx`, >= rx_cap) - the nested-nonce attack matches these
 * keystream-encrypted bits; pass NULL to ignore them. Weak-defaults to unsupported on frontends that
 * can't supply per-byte parity. Returns received data bits, or a negative RFID_ERR_*. */
int hal_rfid_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                               uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us);

/* Low-level probe: read a driver-defined identity/version word from the HF
 * frontend (MFRC522 VersionReg, ST25R3916 IC_IDENTITY, ...) for reader probing
 * and bring-up. Returns the value, or -1 if unsupported / no chip. */
int hal_rfid_hf_probe(void);

/* ---- HF ISO14443-A tag emulation (card side; weak-defaults to unsupported) ----
 * The real-time bit I/O + FDT counterpart of hf_transceive, for emulating a 14443-A tag. After
 * set_mode(RFID_HF_EMU): hf_emu_recv Miller-decodes one reader command into rx[]/rx_par[] (returns the byte
 * count, 0 on timeout/idle, <0 on error); hf_emu_send load-modulates a module-encoded reply (`tosend` = one
 * byte per subcarrier symbol) back at the ISO14443-A frame delay time (the FPGA enforces the 1172-cycle FDT;
 * the correction bit for 1236 comes from the last recv's parity). All protocol logic (anticollision, Crypto1,
 * response encoding, card image) lives in the calling module. Weak-unsupported where the frontend can't
 * load-modulate as a tag. */
int hal_rfid_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms);
int hal_rfid_hf_emu_send(const uint8_t *tosend, int len);
/* Streaming tag reply: fetch each subcarrier symbol from next(ctx) and feed it at the FDT, so a module can
 * encrypt+encode a crypto reply bit-by-bit during the paced feed (low FDT even for long encrypted answers).
 * Frontends that can overlap the producer with TX (PM3 FPGA feed, Chameleon NFCT DMA) implement this. */
int hal_rfid_hf_emu_send_stream(uint8_t (*next)(void *ctx), void *ctx, int nsymbols);
/* Buffer form of the same reply for a frontend that can't overlap (CPU-bit-banged TX): the module pre-encodes
 * the whole reply (iso14a_tag_encode buffer) and passes it here, never pointer-cached. Used by the module only
 * when hf_emu_send_stream returns <0. Weak-defaults to unsupported. */
int hal_rfid_hf_emu_send_stream_buf(const uint8_t *tosend, int len);
/* Incremental receive variant. The data callback runs in caller task context;
 * bits identifies a seven-bit prefix or a complete byte. A standard frame
 * reports index zero twice (first at seven bits, then as the complete byte).
 * Data remains tentative until the function returns a positive frame length.
 * The callback may stage a deferred reply with an emulation-send function but
 * must not block or call other RFID HAL functions. */
int hal_rfid_hf_emu_recv_progress(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms,
                                  void (*data_ready)(void *ctx, int index, uint8_t raw, int bits), void *ctx);
/* Optionally prepare a static encoded reply before a receive/reply window. */
int hal_rfid_hf_emu_prepare(const uint8_t *tosend, int len);
/* Stage a streamed reply from an incremental-receive callback, committing it
 * only when the completed frame exactly matches request. */
int hal_rfid_hf_emu_send_stream_match(uint8_t (*next)(void *ctx), void *ctx, int nsymbols,
                                      const uint8_t *request, int request_len, int request_min_len,
                                      int *result);

/* ---- HF sniffer (passive listen; hardware-specific, weak-defaults to unsupported) ----
 * Passively capture a live 13.56 MHz reader<->card exchange without generating a field
 * (e.g. Flipper ST25R3916 transparent mode: envelope demod on MISO, oversampled). Writes a
 * capture/summary to `dump_path`; waits up to timeout_ms for the field. Returns the sample
 * count, or <0. Only platforms that can passively demodulate implement it. */
int hal_rfid_hf_sniff(const char *dump_path, uint32_t timeout_ms);

/* Caller-buffer variant: capture into `buf` (>= FANTASI_RFID_SNIFF_BUFSZ) instead of a file, so a
 * sniff module can own an ephemeral buffer and print the trace directly - the frontend keeps no
 * static sniff buffer. Leaves the decoded text (L-header + frame lines) at buf[0..return]; returns
 * the text byte length, 0 if idle, or <0. quiet_ms ends a capture after that gap with no new frame;
 * max_ms hard-caps one active capture. Weak-defaults to unsupported. */
int hal_rfid_hf_sniff_capture(uint8_t *buf, uint32_t cap, uint32_t quiet_ms, uint32_t max_ms);

/* ---- LF reader (phase 2 - declared for the contract) ---- */
int hal_rfid_lf_field(bool on, uint32_t divisor);
int hal_rfid_lf_acquire(uint8_t *buf, int max, uint32_t opts);
int hal_rfid_lf_modulate(const uint8_t *pattern, int nbits, uint32_t opts);
int hal_rfid_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap);

#endif /* FANTASI_HAL_RFID_H */
