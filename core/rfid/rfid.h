#ifndef FANTASI_CORE_RFID_H
#define FANTASI_CORE_RFID_H

#include <stdint.h>

/* Portable RFID protocol layer, written once over the hal_rfid.h primitives and
 * compiled into flash on RFID-capable targets (FANTASI_ENABLE_RFID). Holds the
 * logic that is identical across frontends - ISO14443-A anticollision, CRC_A,
 * (later) EM4100 decode - so each platform HAL only provides the transceive /
 * acquire primitives. See docs/rfid.md. */

/* ISO14443-A CRC ("CRC_A", poly 0x8408, preset 0x6363). Writes 2 bytes
 * (LSB, MSB) to crc_out. */
void rfid_crc_a(const uint8_t *data, int len, uint8_t *crc_out);

/* A card found by the 14443-A anticollision + select sequence. */
typedef struct {
    uint8_t uid[10];   /* 4, 7, or 10 bytes */
    uint8_t uid_len;
    uint8_t sak;       /* SAK of the final cascade level */
    uint8_t atqa[2];   /* ATQA (little-endian as received) */
    uint8_t cascade;   /* number of cascade levels (1..3) */
} rfid_iso14443a_tag_t;

/* Run REQA -> anticollision cascade -> SELECT and fill *tag. Requires the HAL
 * already in RFID_HF_READER mode with the field on. Returns 0 on success, or a
 * negative RFID_ERR_* (RFID_ERR_TIMEOUT == no tag in field). */
int rfid_iso14443a_select(rfid_iso14443a_tag_t *tag);

/* Best-effort human label for a (SAK, ATQA) pair, e.g. "MIFARE Classic 1K".
 * Returns a static string ("ISO14443-A" when unknown). */
const char *rfid_iso14443a_type(uint8_t sak, const uint8_t atqa[2]);

/* Decode an EM4100 (EM410x) tag from a buffer of `n` inter-edge intervals
 * (carrier-cycle counts from hal_rfid_lf_acquire), writing the 5-byte UID.
 * Returns the detected bit rate (64/32/16 = RF/n) on success, or a negative
 * RFID_ERR_* (RFID_ERR_TIMEOUT == no valid frame in the capture). */
int rfid_em4100_decode(const uint8_t *intervals, int n, uint8_t uid[5]);

/* Self-contained EM4100 read: enters LF reader mode, energises the field,
 * captures, decodes, and powers down. Returns the bit rate (64/32/16) and fills
 * the 5-byte UID, or a negative RFID_ERR_* (RFID_ERR_TIMEOUT == no tag). */
int rfid_lf_em4100_read(uint8_t uid[5]);

#endif /* FANTASI_CORE_RFID_H */
