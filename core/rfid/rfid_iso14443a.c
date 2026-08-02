#include "rfid.h"
#include "../../hal/hal_rfid.h"

#include <string.h>

/* ISO14443-A activation: REQA -> per-cascade { anticollision, SELECT } -> SAK,
 * assembling the 4/7/10-byte UID. This is the same state machine the three stock
 * firmwares implement at three different register levels; here it is written
 * once over hal_rfid_hf_transceive(). Modelled on the ChameleonUltra
 * pcd_14a_reader_scan_once() flow. */

#define PICC_REQA        0x26   /* 7-bit short frame */
#define PICC_ANTICOLL_1  0x93   /* + level*2 -> 0x93/0x95/0x97 */

int rfid_iso14443a_select(rfid_iso14443a_tag_t *tag)
{
    if (!tag) return RFID_ERR_FRAMING;
    memset(tag, 0, sizeof(*tag));

    uint8_t rx[16];

    /* REQA: 7-bit short frame. Hardware parity stays ON so the MFRC522 strips the
     * ATQA parity bits and returns a clean 16-bit ATQA (with parity OFF the chip
     * hands back data+parity interleaved - 18 bits - which we'd have to
     * de-interleave). A fresh field often drops the first REQA while the card
     * powers up, so retry a few times before declaring no-tag. */
    uint8_t reqa = PICC_REQA;
    int n = RFID_ERR_TIMEOUT;
    for (int try = 0; try < 4; try++) {
        n = hal_rfid_hf_transceive(&reqa, 7, 0, rx, sizeof(rx), 0);
        if (n == 16) break;
    }
    if (n == RFID_ERR_TIMEOUT) return n;   /* no tag */
    if (n != 16) return RFID_ERR_FRAMING;
    tag->atqa[0] = rx[0];
    tag->atqa[1] = rx[1];

    for (int level = 0; level < 3; level++) {
        uint8_t sel = (uint8_t)(PICC_ANTICOLL_1 + level * 2);

        /* Anticollision: SEL + NVB(0x20). Tag returns 4 UID bytes + BCC. */
        uint8_t anticoll[2] = { sel, 0x20 };
        n = hal_rfid_hf_transceive(anticoll, 16, 0, rx, sizeof(rx), 0);
        if (n < 0) return n;
        if (n < 40) return RFID_ERR_FRAMING;

        uint8_t uidcl[5];
        memcpy(uidcl, rx, 5);            /* UID CLn (4) + BCC */
        uint8_t bcc = uidcl[0] ^ uidcl[1] ^ uidcl[2] ^ uidcl[3];
        if (bcc != uidcl[4]) return RFID_ERR_FRAMING;

        /* SELECT: SEL + NVB(0x70) + 4 UID + BCC + CRC_A. Tag returns SAK(+CRC). */
        uint8_t select[9] = { sel, 0x70,
                              uidcl[0], uidcl[1], uidcl[2], uidcl[3], uidcl[4], 0, 0 };
        rfid_crc_a(select, 7, &select[7]);
        n = hal_rfid_hf_transceive(select, 72, 0, rx, sizeof(rx), 0);
        if (n < 0) return n;
        if (n < 8) return RFID_ERR_FRAMING;
        tag->sak = rx[0];
        tag->cascade = (uint8_t)(level + 1);

        if (tag->sak & 0x04) {
            /* Cascade: uidcl[0] is the Cascade Tag (0x88); the real UID bytes are
             * uidcl[1..3]. Append 3 bytes and continue to the next level. */
            memcpy(tag->uid + level * 3, &uidcl[1], 3);
            tag->uid_len += 3;
        } else {
            /* Final level: all 4 bytes are UID. */
            memcpy(tag->uid + level * 3, uidcl, 4);
            tag->uid_len += 4;
            return 0;
        }
    }
    return RFID_ERR_FRAMING;   /* ran past cascade level 3 without completing */
}

const char *rfid_iso14443a_type(uint8_t sak, const uint8_t atqa[2])
{
    (void)atqa;
    /* Coarse SAK-based classification (enough for a scan listing; the app does
     * finer per-vendor identification). */
    switch (sak & 0x7F) {
    case 0x00: return "MIFARE Ultralight / NTAG";
    case 0x08: return "MIFARE Classic 1K";
    case 0x09: return "MIFARE Mini";
    case 0x18: return "MIFARE Classic 4K";
    case 0x10: return "MIFARE Plus 2K";
    case 0x11: return "MIFARE Plus 4K";
    case 0x20: return "ISO14443-4 (DESFire/Plus/JCOP)";
    case 0x28: return "SmartMX (JCOP)";
    default:   return "ISO14443-A";
    }
}
