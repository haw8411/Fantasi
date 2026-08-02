#include "rfid.h"
#include "../../hal/hal_rfid.h"
#include "FreeRTOS.h"                 /* pvPortMalloc/vPortFree - the portable allocator (as in core/vfs.c) */

#include <string.h>

/* EM4100 (EM410x) decode over a stream of inter-edge intervals (carrier-cycle
 * counts from hal_rfid_lf_acquire). Ported once from the ChameleonUltra
 * manchester + em410x decoders (utils/manchester.c, protocols/em410x.c):
 *
 *   interval --period--> 1T/1.5T/2T class --manchester--> bit(s)
 *   bits --> 64-bit sliding frame: 9x '1' header, 10 rows of {4 data + odd row
 *   parity}, 4 column-parity bits (even), stop bit '0' -> 40-bit (5-byte) UID.
 *
 * The tag's bit clock is RF/64, /32, or /16; we try each (divisor 1/2/4). */

/* Interval classification: 1T~64, 1.5T~96, 2T~128 carrier cycles (+-16 jitter),
 * scaled by the bit-rate divisor. Returns 0/1/2 for 1T/1.5T/2T, or 3 (invalid). */
static uint8_t lf_period(uint8_t divisor, uint8_t iv)
{
    if (iv >= (uint8_t)((64 - 16) / divisor)  && iv <= (uint8_t)((64 + 16) / divisor))  return 0;
    if (iv >= (uint8_t)((96 - 16) / divisor)  && iv <= (uint8_t)((96 + 16) / divisor))  return 1;
    if (iv >= (uint8_t)((128 - 16) / divisor) && iv <= (uint8_t)((128 + 16) / divisor)) return 2;
    return 3;
}

static int popcount8(uint8_t x) { int c = 0; while (x) { c++; x &= (uint8_t)(x - 1); } return c; }

/* Validate a 64-bit frame and, if good, extract the 5-byte UID. */
static int frame_ok(uint64_t raw, uint8_t uid[5])
{
    if (((raw >> 55) & 0x1FF) != 0x1FF) return 0;   /* 9-bit '1' header */
    if (raw & 1) return 0;                          /* stop bit must be 0 */

    uint8_t pc = 0, out[5] = {0};
    for (int i = 0; i < 11; i++) {
        uint8_t row  = (uint8_t)((raw >> (64 - 9 - (i + 1) * 5)) & 0x1F);
        uint8_t data = (uint8_t)((row >> 1) & 0x0F);
        pc ^= data;
        if (i == 10) break;                         /* i==10 is the column-parity group */
        if (popcount8(row) & 1) return 0;           /* each row is even parity (EM4100 datasheet: P2..P9 even) */
        if (i & 1) out[i >> 1] |= data;
        else       out[i >> 1]  = (uint8_t)(data << 4);
    }
    if (pc != 0) return 0;                           /* even column parity */
    memcpy(uid, out, 5);
    return 1;
}

int rfid_em4100_decode(const uint8_t *iv, int n, uint8_t uid[5])
{
    static const uint8_t divisors[3] = { 1, 2, 4 };
    static const int      rates[3]   = { 64, 32, 16 };

    /* `inv` covers the two Manchester conventions (bit 1 as high-then-low or low-then-high). `start` only
     * picks the phase, so without this the decoder is polarity-bound: whether a tag's 1 bits arrive as the
     * first or second half depends on the reader's analog inversion. Trying both costs one
     * more pass and cannot produce false positives - a frame still has to satisfy the 9-bit header, all ten
     * row parities, the column parity and the stop bit. */
    for (int d = 0; d < 3; d++)
    for (int start = 0; start < 2; start++)     /* both Manchester phases */
    for (int inv = 0; inv < 2; inv++) {         /* both Manchester conventions */
        int      sync = start, rawlen = 0;
        uint64_t raw = 0;

        for (int k = 0; k < n; k++) {
            uint8_t t = lf_period(divisors[d], iv[k]);
            uint8_t bits[2];
            int bl = 0;

            /* Manchester state machine (sync/non-sync), mirrors manchester.c.
             * On an invalid interval reset only the bit accumulator - not sync;
             * the phase self-corrects on the next 1.5T, and forcing sync here
             * would mis-read the following 1T run (e.g. the 9-bit '1' header). */
            if (t == 3) { raw = 0; rawlen = 0; continue; }
            if (sync) {
                if (t == 0)      { bits[0] = 0; bl = 1; }
                else if (t == 1) { bits[0] = 1; bl = 1; sync = 0; }
                else             { bits[0] = 1; bits[1] = 0; bl = 2; }   /* t==2 */
            } else {
                if (t == 0)      { bits[0] = 1; bl = 1; }
                else if (t == 1) { bits[0] = 1; bits[1] = 0; bl = 2; sync = 1; }
                else             { raw = 0; rawlen = 0; continue; }       /* t==2: invalid */
            }

            for (int b = 0; b < bl; b++) {
                raw = (raw << 1) | (uint64_t)(bits[b] ^ inv);
                if (++rawlen < 64) continue;
                if (frame_ok(raw, uid)) return rates[d];
            }
        }
    }
    return RFID_ERR_TIMEOUT;   /* no valid frame in the capture */
}

/* Self-contained EM4100 read: enter LF reader mode, energise, capture, decode,
 * and power down. Returns the bit rate (64/32/16) + fills uid, or <0. */
int rfid_lf_em4100_read(uint8_t uid[5])
{
    if (hal_rfid_set_mode(RFID_LF_READER) != 0) return RFID_ERR_UNSUPP;

    /* Interval buffer on the heap only for this read - the EM4100 protocol shouldn't pin idle BSS (600 B
     * is also too big for the stack). Portable: pvPortMalloc is provided on every target (see core/vfs.c). */
    enum { IV_CAP = 600 };
    uint8_t *iv = pvPortMalloc(IV_CAP);
    if (!iv) { hal_rfid_set_mode(RFID_OFF); return RFID_ERR_UNSUPP; }
    hal_rfid_lf_field(true, 0);

    /* Re-capture a few times without dropping the field: a cold capture can come
     * back short (weak coupling / still-settling frontend), and the tag frame
     * repeats continuously, so another window usually lands a clean one. Each
     * acquire streams ~3 frames (~100 ms), so 3 attempts is ~9 frames of
     * chances inside a ~300 ms budget,
     * and each one long enough to contain a clean 64-bit run on its own.
     *
     * Five windows: a capture spans only ~2 frames (an EM4100 frame is 64 bits x 64 carrier
     * cycles), and the decoder resets its 64-bit accumulator on any interval it cannot classify, so one
     * glitch can cost a whole window. The extra tries are ~80 ms each and only run until one decodes. */
    int rc = RFID_ERR_TIMEOUT;
    for (int attempt = 0; attempt < 5; attempt++) {
        int n = hal_rfid_lf_acquire(iv, IV_CAP, 0);
        if (n >= 64) {                      /* need at least ~one frame of edges */
            rc = rfid_em4100_decode(iv, n, uid);
            if (rc > 0) break;
        }
    }

    hal_rfid_lf_field(false, 0);
    hal_rfid_set_mode(RFID_OFF);
    vPortFree(iv);
    return rc;
}
