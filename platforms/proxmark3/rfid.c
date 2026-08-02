/* Proxmark3 (AT91SAM7S512 + Xilinx Spartan-II XC2S30) RFID HAL.
 *
 * Unlike the Chameleon/Flipper, the RF front-end is an FPGA: the ARM only
 * bit-bangs a bitstream into it, then talks to it over two channels - a 16-bit
 * SPI "config register" (major mode + minor mode + divisor) and an SSC byte
 * stream carrying samples. This file owns both readers:
 *
 *   - PCK0 = 24 MHz on PA6 feeds the FPGA its master clock.
 *   - DownloadFPGA() slave-serial bit-bangs section 'e' of fpga_pm3_{lf,hf}.bit
 *     (embedded raw) through PROGRAM/CCLK/DIN, waiting on INIT_B then DONE.
 *   - FpgaSendCommand() pushes 16-bit words to the config register over SPI0.
 *   - LF (125 kHz): the SSC receives one 8-bit envelope sample per carrier cycle,
 *     so a sample-index delta is a carrier-cycle count - the portable EM4100
 *     decoder in core/rfid/ classifies it as 1T/1.5T/2T.
 *   - HF (13.56 MHz ISO14443-A): the FPGA's hi_iso14443a sub-module does the
 *     analog; this file Miller-encodes the reader frame out the SSC (READER_MOD)
 *     and Manchester-decodes the tag's subcarrier answer back in (READER_LISTEN),
 *     driving the portable core/rfid/ 14443-A anticollision over hf_transceive.
 *
 * hal_rfid_caps() advertises HF_READ | LF_READ. */

#include "at91sam7s512.h"
#include "../../hal/hal_rfid.h"
#include "../../apps/app_rfid.h"   /* FANTASI_RFID_SNIFF_BUFSZ - shared caller/HAL scratch size */
#include "../../core/vfs.h"
#include "../../core/lzss.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* ---- PIOA pin masks ---------------------------------------------------- */
#define PA(n)          (1u << (n))
#define FPGA_NPROGRAM  PA(28)   /* config: pulse low to (re)start config      */
#define FPGA_NINIT     PA(4)    /* config: FPGA drives high when ready (in)    */
#define FPGA_DONE      PA(27)   /* config: FPGA drives high when configured(in)*/
#define FPGA_CCLK      PA(29)   /* config serial clock                         */
#define FPGA_DIN       PA(30)   /* config serial data                          */
#define FPGA_ON        PA(26)   /* FPGA power enable (active high)             */
#define PCK0_PIN       PA(6)    /* programmable clock 0 -> FPGA master clock   */
#define SPI_NCS0       PA(11)   /* SPI0 chip-select 0 -> FPGA config register  */
#define SPI_MISO       PA(12)
#define SPI_MOSI       PA(13)
#define SPI_SPCK       PA(14)
#define SSC_TF         PA(15)
#define SSC_TK         PA(16)
#define SSC_TD         PA(17)
#define SSC_RD         PA(18)   /* SSC receive data <- FPGA sample stream      */
#define MUX_HIPKD      PA(19)   /* ADC source mux (one-hot select)             */
#define MUX_LOPKD      PA(20)   /* LF peak-detected (envelope) path            */
#define MUX_HIRAW      PA(21)
#define MUX_LORAW      PA(22)

#define HI(x)    (AT91C_BASE_PIOA->PIO_SODR = (x))
#define LO(x)    (AT91C_BASE_PIOA->PIO_CODR = (x))
#define PIN(x)   (AT91C_BASE_PIOA->PIO_PDSR & (x))

/* ---- FPGA config-register command words -------------------------------- */
#define FPGA_CMD_SET_CONFREG          (1u << 12)
#define FPGA_CMD_SET_DIVISOR          (2u << 12)
#define FPGA_MAJOR_MODE_LF_READER     (0u << 6)
#define FPGA_MAJOR_MODE_HF_ISO14443A  (2u << 6)   /* the 14443-A reader sub-module */
#define FPGA_MAJOR_MODE_OFF           (7u << 6)
#define FPGA_LF_ADC_READER_FIELD      0x1        /* conf bit: energise LF antenna */
#define FPGA_HF_14A_READER_MOD        4          /* HF minor mode: transmit (OOK)  */
#define FPGA_HF_14A_READER_LISTEN     3          /* HF minor mode: receive         */
#define FPGA_HF_14A_TAGSIM_LISTEN     1          /* HF minor mode: TAG rx (Miller) */
#define FPGA_HF_14A_TAGSIM_MOD        2          /* HF minor mode: TAG tx (subcar) */
#define FPGA_HF_14A_SNIFFER           0          /* HF minor mode: passive sniff   */
#define LF_DIVISOR_125                95         /* 12 MHz / (95+1) = 125 kHz    */

/* Reader->tag modified-Miller symbols, one SEC byte clocked out per air-bit
 * (a '1' bit = an OOK carrier pause; the FPGA drops the coil while ssp_dout=1). */
#define SEC_X 0x0c   /* pause in 2nd quarter -> logic '1'                    */
#define SEC_Y 0x00   /* no pause -> '0' after a '1', idle, end-of-comm       */
#define SEC_Z 0xc0   /* pause at start -> '0' after '0'/start, start-of-comm */

/* The FPGA bitstreams are not embedded in flash. They live compressed under
 * /fpga in LittleFS (tools/fpga_lzss.py format), provisioned from the host on
 * first use and loaded from there afterwards - so 4+ protocols cost no app-region
 * flash and the user can reclaim the space by deleting /fpga. One bitstream drives
 * the 125 kHz LF reader, another the 13.56 MHz HF reader; the loader is common. */
#define FPGA_LF_PATH  "/fpga/pm3_lf.bit.z"
#define FPGA_HF_PATH  "/fpga/pm3_hf.bit.z"

/* LF envelope acquisition is streamed: the demod runs inline on the SSC sample flow and only
 * the inter-edge intervals are stored (into the caller's buffer), so no envelope buffer exists
 * at all. Streaming gives:
 *
 *   - no sample buffer (1500 B off the in-session heap, on top of the ~6 KB already reclaimed
 *     from static reservations by making the sniff scratch and FPGA window ephemeral),
 *   - no decimation, so edge intervals keep full 1-carrier-cycle resolution,
 *   - coverage is bounded by time instead of RAM, so we take LF_SPAN = 3 full EM4100 frames.
 *
 * Coverage is what actually drives reliability here: the decoder slides a 64-bit window, so it
 * needs 64 consecutive cleanly-decoded bits. 3 frames gives ~192 bit-periods
 * for a clean 64-bit run to land in. Per sample there are ~380 CPU cycles at 8 us/sample, and
 * the inline demod is a few adds/shifts, so streaming keeps up comfortably. */
#define LF_SPAN     12288    /* raw carrier cycles streamed per acquire = 3 EM4100 frames (~98 ms) */
#define LF_WARMUP   256      /* cycles to settle dc/amp trackers before arming edge detection */
#define LF_SMOOTH   4        /* boxcar taps - denoises like the old decimation, but constant group
                              * delay, so it cancels out of interval differences (no resolution lost) */

/* Ephemeral FPGA-config scratch: the 4 KB LZSS history window the decoder needs,
 * plus a small staging buffer for compressed reads (fpga_zsrc chunks the file). */
#define FPGA_LZSS_WINDOW 4096
#define FPGA_LZSS_STAGE  512

static rfid_mode_t s_mode;

/* Which bitstream the FPGA currently holds, so set_mode skips a redundant reload
 * when consecutive operations stay in one protocol family (e.g. a MIFARE dump is
 * all in pm3_hf). Any public fpga_load resets this to RES_NONE. */
static enum { RES_NONE, RES_LF, RES_HF } s_loaded;
/* Set on set_mode(OFF): forces the next reader-mode entry to re-download the bitstream even if the same
 * family is still "loaded". A config-word switch alone does not re-init the sniffer demod cleanly on an
 * FPGA that has been running - the second sniff session then sees carrier but no modulation (rm=0). A
 * MIFARE-style OFF/ON toggle within one session pays a ~90 ms reload, which is fine for interactive use. */
static bool s_stale;

static void spin_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }

/* Busy-wait `us` microseconds off the PIT counter (CPIV runs at MCK/16 ~= 3.003 MHz, so 3
 * ticks/us), accurate and calibration-free. Reads PITC_PIIR - the IMAGE register, so it does
 * not clear the tick IRQ. Each call is far under one 1 ms PIT period, so at most one CPIV wrap.
 * Safe with IRQs masked (the PIT peripheral keeps counting); used for the T5577 downlink gaps. */
static void spin_us(uint32_t us)
{
    uint32_t piv  = AT91C_BASE_PITC->PITC_PIMR & 0xFFFFFu;   /* PIV = the period CPIV wraps at */
    uint32_t last = AT91C_BASE_PITC->PITC_PIIR & 0xFFFFFu;
    for (uint32_t want = us * 3, seen = 0; seen < want; ) {
        uint32_t now = AT91C_BASE_PITC->PITC_PIIR & 0xFFFFFu;
        seen += (now >= last) ? (now - last) : (now + piv - last);   /* handle the CPIV wrap */
        last = now;
    }
}

/* PCK0 = PLL/4 = 24 MHz out on PA6 (peripheral B) - the FPGA's master clock. */
static void fpga_clock_on(void)
{
    AT91C_BASE_PIOA->PIO_BSR = PCK0_PIN;
    AT91C_BASE_PIOA->PIO_PDR = PCK0_PIN;
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_PCK0;
    AT91C_BASE_PMC->PMC_PCKR[0] = AT91C_PMC_CSS_PLL_CLK | AT91C_PMC_PRES_CLK_4;
    AT91C_BASE_PIOA->PIO_OER = PCK0_PIN;
}

/* Clock one configuration byte into the FPGA, MSB first (the lzss_decode sink). */
static void fpga_din_byte(void *ctx, uint8_t w)
{
    (void)ctx;
    for (int bit = 7; bit >= 0; bit--) {
        if (w & (1u << bit)) HI(FPGA_DIN); else LO(FPGA_DIN);
        HI(FPGA_CCLK);
        LO(FPGA_CCLK);
    }
}

/* Pull compressed bytes from the .bit.z file (the lzss_decode source). Each
 * vfs_pread re-opens the lfs file, so we stage reads through `buf` (a small tail
 * of the ephemeral config scratch, past the 4 KB decode window) and hand them out
 * in the small pieces lzss_decode asks for - fewer opens than a byte-at-a-time
 * source. Backend-agnostic; never buffers the whole payload. */
typedef struct {
    const char *path;
    uint32_t    off;            /* next file offset to fetch */
    uint8_t    *buf;            /* staging buffer (config-scratch tail) */
    int         cap, len, pos;  /* capacity / filled / consumed */
} fpga_zsrc_t;
static int fpga_zsrc(void *ctx, uint8_t *out, int max)
{
    fpga_zsrc_t *s = (fpga_zsrc_t *)ctx;
    if (s->pos >= s->len) {
        int32_t n = vfs_pread(s->path, s->off, s->buf, (uint32_t)s->cap);
        if (n <= 0) return 0;
        s->off += (uint32_t)n; s->len = (int)n; s->pos = 0;
    }
    int give = s->len - s->pos;
    if (give > max) give = max;
    for (int i = 0; i < give; i++) out[i] = s->buf[s->pos++];
    return give;
}

/* Slave-serial download of a compressed bitstream (fpga_lzss.py format) from a
 * VFS path: [u32 LE decompressed length][LZSS stream]. Streams straight from
 * flash through the decoder into the FPGA over an ephemeral 4 KB LZSS window
 * (allocated + freed here). Returns 0 once DONE goes high, -1 if the file is
 * missing / short, the window won't allocate, or the FPGA never asserts DONE. */
static int fpga_load(const char *path)
{
    uint8_t hdr[4];
    if (vfs_pread(path, 0, hdr, 4) != 4) return -1;      /* not provisioned */
    uint32_t elen = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                    ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
    if (elen == 0 || elen > (1u << 20)) return -1;

    fpga_clock_on();

    /* Power the FPGA and let its rail settle. */
    AT91C_BASE_PIOA->PIO_PER = FPGA_ON;
    AT91C_BASE_PIOA->PIO_OER = FPGA_ON;
    HI(FPGA_ON);
    spin_ms(50);

    /* INIT_B and DONE are FPGA outputs we sample; pull them up. */
    AT91C_BASE_PIOA->PIO_ODR   = FPGA_NINIT | FPGA_DONE;
    AT91C_BASE_PIOA->PIO_PER   = FPGA_NINIT | FPGA_DONE;
    AT91C_BASE_PIOA->PIO_PPUER = FPGA_NINIT | FPGA_DONE;

    /* PROGRAM idle high, CCLK/DIN low, all three driven. */
    HI(FPGA_NPROGRAM);
    LO(FPGA_CCLK | FPGA_DIN);
    AT91C_BASE_PIOA->PIO_PER = FPGA_NPROGRAM | FPGA_CCLK | FPGA_DIN;
    AT91C_BASE_PIOA->PIO_OER = FPGA_NPROGRAM | FPGA_CCLK | FPGA_DIN;

    /* Pulse PROGRAM low to clear the config, then release. */
    LO(FPGA_NPROGRAM);
    spin_ms(50);
    HI(FPGA_NPROGRAM);

    /* Wait for INIT_B high: the FPGA has cleared and is ready for data. */
    for (volatile uint32_t t = 0; t < 2000000u && !PIN(FPGA_NINIT); t++) { }

    /* Decompress the configuration frames straight into DIN/CCLK. The decoder needs a 4 KB LZSS
     * history window plus a small staging buffer for the compressed reads - allocated ephemerally
     * for the ~90 ms of a config load and freed after, so the frontend keeps no static buffer
     * reserved between loads (config is rare: cold start or after set_mode(OFF)). */
    uint8_t *win = pvPortMalloc(FPGA_LZSS_WINDOW + FPGA_LZSS_STAGE);
    if (!win) return -1;
    fpga_zsrc_t zs = { path, 4, win + FPGA_LZSS_WINDOW, FPGA_LZSS_STAGE, 0, 0 };
    int rc = lzss_decode(elen, fpga_zsrc, &zs, fpga_din_byte, NULL, win, FPGA_LZSS_WINDOW);
    vPortFree(win);
    if (rc != 0) return -1;

    /* A few hundred extra clocks let the FPGA assert DONE and start up. */
    for (uint32_t t = 0; t < 500000u && !PIN(FPGA_DONE); t++) {
        HI(FPGA_CCLK);
        LO(FPGA_CCLK);
    }
    return PIN(FPGA_DONE) ? 0 : -1;
}

/* ---- SPI0: the 16-bit FPGA config register ----------------------------- */
static void fpga_spi_setup(void)
{
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_SPI);
    AT91C_BASE_PIOA->PIO_ASR = SPI_NCS0 | SPI_MISO | SPI_MOSI | SPI_SPCK;
    AT91C_BASE_PIOA->PIO_PDR = SPI_NCS0 | SPI_MISO | SPI_MOSI | SPI_SPCK;

    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SWRST;
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SWRST;
    AT91C_BASE_SPI->SPI_MR = 0x000E0011;        /* master, fixed NPCS0, modfdis */
    AT91C_BASE_SPI->SPI_CSR[0] = 0x01010682;    /* 16-bit, mode 0, MCK/6 (8 MHz) */
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_SPIEN;
}

static void fpga_send_cmd(uint16_t w)
{
    uint32_t t = 0;
    while (!(AT91C_BASE_SPI->SPI_SR & AT91C_SPI_TXEMPTY) && ++t < 200000u) { }
    AT91C_BASE_SPI->SPI_TDR = AT91C_SPI_LASTXFER | w;
    t = 0;
    while (!(AT91C_BASE_SPI->SPI_SR & AT91C_SPI_RDRF) && ++t < 200000u) { }
    (void)AT91C_BASE_SPI->SPI_RDR;
}

static void fpga_conf(uint16_t conf)   { fpga_send_cmd(FPGA_CMD_SET_CONFREG | conf); }
static void fpga_divisor(uint8_t d)    { fpga_send_cmd(FPGA_CMD_SET_DIVISOR | d); }

/* ---- SSC: receive the FPGA's 8-bit LF sample stream -------------------- */
static void fpga_ssc_setup(void)
{
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_SSC);
    AT91C_BASE_PIOA->PIO_ASR = SSC_TF | SSC_TK | SSC_TD | SSC_RD;
    AT91C_BASE_PIOA->PIO_PDR = SSC_TF | SSC_TK | SSC_TD | SSC_RD;

    AT91C_BASE_SSC->SSC_CR = AT91C_SSC_SWRST;
    AT91C_BASE_SSC->SSC_RCMR = 0x00000101;      /* RK clock, continuous receive */
    AT91C_BASE_SSC->SSC_RFMR = 0x00000087;      /* 8 data bits, MSB first        */
    AT91C_BASE_SSC->SSC_TCMR = 0x00000502;
    AT91C_BASE_SSC->SSC_TFMR = 0x00000087;
    AT91C_BASE_SSC->SSC_CR = AT91C_SSC_RXEN | AT91C_SSC_TXEN;
}

/* Route one analog line to the ADC (one-hot): LOPKD is the LF envelope, HIPKD
 * the HF peak-detected demod line. */
static void adc_mux(uint32_t pin)
{
    uint32_t all = MUX_HIPKD | MUX_LOPKD | MUX_HIRAW | MUX_LORAW;
    AT91C_BASE_PIOA->PIO_PER = all;
    AT91C_BASE_PIOA->PIO_OER = all;
    AT91C_BASE_PIOA->PIO_CODR = all;
    AT91C_BASE_PIOA->PIO_SODR = pin;
}

/* Energise / de-energise the LF carrier via the FPGA major mode. */
static void lf_energize(bool on)
{
    if (s_mode != RFID_LF_READER) return;
    if (on) {
        fpga_divisor(LF_DIVISOR_125);
        fpga_conf(FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD);
        fpga_ssc_setup();
        spin_ms(50);            /* antenna + envelope settle before sampling */
    } else {
        fpga_conf(FPGA_MAJOR_MODE_OFF);
    }
}

/* ======================================================================== *
 *  HF (13.56 MHz) ISO14443-A reader - transceive over the FPGA
 * ------------------------------------------------------------------------ *
 * The FPGA's hi_iso14443a sub-module does the analog only: in READER_MOD it
 * OOK-pauses the carrier per the SSC-TX byte stream (modified Miller), in
 * READER_LISTEN it streams back the demodulated subcarrier envelope (one SSC
 * byte = 8 samples = one 106 kbit/s bit period). The bit coding, parity and
 * Manchester decode live here, ported from the stock PM3 armsrc/iso14443a.c. */

/* ISO14443-A odd parity: the bit that makes {byte, parity} contain an odd
 * number of 1s, i.e. 1 when the byte itself has an even number of 1s. */
static inline uint8_t oddparity8(uint8_t x) { return (uint8_t)(!__builtin_parity(x)); }

/* Encode a reader frame as modified-Miller SEC bytes (one per air-bit) with a parity bit after each *full*
 * byte (a 7-bit short frame like REQA gets none). `par`, if non-NULL, supplies the parity bit per byte
 * (par[i] for byte i) - used for MIFARE Crypto1-encrypted frames where parity is keystream-encrypted, not
 * the byte's own odd parity; when NULL, the byte's odd parity is used. Returns the SEC-byte count, or -1. */
static int code_14a_reader_ex(const uint8_t *cmd, int bits, const uint8_t *par, uint8_t *ts, int cap)
{
    int last = 0, n = 0;
#define TS_PUSH(b) do { if (n >= cap) return -1; ts[n++] = (uint8_t)(b); } while (0)
    TS_PUSH(SEC_Z);                                 /* start of communication */
    int bytecount = (bits + 7) / 8;
    for (int i = 0; i < bytecount; i++) {
        uint8_t b = cmd[i];
        int bitsleft = bits - i * 8;
        if (bitsleft > 8) bitsleft = 8;
        int j;
        for (j = 0; j < bitsleft; j++) {            /* LSB first within the byte */
            if (b & 1)          { TS_PUSH(SEC_X); last = 1; }
            else if (last == 0) { TS_PUSH(SEC_Z); }
            else                { TS_PUSH(SEC_Y); last = 0; }
            b >>= 1;
        }
        if (j == 8) {                               /* parity only on a whole byte */
            uint8_t p = par ? (par[i] & 1) : oddparity8(cmd[i]);
            if (p)                  { TS_PUSH(SEC_X); last = 1; }
            else if (last == 0)     { TS_PUSH(SEC_Z); }
            else                    { TS_PUSH(SEC_Y); last = 0; }
        }
    }
    if (last == 0) TS_PUSH(SEC_Z);                   /* end of communication */
    else           TS_PUSH(SEC_Y);
    TS_PUSH(SEC_Y);
#undef TS_PUSH
    return n;
}
static int code_14a_reader(const uint8_t *cmd, int bits, uint8_t *ts, int cap)
{ return code_14a_reader_ex(cmd, bits, NULL, ts, cap); }

/* A nibble counts as subcarrier modulation if it has 3 or 4 set bits. */
static const uint8_t MANCH_LUT[16] = { 0,0,0,0,0,0,0,1, 0,0,0,1,0,1,1,1 };

static struct {
    int       state;        /* 0 = unsynced, 1 = manchester data */
    uint16_t  twoBits;      /* sliding 2-bit-period window */
    uint16_t  highCnt;
    uint16_t  bitCount;
    uint16_t  syncBit;
    uint16_t  shiftReg;
    uint16_t  len;          /* decoded data bytes */
    int       output_len;
    uint8_t  *output;
    uint8_t  *par;          /* optional per-byte received-parity out (1 byte/bit), or NULL */
    int       par_cap;
} s_demod;

static void demod_reset(uint8_t *out, int cap, uint8_t *par, int par_cap)
{
    s_demod.state = 0;
    s_demod.twoBits = 0xFFFF;
    s_demod.highCnt = 0;
    s_demod.bitCount = 0;
    s_demod.syncBit = 0xFFFF;
    s_demod.shiftReg = 0;
    s_demod.len = 0;
    s_demod.output = out;
    s_demod.output_len = cap;
    s_demod.par = par;
    s_demod.par_cap = par_cap;
}

/* Store the just-completed byte's 9th (parity) bit alongside data byte index `idx`. The bits shift in
 * LSB-first with the 9th landing at bit 8, so parity = (shiftReg >> 8) & 1. MIFARE Crypto1 keystream-
 * encrypts this bit, so the mfc module needs it (the nested-nonce attack matches it); plain frames ignore it. */
static inline void demod_store_par(int idx, uint16_t shiftReg)
{
    if (s_demod.par && idx < s_demod.par_cap) s_demod.par[idx] = (uint8_t)((shiftReg >> 8) & 1);
}

/* Feed one SSC RX byte. Syncs to the tag's bit phase, then decodes Sequence D
 * (subcarrier in 1st half = '1') / E (2nd half = '0') / F (none = end). The 9th
 * (parity) bit of each byte is discarded - the caller checks BCC/CRC. Returns 1
 * once a complete frame is decoded into s_demod.output. */
static int manchester_decode(uint8_t bit)
{
    if ((int)s_demod.len >= s_demod.output_len) return 1;
    s_demod.twoBits = (uint16_t)((s_demod.twoBits << 8) | bit);

    if (s_demod.state == 0) {                        /* UNSYNCD */
        if (s_demod.highCnt < 2) {                   /* wait for a stable idle line */
            if (s_demod.twoBits == 0x0000) s_demod.highCnt++;
            else s_demod.highCnt = 0;
            return 0;
        }
        uint16_t t = s_demod.twoBits, s = 0xFFFF;
        if      ((t & 0x7700) == 0x7000) s = 7;
        else if ((t & 0x3B80) == 0x3800) s = 6;
        else if ((t & 0x1DC0) == 0x1C00) s = 5;
        else if ((t & 0x0EE0) == 0x0E00) s = 4;
        else if ((t & 0x0770) == 0x0700) s = 3;
        else if ((t & 0x03B8) == 0x0380) s = 2;
        else if ((t & 0x01DC) == 0x01C0) s = 1;
        else if ((t & 0x00EE) == 0x00E0) s = 0;
        if (s != 0xFFFF) { s_demod.syncBit = s; s_demod.bitCount = 0; s_demod.state = 1; }
        return 0;
    }

    /* MANCHESTER_DATA */
    uint16_t w = (uint16_t)(s_demod.twoBits >> s_demod.syncBit);
    int mod1 = MANCH_LUT[(w & 0x00F0) >> 4];
    int mod2 = MANCH_LUT[w & 0x000F];
    if (mod1) {                                      /* 1st half -> '1' (also collision) */
        s_demod.bitCount++;
        s_demod.shiftReg = (uint16_t)((s_demod.shiftReg >> 1) | 0x100);
        if (s_demod.bitCount == 9) {
            demod_store_par(s_demod.len, s_demod.shiftReg);
            s_demod.output[s_demod.len++] = (uint8_t)(s_demod.shiftReg & 0xff);
            s_demod.bitCount = 0; s_demod.shiftReg = 0;
        }
    } else if (mod2) {                               /* 2nd half -> '0' */
        s_demod.bitCount++;
        s_demod.shiftReg = (uint16_t)(s_demod.shiftReg >> 1);
        if (s_demod.bitCount >= 9) {
            demod_store_par(s_demod.len, s_demod.shiftReg);
            s_demod.output[s_demod.len++] = (uint8_t)(s_demod.shiftReg & 0xff);
            s_demod.bitCount = 0; s_demod.shiftReg = 0;
        }
    } else {                                         /* no subcarrier -> end of comm */
        if (s_demod.bitCount > 0) {
            s_demod.shiftReg = (uint16_t)(s_demod.shiftReg >> (9 - s_demod.bitCount));
            demod_store_par(s_demod.len, 0);         /* partial byte: parity not received */
            s_demod.output[s_demod.len++] = (uint8_t)(s_demod.shiftReg & 0xff);
            return 1;
        }
        if (s_demod.len) return 1;
        demod_reset(s_demod.output, s_demod.output_len, s_demod.par, s_demod.par_cap);   /* nothing yet - keep waiting */
    }
    return 0;
}

/* Energise / de-energise the HF field (field is on in either reader minor mode). */
static void hf_energize(bool on)
{
    if (s_mode != RFID_HF_READER) return;
    fpga_conf(on ? (FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN)
                 : FPGA_MAJOR_MODE_OFF);
}

/* ---- HAL contract ------------------------------------------------------ */
uint32_t hal_rfid_caps(void) { return RFID_CAP_LF_READ | RFID_CAP_HF_READ | RFID_CAP_HF_EMU; }

static void ssp_clk_start(void);   /* SSP-clock counter bring-up (defined with the emulation block below) */

/* The bitstream a reader mode needs (basename under /fpga, no dir/extension), so
 * the app can provision it from the host before use. NULL = no FPGA resource. */
const char *hal_rfid_fpga_resource(rfid_mode_t mode)
{
    if (mode == RFID_HF_READER) return "pm3_hf";
    if (mode == RFID_LF_READER) return "pm3_lf";
    return NULL;
}

/* Load an arbitrary compressed bitstream (fpga_lzss.py format) from a VFS path
 * into the FPGA - exposed so a custom app / Berry script can drive its own design,
 * not just the RFID reader. Invalidates the tracked reader bitstream. */
int hal_rfid_fpga_load(const char *path)
{
    if (!path) return RFID_ERR_UNSUPP;
    int rc = fpga_load(path);
    s_loaded = RES_NONE;                      /* FPGA now holds a caller-chosen design */
    s_mode   = RFID_OFF;
    return rc == 0 ? 0 : RFID_ERR_UNSUPP;
}

int hal_rfid_set_mode(rfid_mode_t mode)
{
    if (mode == s_mode) return 0;

    if (mode == RFID_OFF) {
        if (s_mode != RFID_OFF) fpga_conf(FPGA_MAJOR_MODE_OFF);
        s_mode = RFID_OFF;
        s_stale = true;                       /* next reader entry re-downloads (clean sniffer demod) */
        return 0;                             /* drop the field but keep s_loaded */
    }
    if (mode != RFID_LF_READER && mode != RFID_HF_READER && mode != RFID_HF_EMU) return -1;

    bool hf = (mode == RFID_HF_READER || mode == RFID_HF_EMU);   /* EMU shares the HF bitstream */

    /* Load the bitstream only when the FPGA doesn't already hold it - so repeated
     * same-family operations (e.g. a MIFARE dump, or LF read after LF read) skip the
     * ~90 ms reload. It streams from /fpga, provisioned there from the host on first
     * use; if it isn't present the app failed to provision it -> RFID_ERR_UNSUPP. */
    int want = hf ? RES_HF : RES_LF;
    if (s_loaded != want || s_stale) {
        if (fpga_load(hf ? FPGA_HF_PATH : FPGA_LF_PATH) != 0) {
            s_loaded = RES_NONE; s_mode = RFID_OFF; return RFID_ERR_UNSUPP;
        }
        s_loaded = want;
        spin_ms(5);                           /* post-DONE startup before driving CONFREG */
    }
    s_stale = false;
    fpga_spi_setup();

    if (mode == RFID_HF_EMU) {
        /* Card-side: no field of our own - we load-modulate the reader's field. Enter the FPGA tag
         * simulator's LISTEN sub-mode (Miller RX) and bring up the ssp_clk counter the FDT alignment
         * needs; the emulation module then pumps hf_emu_recv/hf_emu_send. */
        fpga_ssc_setup();
        adc_mux(MUX_HIPKD);                   /* HF field-strength / demod line */
        fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_LISTEN);
        ssp_clk_start();                      /* free-running counter clocked by the FPGA ssp_clk */
        s_mode = RFID_HF_EMU;
    } else if (hf) {
        fpga_ssc_setup();                     /* 8-bit HF framing == the LF one */
        adc_mux(MUX_HIPKD);                   /* HF demod line (not LOPKD)      */
        fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN);  /* field on */
        spin_ms(50);                          /* antenna + card power-up settle */
        s_mode = RFID_HF_READER;
    } else {
        adc_mux(MUX_LOPKD);
        fpga_divisor(LF_DIVISOR_125);
        fpga_conf(FPGA_MAJOR_MODE_OFF);       /* configured, field still off */
        s_mode = RFID_LF_READER;
    }
    return 0;
}

void hal_rfid_field(bool on)
{
    if (s_mode == RFID_HF_READER) hf_energize(on);
    else                          lf_energize(on);
}

int hal_rfid_lf_field(bool on, uint32_t divisor)
{
    (void)divisor;
    if (s_mode != RFID_LF_READER) return RFID_ERR_UNSUPP;
    lf_energize(on);
    return 0;
}

/* Capture the LF envelope and turn it into inter-edge intervals (carrier-cycle counts) for the
 * EM4100 decoder. Same software demod as the Chameleon - detrend the envelope with an EMA (tracks
 * the frontend's DC drift), then take rising mean-crossings through a hysteresis band as bit
 * edges; the gap to the previous edge, in carrier cycles, is what the decoder reads as 1T/1.5T/2T
 * - but run streaming, one SSC sample at a time, so nothing but the intervals is ever stored.
 *
 * Streaming means the +-band can't come from a first pass over the whole capture, so the band is
 * tracked adaptively: `amp` is an EMA of |ac| (mean absolute deviation) and the band is half of
 * it, which follows the coupling strength as the tag moves instead of being fixed by whatever the
 * swing happened to be at the start. Detection is armed only after LF_WARMUP cycles, by which
 * point dc/amp have settled - otherwise the trackers' start-up transient fires spurious edges.
 * Not in a critical section: one LF sample every ~8 us, so the SSC FIFO + deadline tolerate
 * preemption. Returns the interval count, 0 if the envelope is flat (no field/tag), or <0. */
int hal_rfid_lf_acquire(uint8_t *buf, int max, uint32_t opts)
{
    (void)opts;
    if (s_mode != RFID_LF_READER || !buf || max <= 0) return RFID_ERR_UNSUPP;

    if (AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) (void)AT91C_BASE_SSC->SSC_RHR;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);

    int32_t dc = -1, amp = 0;                  /* Q8 EMAs: envelope mean, and mean |deviation| */
    uint8_t win[LF_SMOOTH] = {0};              /* boxcar taps (denoise; constant group delay) */
    int32_t wsum = 0;
    int count = 0, last = -1, prev = 0;
    bool armed = true;
    /* opts bit0 = fast-arm: a T5577 read has already held the field steady for ~2 ms before this
     * call, so there's no power-on transient to skip - a short warmup keeps the first captured edge
     * at the reply's onset (bit 31), which is what frames the 32-bit block. A cold EM4100 read still
     * uses the full warmup so its start-up transient can't fire spurious edges. */
    const int warmup = (opts & 1u) ? 24 : LF_WARMUP;

    for (int i = 0; i < LF_SPAN && count < max; i++) {
        while (!(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY))
            if (xTaskGetTickCount() > deadline) return count ? count : RFID_ERR_TIMEOUT;
        uint8_t s = (uint8_t)AT91C_BASE_SSC->SSC_RHR;

        wsum += (int32_t)s - win[i % LF_SMOOTH];   /* rolling boxcar over the last LF_SMOOTH cycles */
        win[i % LF_SMOOTH] = s;
        int32_t v = (wsum / LF_SMOOTH) << 8;

        if (dc < 0) dc = v;                    /* seed on the first sample, not on 0 */
        dc += (v - dc) >> 7;
        int ac = (int)((v - dc) >> 8);
        amp += ((((int32_t)(ac < 0 ? -ac : ac)) << 8) - amp) >> 6;

        int band = (int)(amp >> 8) / 2;
        if (band < 2) band = 2;                /* floor: don't chase pure noise */

        if (i >= warmup) {
            if (armed && ac > band && prev <= band) {
                if (last >= 0) {
                    int d = i - last;
                    buf[count++] = (d > 0xFF) ? 0xFF : (uint8_t)d;
                }
                last = i;
                armed = false;
            } else if (!armed && ac < -band) {
                armed = true;
            }
        }
        prev = ac;
    }
    if ((amp >> 8) < 2) return 0;              /* flat envelope: no field / no tag in range */
    return count;
}

/* RX loop bounds, in SSC bytes (READER_LISTEN streams ~1 byte per air bit-period).
 * Modelled on stock GetIso14443aAnswerFromTag(): a byte counter is a "the tag
 * never answered" guard that only applies while the demod is still unsynced - it
 * does not cap a response already in progress. Once synced, the loop runs until
 * the tag's end-of-comm (sequence F) completes the frame, so any-length answer
 * reads in full. HF_RX_NOANSWER_BYTES must span the card's worst-case time-to-first
 * -bit, which for a computing card is not the ~86 us FDT but the processing delay:
 * a UL-C/AES 3DES-or-AES auth challenge, or an EEPROM write's delayed ACK, keeps the
 * card silent for milliseconds. 768 (~7 ms) covers those; a fast READ syncs in ~9
 * bytes so it never waits that long. Still unsynced past the window -> no card, bail
 * (~7 ms IRQs off only in the rare genuine no-tag case, well under the abort below).
 * HF_RX_ABORT_BYTES is the absolute backstop the
 * stock loop gets from its wall-clock ms timeout - we can't use GetTickCount with
 * IRQs masked, so a byte ceiling bounds the loop instead. Sized to the stock
 * MAX_FRAME_SIZE (256 = FSC for FSDI=8, the largest legal ISO14443 frame a card may
 * send in one go - e.g. an ISO-DEP I-block or an NTAG FAST_READ): 256 bytes x 9 air
 * bit-periods = 2304, + SOF/pre-sync idle, so 2560 never truncates a legal frame.
 * Note the cost stock avoids: it receives IRQs-enabled via BigBuf, but this path
 * masks IRQs for the whole exchange (no RX DMA), so a genuine max frame is a ~24 ms
 * critical section. Real answers end far sooner via end-of-comm (a READ is ~172
 * bytes / ~1.6 ms); this ceiling only bites on a true max-size read or the
 * pathological "synced on noise" runaway. If routinely reading large frames becomes
 * a requirement, move this RX to PDC/DMA (as the sniffer already does) rather than
 * lengthening the IRQs-off spin. */
#define HF_RX_NOANSWER_BYTES  768
#define HF_RX_ABORT_BYTES     2560

/* One ISO14443-A reader transceive: encode + Miller-modulate the frame, wait the
 * frame-delay (implicit - the decoder ignores the idle gap), then Manchester-
 * decode the tag's subcarrier answer. flags==0 for every caller (REQA/anticoll/
 * SELECT): parity is added on TX and stripped on RX; no CRC is auto-appended.
 * Returns the number of received *data* bits, or RFID_ERR_TIMEOUT (no answer). */
/* Drive one reader->tag frame (already Miller-SEC-encoded in `tosend[0..tlen]`) then Manchester-decode the
 * tag answer into `rx`. Shared by the plain and custom-parity transceives. Returns received data bits, or
 * a negative RFID_ERR_*. */
static int hf_txrx(const uint8_t *tosend, int tlen, uint8_t *rx, int rx_cap, uint8_t *rx_par, int rx_par_cap)
{
    vTaskDelay(pdMS_TO_TICKS(1));               /* inter-frame guard (>= FDT/guard time) */

    /* TX + RX run uninterrupted: one SSC byte is ~9.4 us and a dropped byte
     * corrupts the (non-drop-tolerant) Manchester decode. A real exchange ends when
     * the tag completes its frame (~2 ms for a READ) or the no-answer guard trips
     * (~0.6 ms); HF_RX_ABORT_BYTES caps the worst case (a max-size frame or a noise
     * runaway) at ~24 ms - a known tradeoff of this no-DMA RX, see HF_RX_ABORT_BYTES. */
    taskENTER_CRITICAL();

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_MOD);   /* transmit */
    /* Spin-guard every SSC poll: the TX clock is driven by the FPGA, so if it comes back not-quite-
     * ready (e.g. the reload forced after a sniff's set_mode(OFF)/s_stale), TXRDY would never cycle and
     * an unguarded loop hangs the whole app inside this critical section. Bail to a framing error
     * instead - the caller just sees no answer. ~200k spins >> the ~few us a healthy TXRDY takes. */
    for (int c = 0; c < tlen; ) {
        uint32_t w = 0;
        while (!(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY) && ++w < 200000u) { }
        if (!(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY)) { taskEXIT_CRITICAL(); return RFID_ERR_FRAMING; }
        AT91C_BASE_SSC->SSC_THR = tosend[c++];
    }
    for (uint32_t w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXEMPTY) && ++w < 200000u; ) { }  /* drain EOF */

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN);/* receive */
    demod_reset(rx, rx_cap, rx_par, rx_par_cap);
    (void)AT91C_BASE_SSC->SSC_RHR;              /* drop the stale self-field byte */
    int done = 0;
    for (int nb = 0; !done && nb < HF_RX_ABORT_BYTES; nb++) {
        uint32_t w = 0;
        while (!(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) && ++w < 4000) { }
        if (!(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY)) break;    /* FPGA stopped producing */
        done = manchester_decode((uint8_t)AT91C_BASE_SSC->SSC_RHR);
        /* No-answer guard (stock: c > timeout && state==UNSYNCD): give up only if the
         * tag hasn't started answering. Once synced, nb no longer bounds the frame. */
        if (!done && s_demod.state == 0 && nb >= HF_RX_NOANSWER_BYTES) break;
    }

    taskEXIT_CRITICAL();

    if (!done || s_demod.len == 0) return RFID_ERR_TIMEOUT;
    return (int)s_demod.len * 8;
}

int hal_rfid_hf_transceive(const uint8_t *tx, int tx_bits, uint32_t flags,
                           uint8_t *rx, int rx_cap, uint32_t timeout_us)
{
    (void)flags; (void)timeout_us;
    if (s_mode != RFID_HF_READER || !tx || tx_bits <= 0 || !rx || rx_cap <= 0)
        return RFID_ERR_UNSUPP;
    static uint8_t tosend[96];
    int tlen = code_14a_reader(tx, tx_bits, tosend, (int)sizeof tosend);
    if (tlen <= 0) return RFID_ERR_FRAMING;
    return hf_txrx(tosend, tlen, rx, rx_cap, NULL, 0);
}

/* Custom-parity transceive: send `nbytes` from `tx` with the caller's parity bit per byte (`par[i]` for
 * byte i), no CRC appended; receive the tag answer into `rx`. For MIFARE Crypto1-encrypted frames, whose
 * parity is keystream-encrypted rather than the byte's own odd parity. The higher-level cipher, auth and
 * read logic lives entirely in the hot-loaded `mfc` module - this only exposes the parity control the FPGA
 * framing already needs. Returns received data bits, or a negative RFID_ERR_*. */
int hal_rfid_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                               uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us)
{
    (void)timeout_us;
    if (s_mode != RFID_HF_READER || !tx || nbytes <= 0 || !par || !rx || rx_cap <= 0)
        return RFID_ERR_UNSUPP;
    static uint8_t tosend[96];
    int tlen = code_14a_reader_ex(tx, nbytes * 8, par, tosend, (int)sizeof tosend);
    if (tlen <= 0) return RFID_ERR_FRAMING;
    return hf_txrx(tosend, tlen, rx, rx_cap, rx_par, rx_par ? rx_cap : 0);
}

/* ======================================================================== *
 *  HF (13.56 MHz) ISO14443-A passive SNIFFER - both directions at once
 * ------------------------------------------------------------------------ *
 * FPGA minor mode SNIFFER (0) is passive (never energises the coil): the FPGA's
 * hi_iso14443a module digitises the antenna envelope and streams 8-bit SSC words
 * at ~212 kB/s, each word = {reader_data[3:0], tag_data[3:0]} - 4 Miller ticks
 * (reader->tag, high nibble) and 4 Manchester ticks (tag->reader, low nibble).
 * The stream is DMA'd (PDC circular buffer) so the CPU never has to keep real-time
 * pace; two consecutive words de-interleave into one 8-tick byte per direction,
 * fed to the two state machines below (ported from stock armsrc/iso14443a.c). Each
 * decoder keeps the 9th on-air parity bit per byte (marked '!' when it violates
 * ISO14443-A odd parity). 1 tick = 16 carrier cycles = 1.18 us; timestamps are in
 * ticks, emitted with p=1180 so the host renders us directly. */

#define SNIFF_DMA_SIZE   3584   /* SSC/PDC capture ring (~17 ms at 212 kB/s), deep enough that a USB-task
                                 * preemption of the capture loop can't overrun it. Sized so the ring + the
                                 * decoded-text buffer + the two frame buffers == FANTASI_RFID_SNIFF_BUFSZ,
                                 * the ephemeral scratch the sniff module hands to hf_sniff_capture. */
#define SNIFF_FRAME_CAP  64     /* max decoded bytes per frame (a READ response is 18) */
#define SNIFF_QUIET_MS   250    /* a transaction is "done" once this long passes with no new decoded frame.
                                 * Must exceed the largest intra-dump gap, not just the ~2 ms inter-READ gap:
                                 * an `hf mfu dump` is host-paced (cascade, then RATS, then the UL-C AUTH,
                                 * then the READ burst) with tens of ms between phases - measured ~21 ms
                                 * cascade->RATS and more RATS->AUTH. 250 ms spans them all yet stays far
                                 * below the ~2.5 s gap between dumps, so one call still == one transaction */
#define SNIFF_MAX_MS     2500   /* hard cap on one active capture, regardless of quiet - the backstop that
                                 * stops a continuous field or steady demod noise from hanging the call. The
                                 * cascade+RATS+AUTH+READ 00..0A that the user wants lands well inside 1 s. */
#define SNIFF_CYCLE_MS   50     /* mid-capture, a gap this long between decoded frames marks a new field cycle
                                 * (the stock reader drops the field ~120 ms between its card-type probes and
                                 * the final READ burst). We don't end the call there - that would risk missing
                                 * the next cycle in the app's re-arm gap - we just clear the text buffer, so
                                 * what we finally emit is the last cycle alone (the READ 00..0A burst) and a
                                 * whole burst fits. Must exceed the largest intra-cycle gap (~25 ms cascade->
                                 * AUTH) and stay under the ~120 ms inter-cycle gap. */

/* nibble -> "is this 4-tick half a modulation?" LUTs (stock Mod_Miller/Manchester_LUT) */
/* Run the two decoders from RAM (.ramfunc is copied to RAM at boot - linker.ld:83). The FPGA streams
 * ~212 kB/s continuously; decoding from flash (wait-states + prefetch breaks on the per-byte branches/calls)
 * is ~12% too slow, so the DMA overruns and the recovery resets the decoder mid-frame - frames never complete.
 * Executing from RAM (0 wait-states) closes that gap. long_call so the flash->RAM branch always reaches. */
#define SN_RAMFUNC __attribute__((noinline, long_call, section(".ramfunc")))

static const uint8_t MILLER_LUT[16] = { 0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0 };
static const uint8_t MANCH_LUT2[16] = { 0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1 };

enum { U_UNSYNCD = 0, U_SOC, U_MILLER_X, U_MILLER_Y, U_MILLER_Z };
enum { D_UNSYNCD = 0, D_DATA };

static struct sn_uart {                            /* reader->tag (Miller) */
    int state, syncBit, bitCount, len, cap;
    uint32_t fourBits, startTime, endTime, shiftReg, parerr;
    uint8_t *out;
} s_u;
static struct sn_demod {                           /* tag->reader (Manchester) */
    int state, syncBit, bitCount, len, cap;
    uint16_t twoBits, highCnt, shiftReg;
    uint32_t startTime, endTime, parerr;
    uint8_t *out;
} s_dm;

static void sn_uart_reset(void)
{
    s_u.state = U_UNSYNCD; s_u.fourBits = 0; s_u.syncBit = -1;
    s_u.bitCount = 0; s_u.len = 0; s_u.shiftReg = 0; s_u.parerr = 0;
    s_u.startTime = 0; s_u.endTime = 0;
}
static void sn_demod_reset(void)
{
    s_dm.state = D_UNSYNCD; s_dm.twoBits = 0xFFFF; s_dm.highCnt = 0;
    s_dm.bitCount = 0; s_dm.syncBit = -1; s_dm.shiftReg = 0; s_dm.len = 0;
    s_dm.startTime = 0; s_dm.endTime = 0; s_dm.parerr = 0;
}

/* store a completed 9-bit group: byte + odd-parity-violation flag into parerr */
#define SN_STORE(S, SHIFT) do { \
    uint8_t _b = (uint8_t)((SHIFT) & 0xff); int _i = (S).len; \
    if ((S).len < (S).cap) (S).out[(S).len++] = _b; \
    if (_i < 32 && (((SHIFT) >> 8) & 1) != oddparity8(_b)) (S).parerr |= (1u << _i); \
    (S).bitCount = 0; (S).shiftReg = 0; \
} while (0)

/* Feed one de-interleaved reader byte (8 Miller ticks). Returns 1 on a complete frame. */
static int SN_RAMFUNC sn_miller(uint8_t bit, uint32_t t)
{
    if (s_u.len >= s_u.cap) return 1;
    s_u.fourBits = (s_u.fourBits << 8) | bit;

    if (s_u.state == U_UNSYNCD) {
        s_u.syncBit = -1;
        for (int sh = 0; sh <= 7; sh++)
            if ((s_u.fourBits & (0x07FFEF80u >> sh)) == (0x07FF8F80u >> sh)) { s_u.syncBit = 7 - sh; break; }
        if (s_u.syncBit >= 0) {
            s_u.startTime = t - (uint32_t)s_u.syncBit;
            s_u.endTime = s_u.startTime;
            s_u.state = U_SOC;
        }
        return 0;
    }

    uint32_t w = s_u.fourBits >> s_u.syncBit;
    int m1 = MILLER_LUT[(w >> 4) & 0x0F], m2 = MILLER_LUT[w & 0x0F];

    if (m1 && m2) { sn_uart_reset(); return 0; }        /* modulation both halves -> error */
    if (m1) {                                           /* Sequence Z = logic 0 */
        if (s_u.state == U_MILLER_X) { sn_uart_reset(); return 0; }
        s_u.bitCount++; s_u.shiftReg >>= 1; s_u.state = U_MILLER_Z;
        s_u.endTime = s_u.startTime + 8u * (9u * s_u.len + s_u.bitCount + 1) - 6;
        if (s_u.bitCount >= 9) SN_STORE(s_u, s_u.shiftReg);
    } else if (m2) {                                    /* Sequence X = logic 1 */
        s_u.bitCount++; s_u.shiftReg = (s_u.shiftReg >> 1) | 0x100; s_u.state = U_MILLER_X;
        s_u.endTime = s_u.startTime + 8u * (9u * s_u.len + s_u.bitCount + 1) - 2;
        if (s_u.bitCount >= 9) SN_STORE(s_u, s_u.shiftReg);
    } else {                                            /* no modulation -> Sequence Y */
        if (s_u.state == U_MILLER_Z || s_u.state == U_MILLER_Y) {   /* Y after 0 = end of comm */
            s_u.state = U_UNSYNCD; s_u.bitCount--; s_u.shiftReg <<= 1;
            if (s_u.bitCount > 0) {                     /* trailing partial byte (no parity) */
                s_u.shiftReg >>= (9 - s_u.bitCount);
                if (s_u.len < s_u.cap) s_u.out[s_u.len++] = (uint8_t)(s_u.shiftReg & 0xff);
                return 1;
            }
            return s_u.len ? 1 : (sn_uart_reset(), 0);
        }
        if (s_u.state == U_SOC) { sn_uart_reset(); return 0; }      /* Y right after SOC = error */
        s_u.bitCount++; s_u.shiftReg >>= 1; s_u.state = U_MILLER_Y; /* a logic 0 */
        if (s_u.bitCount >= 9) SN_STORE(s_u, s_u.shiftReg);
    }
    return 0;
}

/* Feed one de-interleaved tag byte (8 Manchester ticks). Returns 1 on a complete frame. */
static int SN_RAMFUNC sn_manch(uint8_t bit, uint32_t t)
{
    if (s_dm.len >= s_dm.cap) return 1;
    s_dm.twoBits = (uint16_t)((s_dm.twoBits << 8) | bit);

    if (s_dm.state == D_UNSYNCD) {
        if (s_dm.highCnt < 2) {                         /* wait for a stable unmodulated run */
            if (s_dm.twoBits == 0x0000) s_dm.highCnt++; else s_dm.highCnt = 0;
            return 0;
        }
        int sb = -1; uint16_t tb = s_dm.twoBits;
        if      ((tb & 0x7700) == 0x7000) sb = 7;
        else if ((tb & 0x3B80) == 0x3800) sb = 6;
        else if ((tb & 0x1DC0) == 0x1C00) sb = 5;
        else if ((tb & 0x0EE0) == 0x0E00) sb = 4;
        else if ((tb & 0x0770) == 0x0700) sb = 3;
        else if ((tb & 0x03B8) == 0x0380) sb = 2;
        else if ((tb & 0x01DC) == 0x01C0) sb = 1;
        else if ((tb & 0x00EE) == 0x00E0) sb = 0;
        if (sb >= 0) {
            s_dm.syncBit = sb; s_dm.startTime = t - (uint32_t)sb;
            s_dm.bitCount = 0; s_dm.state = D_DATA;
        }
        return 0;
    }

    uint16_t w = (uint16_t)(s_dm.twoBits >> s_dm.syncBit);
    int m1 = MANCH_LUT2[(w >> 4) & 0x0F], m2 = MANCH_LUT2[w & 0x0F];

    if (m1) {                                           /* Sequence D = 1 (m1&&m2 = collision, still 1) */
        s_dm.bitCount++; s_dm.shiftReg = (uint16_t)((s_dm.shiftReg >> 1) | 0x100);
        if (s_dm.bitCount == 9) SN_STORE(s_dm, s_dm.shiftReg);
        s_dm.endTime = s_dm.startTime + 8u * (9u * s_dm.len + s_dm.bitCount + 1) - 4;
    } else if (m2) {                                    /* Sequence E = 0 */
        s_dm.bitCount++; s_dm.shiftReg = (uint16_t)(s_dm.shiftReg >> 1);
        if (s_dm.bitCount >= 9) SN_STORE(s_dm, s_dm.shiftReg);
        s_dm.endTime = s_dm.startTime + 8u * (9u * s_dm.len + s_dm.bitCount + 1);
    } else {                                            /* no modulation -> end of comm */
        if (s_dm.bitCount > 0) {
            s_dm.shiftReg = (uint16_t)(s_dm.shiftReg >> (9 - s_dm.bitCount));
            if (s_dm.len < s_dm.cap) s_dm.out[s_dm.len++] = (uint8_t)(s_dm.shiftReg & 0xff);
            return 1;
        }
        if (s_dm.len) return 1;
        sn_demod_reset();
    }
    return 0;
}

/* Append a base-10 uint32 (no libc). */
static char *sn_u32(char *p, uint32_t v)
{
    char t[10]; int k = 0;
    do { t[k++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (k) *p++ = t[--k];
    return p;
}

/* Emit one decoded frame as a host sniff line: "<R|C> <start> <end> <hex>[!]..".
 * Hand-rolled (no snprintf): the inline emit runs inside the sample loop, and
 * picolibc's snprintf - a format-string parse plus a %lu long-division per call,
 * ~100 us/frame on the 48 MHz ARM7 with no HW divide - made the loop fall behind
 * during the dense AUTH+READ burst, overrun, and skip the reads. This is ~10x
 * cheaper (one divide-loop per timestamp, table hex for the bytes), so the loop
 * keeps up and the whole burst is captured. */
static int sn_emit(char *txt, int pos, int cap, char dir, const uint8_t *b, int n,
                   uint32_t start, uint32_t end, uint32_t parerr)
{
    static const char HEX[] = "0123456789ABCDEF";
    if (n <= 0 || pos > cap - (24 + n * 4)) return pos;   /* worst case: 2x10-digit ts + " XX!" per byte */
    char *p = txt + pos;
    *p++ = dir; *p++ = ' ';
    p = sn_u32(p, start); *p++ = ' ';
    p = sn_u32(p, end);
    for (int i = 0; i < n; i++) {
        *p++ = ' '; *p++ = HEX[b[i] >> 4]; *p++ = HEX[b[i] & 0x0F];
        if ((parerr >> i) & 1) *p++ = '!';
    }
    *p++ = '\n';
    return (int)(p - txt);
}

#define SNIFF_HDR 40   /* bytes reserved at the front of txt for the prepended L-header */

/* Keep the decoded-text buffer to its most recent frames. An `hf mfu dump` is a long, multi-phase
 * transaction - the stock reader spends most of it probing the card type (Classic auth, RATS, UL-C auth,
 * version/config reads) across many field cycles, and only issues the READ 00..0A burst near the very end.
 * That is far more than fits `cap`. A plain bounded buffer keeps the first cap bytes and so drops exactly
 * the burst the user wants; instead, when near-full we drop the oldest whole lines, so the burst - which
 * comes last - always survives to be emitted. Frames are whole '\n'-terminated lines, so we cut on one. */
static int sn_compact(char *txt, int pos, int cap)
{
    if (pos <= cap - 300) return pos;                    /* room for another max-length frame line */
    int cut = SNIFF_HDR + (pos - SNIFF_HDR) / 3;         /* drop the oldest ~third of the buffered lines */
    while (cut < pos && txt[cut] != '\n') cut++;         /* advance to the end of that line */
    if (cut >= pos) return pos;                          /* one line spanning the third (can't happen) */
    cut++;                                               /* keep from the next line's first byte */
    memmove(txt + SNIFF_HDR, txt + cut, (size_t)(pos - cut));
    return SNIFF_HDR + (pos - cut);
}

int hal_rfid_hf_sniff_capture(uint8_t *buf, uint32_t cap_bytes, uint32_t quiet_ms, uint32_t max_ms)
{
    if (s_mode != RFID_HF_READER || !buf) return RFID_ERR_UNSUPP;
    if (cap_bytes < FANTASI_RFID_SNIFF_BUFSZ) return RFID_ERR_UNSUPP;   /* too small to carve safely */
    if (!quiet_ms) quiet_ms = SNIFF_QUIET_MS;
    if (!max_ms)   max_ms   = SNIFF_MAX_MS;

    /* One continuous SNIFFER capture per call, modelled on the reference armsrc/iso14443a.c
     * SniffIso14443a: set up the FPGA sniffer + SSC/PDC once, then stream samples in a single loop - no
     * per-window teardown, re-bias or settle mid-capture. We return when the
     * transaction goes quiet (quiet_ms with no new frame) or the idle cap expires, so one call ==
     * one whole transaction and a READ burst (30 00..30 0A) is captured contiguously, never split across
     * a window boundary. The sniff module loops, printing each transaction and re-arming in the gap. */

    /* All scratch is carved from the caller's ephemeral buffer (the sniff module allocates it from the
     * heap for the duration of a sniff and frees it after) - the frontend keeps no static sniff buffer.
     * cap(text) + dma(ring) + ub + db == FANTASI_RFID_SNIFF_BUFSZ (5760). */
    const int cap = 2048;                          /* decoded-frame text: a full dump (cascade + AUTH + all
                                                    * 11 READs + 18-byte responses) is ~1.7 KB */
    char    *txt = (char *)buf;
    uint8_t *dma = (uint8_t *)(((uintptr_t)buf + cap + 3) & ~(uintptr_t)3);        /* 4-align for the PDC */
    uint8_t *ub  = dma + SNIFF_DMA_SIZE;
    uint8_t *db  = ub + SNIFF_FRAME_CAP;           /* 2048 + 3584 + 64 + 64 = 5760 == SNIFF_BUFSZ */

    fpga_ssc_setup();                              /* 8-bit MSB-first framing (matches the sniffer word) */
    adc_mux(MUX_HIPKD);                            /* route the HF peak-detector into the ADC (ref SetAdcMuxFor) */
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_SNIFFER);   /* passive - no field */
    spin_ms(8);                                    /* let the peak-detector bias settle once, before capture */

    /* SSC PDC: one self-chaining ring (primary and "next" both point at it). */
    AT91C_BASE_SSC->SSC_PTCR = AT91C_PDC_RXTDIS;
    AT91C_BASE_SSC->SSC_RPR  = (uint32_t)(uintptr_t)dma;
    AT91C_BASE_SSC->SSC_RCR  = SNIFF_DMA_SIZE;
    AT91C_BASE_SSC->SSC_RNPR = (uint32_t)(uintptr_t)dma;
    AT91C_BASE_SSC->SSC_RNCR = SNIFF_DMA_SIZE;
    AT91C_BASE_SSC->SSC_PTCR = AT91C_PDC_RXTEN;

    s_u.out  = ub; s_u.cap  = SNIFF_FRAME_CAP; sn_uart_reset();
    s_dm.out = db; s_dm.cap = SNIFF_FRAME_CAP; sn_demod_reset();

    int pos = SNIFF_HDR;                           /* reserve the front for the L-header */
    int valid = 0, saw_activity = 0;
    uint32_t rx_samples = 0;
    uint8_t *data = dma, previous = 0;
    int TagActive = 0, ReaderActive = 0;

    TickType_t t0 = xTaskGetTickCount(), last_active = t0, last_wdt = t0;

    for (;;) {
        /* One xTaskGetTickCount for both the WDT feed and the return decision, both evaluated every iteration
         * (not gated on the ring draining - a live dump streams faster than we drain, so avail stays >0 and
         * the check would otherwise never run and the call would stall mid-dump). */
        TickType_t now = xTaskGetTickCount();

        /* Feed the offical WDT on a ~150 ms time basis. It is a windowed watchdog: restarting it too late
         * times out, but restarting it too soon (every iteration) lands in the forbidden window and also
         * resets - both show as total silence. ~150 ms sits safely in the allowed window, so a long single-call
         * capture can run without tripping it. */
        if ((now - last_wdt) >= pdMS_TO_TICKS(150)) { AT91C_BASE_WDTC->WDTC_WDCR = 0xA5000001; last_wdt = now; }

        {
            if (saw_activity) {
                /* stay in one call for a whole cascade->RATS->AUTH->READ burst (gaps < quiet_ms),
                 * but always bail out by max_ms so a continuous field or steady noise can't hang us */
                if ((now - last_active) >= pdMS_TO_TICKS(quiet_ms)) break;
                if ((now - t0)        >= pdMS_TO_TICKS(max_ms))     break;
            } else if ((now - t0) >= pdMS_TO_TICKS(quiet_ms)) {
                break;                             /* idle: return so the module can poll for a stop keypress */
            }
        }

        /* DMA fully stalled (both PDC banks drained): disable RX, clear the SSC overrun latch, re-arm both
         * banks, resync the read pointer, drop the in-flight frame. Straight from the reference stall
         * recovery - rx_samples += (not = 0) preserves its parity, which is the reader/tag de-interleave
         * phase. Without the RX re-enable the re-armed RCR never counts down and the loop spins idle. */
        if (AT91C_BASE_SSC->SSC_RCR == 0) {
            AT91C_BASE_SSC->SSC_PTCR = AT91C_PDC_RXTDIS;
            (void)AT91C_BASE_SSC->SSC_SR;           /* clear OVRUN */
            AT91C_BASE_SSC->SSC_RPR  = (uint32_t)(uintptr_t)dma; AT91C_BASE_SSC->SSC_RCR  = SNIFF_DMA_SIZE;
            AT91C_BASE_SSC->SSC_RNPR = (uint32_t)(uintptr_t)dma; AT91C_BASE_SSC->SSC_RNCR = SNIFF_DMA_SIZE;
            AT91C_BASE_SSC->SSC_PTCR = AT91C_PDC_RXTEN;
            data = dma; rx_samples += SNIFF_DMA_SIZE; sn_uart_reset(); sn_demod_reset();
            continue;
        }
        if (AT91C_BASE_SSC->SSC_RNCR == 0) {       /* secondary drained, primary running -> refill */
            AT91C_BASE_SSC->SSC_RNPR = (uint32_t)(uintptr_t)dma; AT91C_BASE_SSC->SSC_RNCR = SNIFF_DMA_SIZE;
        }

        int wr = SNIFF_DMA_SIZE - (int)AT91C_BASE_SSC->SSC_RCR;   /* DMA write index */
        int rd = (int)(data - dma);
        int avail = (rd <= wr) ? (wr - rd) : (SNIFF_DMA_SIZE - rd + wr);

        if (avail == 0)                            /* caught up with the DMA: spin (return is decided at the
                                                    * top). Never yield - a task switch here lets the DMA
                                                    * overrun and the dense READ burst gets skipped. */
            continue;
        if (avail > 9 * SNIFF_DMA_SIZE / 10) {     /* fell behind the writer: skip to the head (ref overrun) */
            data = dma + wr; rx_samples += avail; sn_uart_reset(); sn_demod_reset();
            continue;
        }

        for (int i = 0; i < avail; i++) {
            uint8_t cur = *data;
            if (rx_samples & 1) {                  /* two SSC words de-interleave into one byte per direction */
                if (!TagActive) {
                    uint8_t rdr = (uint8_t)((previous & 0xF0) | (cur >> 4));
                    if (sn_miller(rdr, (rx_samples - 1) * 4)) {
                        if (s_u.len > 0) {
                            TickType_t fn = xTaskGetTickCount();
                            if (saw_activity && (fn - last_active) >= pdMS_TO_TICKS(SNIFF_CYCLE_MS))
                                pos = SNIFF_HDR;               /* new field cycle: keep only the newest one */
                            pos = sn_compact(txt, pos, cap);   /* safety net if one cycle alone overruns cap */
                            pos = sn_emit(txt, pos, cap, 'R', ub, s_u.len,
                                          s_u.startTime, s_u.endTime, s_u.parerr);
                            valid++; saw_activity = 1; last_active = fn;
                        }
                        sn_uart_reset(); sn_demod_reset();
                    }
                    ReaderActive = (s_u.state != U_UNSYNCD);
                }
                if (!ReaderActive) {
                    uint8_t tag = (uint8_t)((previous << 4) | (cur & 0x0F));
                    if (sn_manch(tag, (rx_samples - 1) * 4)) {
                        if (s_dm.len > 0) {
                            TickType_t fn = xTaskGetTickCount();
                            if (saw_activity && (fn - last_active) >= pdMS_TO_TICKS(SNIFF_CYCLE_MS))
                                pos = SNIFF_HDR;               /* new field cycle: keep only the newest one */
                            pos = sn_compact(txt, pos, cap);   /* safety net if one cycle alone overruns cap */
                            pos = sn_emit(txt, pos, cap, 'C', db, s_dm.len,
                                          s_dm.startTime, s_dm.endTime, s_dm.parerr);
                            valid++; saw_activity = 1; last_active = fn;
                        }
                        sn_uart_reset(); sn_demod_reset();
                    }
                    TagActive = (s_dm.state != D_UNSYNCD);
                }
            }
            previous = cur;
            rx_samples++;
            if (++data == dma + SNIFF_DMA_SIZE) data = dma;
        }
    }

    /* Stop DMA; leave the front-end in READER_LISTEN so a follow-up raw/search is ready. */
    AT91C_BASE_SSC->SSC_PTCR = AT91C_PDC_RXTDIS;
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN);

    if (valid == 0) return 0;                      /* nothing decoded: the module prints nothing */

    /* The L-header frame count must be what survived in the buffer, not the total ever decoded: on a long
     * dump, the per-cycle clear and compaction drop earlier lines, so `valid` (every frame decoded)
     * overstates what we emit. Count the frame lines actually left in the buffer. */
    int shown = 0;
    for (int k = SNIFF_HDR; k < pos; k++) if (txt[k] == '\n') shown++;

    /* Prepend the L-header (the host CLI tabulates the frames): coupling unknown (the PM3 sniffer streams
     * pre-digitised data, no field amplitude), p1180 = each timestamp tick is 1180 ns. */
    char h[SNIFF_HDR];
    int hl = snprintf(h, sizeof h, "L9 v%d p1180\n", shown ? shown : 1);
    memcpy(txt + (SNIFF_HDR - hl), h, (size_t)hl);
    /* Slide the finished trace (header + frame lines) to the front of the caller's buffer and return its
     * length; the sniff module prints buf[0..len] straight to its console. No file, no static buffer -
     * the scratch is the module's own ephemeral heap allocation, freed when the sniff loop ends. */
    int off = SNIFF_HDR - hl, len = pos - off;
    memmove(txt, txt + off, (size_t)len);
    return len;
}

int hal_rfid_hf_probe(void) { return -1; }

/* ============================================================================
 * HF ISO14443-A TAG EMULATION (card side)
 *
 * The mirror image of the reader path: rather than Miller-encode a command and
 * Manchester-decode the tag reply, we Miller-decode the reader's command
 * (FPGA TAGSIM_LISTEN) and load-modulate a pre-encoded subcarrier reply back
 * (FPGA TAGSIM_MOD) at the ISO14443-A frame delay time. The FPGA's hi_iso14443a
 * tag simulator enforces the 1172-carrier-cycle FDT in hardware (FDT_COUNT); the
 * ARM only has to have the reply bytes ready and add the 1236 correction bit when
 * the last received bit demands it. Thin by design (cf. lf_transceive): this HAL
 * does only the real-time bit I/O + FDT timing - the Manchester response encoding,
 * the anticollision/Crypto1 state machine and the card image all live in the
 * emulation module. Ported from stock Proxmark3 firmware.
 * ==========================================================================*/

/* The reader->tag Miller decoder (sn_miller / struct sn_uart s_u) is already ported above for the sniffer -
 * we reuse it verbatim, so emulation adds no second decoder. sn_miller stores decoded bytes to s_u.out and a
 * per-byte odd-parity-violation bitmap to s_u.parerr; the raw parity bit of byte i (which the reply's
 * 1236-vs-1172 correction and the module's Crypto1 need) is oddparity8(out[i]) ^ ((parerr>>i)&1). */
static uint16_t s_queue_delay;      /* FPGA send-queue depth reported back over SSC (for the flush) */

/* ---- SSP-clock counter: TC0/TC1/TC2 chained into a 32-bit counter clocked by the FPGA ssp_clk (13.56/16
 * MHz), synced to ssp_frame. The reader path times out on the RTOS tick, but FDT alignment needs sub-us
 * SSP-phase resolution. Verbatim port of common_arm/ticks.c StartCountSspClk (8-bit-frame path only - the
 * Fantasi SSC is 8-bit, so the 16-bit-frame extra clock waits are dropped). GPIO_SSC_FRAME=PA15, CLK=PA16. */
static void ssp_clk_start(void)
{
    AT91C_BASE_PMC->PMC_PCER = (1u << AT91C_ID_TC0) | (1u << AT91C_ID_TC1) | (1u << AT91C_ID_TC2);
    AT91C_BASE_TCB->TCB_BMR = AT91C_TCB_TC0XC0S_TIOA1 | AT91C_TCB_TC1XC1S_NONE | AT91C_TCB_TC2XC2S_TIOA0;

    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC1->TC_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK | AT91C_TC_CPCSTOP | AT91C_TC_EEVTEDG_RISING
                           | AT91C_TC_EEVT_TIOB | AT91C_TC_ENETRG | AT91C_TC_WAVESEL_UP | AT91C_TC_WAVE
                           | AT91C_TC_AEEVT_SET | AT91C_TC_ACPC_CLEAR;
    AT91C_BASE_TC1->TC_RC = 0x01;

    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_XC0 | AT91C_TC_WAVE | AT91C_TC_WAVESEL_UP
                           | AT91C_TC_ACPA_CLEAR | AT91C_TC_ACPC_SET | AT91C_TC_ASWTRG_SET;
    AT91C_BASE_TC0->TC_RA = 1;
    AT91C_BASE_TC0->TC_RC = 0;

    AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC2->TC_CMR = AT91C_TC_CLKS_XC2 | AT91C_TC_WAVE | AT91C_TC_WAVESEL_UP;

    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;

    /* sync to ssp_frame (FPGA must be in an SSC mode so SSC_FRAME/SSC_CLK are live) */
    while (AT91C_BASE_PIOA->PIO_PDSR & SSC_TF);       /* wait for ssp_frame low        */
    while (!(AT91C_BASE_PIOA->PIO_PDSR & SSC_TF));    /* wait for ssp_frame high (SOF)  */
    while (!(AT91C_BASE_PIOA->PIO_PDSR & SSC_TK));    /* 1st ssp_clk rising             */
    while (AT91C_BASE_PIOA->PIO_PDSR & SSC_TK);
    while (!(AT91C_BASE_PIOA->PIO_PDSR & SSC_TK));    /* 2nd ssp_clk rising             */

    AT91C_BASE_TCB->TCB_BCR = 1;                      /* assert sync: zero all timers on next active edge */
    while (AT91C_BASE_TC2->TC_CV > 0);
}

static uint32_t ssp_clk_get(void)
{
    uint32_t v = (AT91C_BASE_TC2->TC_CV << 16) | AT91C_BASE_TC0->TC_CV;
    if ((v & 0x0000ffff) == 0) return (AT91C_BASE_TC2->TC_CV << 16);   /* possibly missed a TC2 increment */
    return v;
}

/* Receive one reader command as a tag: reuse the sniffer's Miller decoder (sn_miller/s_u) over the FPGA's
 * demodulated TAGSIM_LISTEN stream until end-of-frame. Fills rx[0..n) and rx_par[i] = the raw parity bit of
 * byte i (derived from s_u.parerr); returns the byte count, 0 if no command arrived within timeout_ms
 * (reader idle/gone), or <0 on error. The FPGA arms its FDT counter off this frame's end, and s_u is retained
 * so hf_emu_send can pick the 1236-vs-1172 correction bit. */
int hal_rfid_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{
    if (s_mode != RFID_HF_EMU || !rx || !rx_par || cap <= 0) return RFID_ERR_UNSUPP;

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_LISTEN);
    s_u.out = rx; s_u.cap = cap; sn_uart_reset();
    /* Flush all SSC RX left over from the preceding TAGSIM_MOD send (the fdt_indicator/queue-delay reads):
     * feeding those stale bytes into the Miller decoder desyncs its sync on the next reader command. Drain what's pending now - the
     * next command hasn't arrived yet (the reader's FDT gap), so this can't eat it. */
    for (int f = 0; f < 64 && (AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY); f++) (void)AT91C_BASE_SSC->SSC_RHR;

    int n = 0;

    /* PHASE 1 - catch an imminent command IRQs-off. After we load-modulate a reply the reader's next command
     * lands within ~1 ms; if a FreeRTOS preemption hits during those first Miller bits the 1-byte SSC receive
     * register overflows and the whole command is lost. So wait for the command's start with interrupts off, bounded (~2 ms) so an
     * idle wait can't monopolise the CPU; once a command starts, decode it fully in the same critical section
     * (a partial command that stalls is capped so a glitch can't hang the app). */
    taskENTER_CRITICAL();
    {
        uint32_t spins = 0;
        for (;;) {
            if (AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) {
                uint8_t b = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
                if (sn_miller(b, 0)) { n = (int)s_u.len; break; }
            }
            if (++spins > 45000u && s_u.state == U_UNSYNCD) break;  /* nothing arriving -> to preemptible phase */
            if (spins > 600000u) break;                            /* safety: never hold the critical section forever */
        }
    }
    taskEXIT_CRITICAL();

    /* PHASE 2 - nothing arrived in the imminent window, so wait the rest of the timeout preemptibly (the reader
     * may be idle for ms). Decode IRQs-off once a command starts, same as before. A missed idle poll (REQA/WUPA)
     * is simply retried by the reader, so preemption here is harmless - unlike a dropped mid-sequence command. */
    if (!n && s_u.state == U_UNSYNCD) {
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 200);
        uint32_t spins = 0;
        int crit = 0;
        for (;;) {
            spins++;
            if (crit) {
                if (spins > 300000u) break;                        /* safety: never hold the critical section forever */
            } else if ((spins & 0xFFF) == 0) {
                AT91C_BASE_WDTC->WDTC_WDCR = 0xA5000001;            /* WDT fed only while preemptible/idle */
                if (xTaskGetTickCount() > deadline) break;
            }
            if (AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) {
                uint8_t b = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
                int done = sn_miller(b, 0);                        /* t=0: startTime unused in emu; tightens the loop */
                if (!crit && s_u.state != U_UNSYNCD) { taskENTER_CRITICAL(); crit = 1; spins = 0; }  /* cmd started */
                if (done) { n = (int)s_u.len; break; }
            }
        }
        if (crit) taskEXIT_CRITICAL();
    }

    for (int i = 0; i < n && i < 32; i++)
        rx_par[i] = oddparity8(rx[i]) ^ (uint8_t)((s_u.parerr >> i) & 1);
    return n;
}

/* Load-modulate a pre-encoded tag reply at the ISO14443-A frame delay time. `tosend` is one byte per
 * subcarrier symbol (the module built it: leading 8-bit correction template, start bit, per-bit SEC_D/E +
 * parity symbols, stop bit). We switch to TAGSIM_MOD, keep or drop the correction symbol from the last
 * received frame's parity (1236 vs 1172), wait for the FPGA fdt_indicator, phase-align to the ssp_clk, then
 * stream the symbols; the FPGA holds them in its delay line and releases them exactly at the FDT. Returns 0.
 * Verbatim port of EmSendCmd14443aRaw. Must directly follow hf_emu_recv. */
int hal_rfid_hf_emu_send(const uint8_t *tosend, int len)
{
    if (s_mode != RFID_HF_EMU || !tosend || len <= 0) return RFID_ERR_UNSUPP;
    volatile uint8_t b;
    uint32_t w;

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_MOD);

    /* Keep the leading correction symbol (-> 1236) vs drop it (-> 1172) from the last received frame's final
     * bit: a lone byte is a 7-bit short frame (REQA/WUPA) -> its bit 6; else the raw parity of the last byte
     * (oddparity8 ^ the parerr flag sn_miller stored). */
    bool correction;
    if (s_u.len == 1) correction = (s_u.out[0] & 0x40) != 0;
    else { int li = s_u.len - 1; correction = (oddparity8(s_u.out[li]) ^ (uint8_t)((s_u.parerr >> li) & 1)) != 0; }
    int i = correction ? 0 : 1;

    /* The whole reply must stream without a gap: the SSC has only a 1-byte holding register, so if FreeRTOS
     * preempts us mid-feed the TX underruns and the FPGA stops modulating - the reader then sees a truncated
     * reply (a 5-bit UID / 1-bit SAK). Bare-metal stock never hits this; we run the feed IRQs-off. Every wait
     * is bounded so a missing reader can't hang the app inside the critical section. */
    taskENTER_CRITICAL();

    for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) && ++w < 200000u; ) { }
    b = AT91C_BASE_SSC->SSC_RHR; (void)b;

    for (uint8_t j = 0; j < 5; j++) {                          /* wait for FPGA fdt_indicator (SSC RX != 0) */
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) && ++w < 200000u; ) { }
        if (AT91C_BASE_SSC->SSC_RHR) break;
    }

    while ((ssp_clk_get()) & 0x00000007) { }                  /* phase-lock to an 8-tick SSP boundary */

    AT91C_BASE_SSC->SSC_THR = 0x00;                            /* prime TXRDY with an idle symbol */
    for (; i < len; ) {
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY) && ++w < 200000u; ) { }
        AT91C_BASE_SSC->SSC_THR = tosend[i++];
        s_queue_delay = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }
    /* drain the FPGA delay queue with idle symbols so the last reply symbol propagates all the way out the
     * coil before the caller's next recv switches to TAGSIM_LISTEN (a short drain also truncates the reply). */
    uint8_t queued = s_queue_delay >> 3;
    int flush = (queued >> 3) + 1; if (flush < 6) flush = 6;
    for (i = 0; i < flush; ) {
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY) && ++w < 200000u; ) { }
        AT91C_BASE_SSC->SSC_THR = 0x00;
        s_queue_delay = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
        i++;
    }

    taskEXIT_CRITICAL();
    return 0;
}

/* Streaming tag reply: identical FDT discipline to hf_emu_send, but each subcarrier symbol is fetched from
 * next(ctx) just before it's fed. The compute for symbol k+1 overlaps the transmission of symbol k, so a
 * module can encrypt+encode a Crypto1 reply one bit per symbol in the ~9.4 us feed gaps - the first symbol
 * (start bit, no crypto) is fed right at the FDT, so even an 18-byte encrypted READ answers at ~a real
 * card's frame delay instead of after computing the whole ciphertext. next() must return exactly nsymbols
 * symbols (start bit + 9 per byte + stop); the correction template is prepended here as for hf_emu_send. */
int hal_rfid_hf_emu_send_stream(uint8_t (*next)(void *ctx), void *ctx, int nsymbols)
{
    if (s_mode != RFID_HF_EMU || !next || nsymbols <= 0) return RFID_ERR_UNSUPP;
    volatile uint8_t b;
    uint32_t w;

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_MOD);

    bool correction;
    if (s_u.len == 1) correction = (s_u.out[0] & 0x40) != 0;
    else { int li = s_u.len - 1; correction = (oddparity8(s_u.out[li]) ^ (uint8_t)((s_u.parerr >> li) & 1)) != 0; }

    taskENTER_CRITICAL();

    for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) && ++w < 200000u; ) { }
    b = AT91C_BASE_SSC->SSC_RHR; (void)b;
    for (uint8_t j = 0; j < 5; j++) {                          /* wait for FPGA fdt_indicator */
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) && ++w < 200000u; ) { }
        if (AT91C_BASE_SSC->SSC_RHR) break;
    }
    while ((ssp_clk_get()) & 0x00000007) { }                  /* phase-lock to the 8-tick SSP boundary */

    AT91C_BASE_SSC->SSC_THR = 0x00;                            /* prime */
    if (correction) {                                         /* keep the correction symbol -> 1236 FDT */
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY) && ++w < 200000u; ) { }
        AT91C_BASE_SSC->SSC_THR = 0x08;
        s_queue_delay = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }
    for (int k = 0; k < nsymbols; k++) {
        uint8_t sym = next(ctx);                              /* compute k overlaps the transmission of k-1 */
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY) && ++w < 200000u; ) { }
        AT91C_BASE_SSC->SSC_THR = sym;
        s_queue_delay = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }
    uint8_t queued = s_queue_delay >> 3;
    int flush = (queued >> 3) + 1; if (flush < 6) flush = 6;
    for (int k = 0; k < flush; ) {
        for (w = 0; !(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_TXRDY) && ++w < 200000u; ) { }
        AT91C_BASE_SSC->SSC_THR = 0x00;
        s_queue_delay = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
        k++;
    }

    taskEXIT_CRITICAL();
    return 0;
}

/* T5577 "fixed bit length" downlink timing, PM3 standard-antenna "Default Fixed" defaults
 * (armsrc/lfops.c), in us (1 field clock = 8 us @125 kHz): a data bit = field ON for write_0
 * or write_1 us (the value), then a write_gap field-OFF; the command is preceded by a start_gap. */
#define T55_START_GAP_US  248   /* 31 fc */
#define T55_WRITE_GAP_US  160   /* 20 fc */
#define T55_WRITE_0_US    144   /* 18 fc */
#define T55_WRITE_1_US    400   /* 50 fc */
#define T55_POWERUP_MS    8     /* field on before the command, to charge the tag  */
#define T55_PROGRAM_MS    6     /* field on after, for the EEPROM write to commit   */
/* Field-ON settle after the read command, before capturing the reply. Stock (armsrc/lfops.c
 * T55xxReadBlock) waits 137*8 us here via turn_read_lf_on, then captures a long window (12000 samples ~=
 * 6 block repeats) so the settled repeating data - not the noisy first bits - is what the demod frames
 * ("we want to go past the start and let the repeating data settle in"). We keep this short so the leading
 * quiet gap stays in the (shorter) capture as a frame anchor, and skip the settling-corrupted first copy
 * in the demodulator instead. */
#define T55_READ_SETTLE_US 120    /* ~15 fc - land just before the reply so the gap is captured */

/* Fast LF field on/off for gap modulation - just the FPGA config word (the divisor is already set
 * by set_mode(LF_READER)); no SSC/settle, unlike lf_energize which is for capture. */
static void lf_tx_on(void)  { fpga_conf(FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD); }
static void lf_tx_off(void) { fpga_conf(FPGA_MAJOR_MODE_OFF); }

/* Gap-modulate the T5577 reader->tag command in `p` (one byte per bit, 0/1, MSB-first). Charges the
 * tag, then IRQs-off sends start_gap + each bit's field-ON/gap-OFF (the per-bit timing must not be
 * preempted). Leaves the field on at exit (IRQs back on) - the caller decides what happens next: a
 * write holds it for the EEPROM commit, a read holds it for the tag's reply. Shared by modulate+read. */
static void lf_tx_cmd(const uint8_t *p, int nbits)
{
    lf_tx_on();
    spin_ms(T55_POWERUP_MS);                             /* charge the tag (IRQs on, field steady) */

    taskENTER_CRITICAL();                                /* gap timing must not be preempted */
    lf_tx_off(); spin_us(T55_START_GAP_US);              /* start gap */
    for (int i = 0; i < nbits; i++) {
        lf_tx_on();  spin_us(p[i] ? T55_WRITE_1_US : T55_WRITE_0_US);
        lf_tx_off(); spin_us(T55_WRITE_GAP_US);
    }
    lf_tx_on();                                          /* field back on */
    taskEXIT_CRITICAL();
}

/* T5577 block WRITE downlink (TX only): gap-modulate `p` then hold the field for the EEPROM commit.
 * Returns 0, or RFID_ERR_UNSUPP if not in LF mode. */
int hal_rfid_lf_modulate(const uint8_t *p, int nbits, uint32_t opts)
{
    (void)opts;
    if (s_mode != RFID_LF_READER || !p || nbits <= 0) return RFID_ERR_UNSUPP;

    lf_tx_cmd(p, nbits);
    spin_ms(T55_PROGRAM_MS);                             /* EEPROM programming (IRQs on, field steady) */
    lf_tx_off();
    return 0;
}

/* LF reader->tag round-trip capture (see app_rfid.h): optionally gap-modulate a downlink command, hold the
 * field, and stream-demodulate the reply into `buf` as inter-edge run lengths (sample counts of each level
 * run), up to `cap` runs - never a raw envelope buffer, so it costs no heap. The runs alternate low, high,
 * low... from the first falling edge (the gap->block transition); the caller reconstructs half-bit levels
 * and frames the block. For a
 * T5577 read the command is opcode 10 + lock 0 + 3-bit block address; the field stays on and the tag clocks
 * the block out continuously. Returns the run count, or <0. */
int hal_rfid_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap)
{
    if (s_mode != RFID_LF_READER || !buf || cap <= 0) return RFID_ERR_UNSUPP;

    if (cmd && nbits > 0) lf_tx_cmd(cmd, nbits);         /* send downlink; field left ON */
    else                  lf_tx_on();
    fpga_ssc_setup();                                    /* (re)arm the envelope sample stream */
    spin_us(T55_READ_SETTLE_US);                         /* wait past the peak-detector settle */
    if (AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY) (void)AT91C_BASE_SSC->SSC_RHR;   /* flush stale */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);

    /* Demodulate streaming, one SSC sample at a time, storing only the reply's inter-edge run lengths -
     * never a raw envelope buffer. A boxcar
     * denoises; EMA trackers give the envelope mean (dc) and a hysteresis band (amp); a two-state edge
     * detector emits the sample count of each level run. The read gap is unmodulated carrier = envelope
     * high, so we start `high` and anchor recording on the first falling edge - the gap->block transition,
     * whose timing is data-independent (to +-1 half-bit). Anchoring instead on the first rising edge would
     * start 1-2 half-bits into the block depending on its first bits, a data-dependent full-bit phase.
     * The runs alternate low, high, low... from that edge. */
    int32_t dc = -1, amp = 0;
    uint8_t win[LF_SMOOTH] = {0};
    int32_t wsum = 0;
    int count = 0, last_edge = -1;
    int high = 1, started = 0;

    for (int i = 0; i < LF_SPAN && count < cap; i++) {
        while (!(AT91C_BASE_SSC->SSC_SR & AT91C_SSC_RXRDY))
            if (xTaskGetTickCount() > deadline) { lf_tx_off(); return count; }
        uint8_t s = (uint8_t)AT91C_BASE_SSC->SSC_RHR;

        wsum += (int32_t)s - win[i % LF_SMOOTH];         /* rolling boxcar over the last LF_SMOOTH cycles */
        win[i % LF_SMOOTH] = s;
        int32_t v = (wsum / LF_SMOOTH) << 8;

        if (dc < 0) dc = v;
        dc += (v - dc) >> 7;
        int ac = (int)((v - dc) >> 8);
        amp += ((((int32_t)(ac < 0 ? -ac : ac)) << 8) - amp) >> 6;

        int band = (int)(amp >> 8) / 2;
        if (band < 2) band = 2;                          /* floor: don't chase pure noise */

        if (i >= 24) {                                   /* fast-arm: field already steady */
            if (high && ac < -band) {                    /* falling edge (high -> low) */
                if (started) { int d = i - last_edge; buf[count++] = d > 0xFF ? 0xFF : (uint8_t)d; }
                else started = 1;                        /* first falling edge = gap->block: begin, no run yet */
                last_edge = i; high = 0;
            } else if (!high && ac > band) {             /* rising edge (low -> high) */
                if (started) { int d = i - last_edge; buf[count++] = d > 0xFF ? 0xFF : (uint8_t)d; }
                last_edge = i; high = 1;
            }
        }
    }
    lf_tx_off();
    return count;
}
