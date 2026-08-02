#ifndef FANTASI_LZSS_H
#define FANTASI_LZSS_H

#include <stdint.h>

/* Minimal streaming LZSS decoder for the FPGA-bitstream compression (tools/
 * fpga_lzss.py produces the stream). Pull-based so a caller can feed a chunked
 * flash/VFS read and push each decoded byte straight into a hardware sink (e.g.
 * bit-bang into the FPGA) without ever buffering the whole ~42 KB payload.
 *
 * Wire format (matches fpga_lzss.py): a stream of groups. Each group is a flag
 * byte (bit 7 = first token ... bit 0 = eighth) followed by the token bodies:
 *   literal (flag bit 0): 1 verbatim byte
 *   match   (flag bit 1): 2 bytes = [offset_hi(4) | (len-3)(4)] [offset_lo(8)],
 *                         offset in 1..4096, len in 3..18, copied from history.
 * Decoding stops once `out_len` bytes have been produced (so a partial final
 * group needs no terminator). */

typedef int  (*lzss_src_fn)(void *ctx, uint8_t *buf, int max);  /* fill buf, return count, <=0 = end */
typedef void (*lzss_sink_fn)(void *ctx, uint8_t byte);          /* consume one decoded byte */

/* Decode exactly `out_len` bytes. `win`/`wsize` is the back-reference history;
 * wsize must be a power of two >= 4096 (the max match offset). Returns 0 on
 * success, -1 if the compressed stream ends early / is malformed. */
int lzss_decode(uint32_t out_len,
                lzss_src_fn src, void *src_ctx,
                lzss_sink_fn sink, void *sink_ctx,
                uint8_t *win, uint32_t wsize);

#endif /* FANTASI_LZSS_H */
