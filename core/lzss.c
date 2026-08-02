#include "lzss.h"

int lzss_decode(uint32_t out_len,
                lzss_src_fn src, void *src_ctx,
                lzss_sink_fn sink, void *sink_ctx,
                uint8_t *win, uint32_t wsize)
{
    uint8_t  inbuf[128];
    int      inlen = 0, inpos = 0;
    uint32_t produced = 0, wpos = 0;
    uint8_t  flags = 0;
    int      fbits = 0;
    uint32_t mask = wsize - 1;

    /* Pull one compressed byte, refilling from src as needed. Jumps to `fail`
     * (via the enclosing function's return) if the stream runs dry mid-token. */
#define NEXT(out) do {                                              \
        if (inpos >= inlen) {                                        \
            inlen = src(src_ctx, inbuf, (int)sizeof inbuf);          \
            inpos = 0;                                               \
            if (inlen <= 0) return -1;                               \
        }                                                            \
        (out) = inbuf[inpos++];                                      \
    } while (0)

    while (produced < out_len) {
        if (fbits == 0) { NEXT(flags); fbits = 8; }
        int is_match = flags & 0x80;
        flags <<= 1;
        fbits--;

        if (is_match) {
            uint8_t b0, b1;
            NEXT(b0);
            NEXT(b1);
            uint32_t off = ((((uint32_t)b0 >> 4) << 8) | b1) + 1;
            uint32_t len = (uint32_t)(b0 & 0x0F) + 3;
            for (uint32_t k = 0; k < len && produced < out_len; k++) {
                uint8_t byte = win[(wpos - off) & mask];
                sink(sink_ctx, byte);
                win[wpos & mask] = byte;
                wpos++;
                produced++;
            }
        } else {
            uint8_t byte;
            NEXT(byte);
            sink(sink_ctx, byte);
            win[wpos & mask] = byte;
            wpos++;
            produced++;
        }
    }
    return 0;
#undef NEXT
}
