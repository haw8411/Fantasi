/* Host-side MIFARE Crypto1 offline matcher for the mfc read.
 *
 * The device (mfc_collect module) harvests raw encrypted nested nonces; the HOST does the non-time-sensitive
 * cracking here - decrypt each candidate dictionary key against a nonce and test it - so the 12 KB dictionary
 * and this matcher never live on the device. Cipher + rollback ported from proxmark3 common/crapto1. */
#ifndef FANTASI_HOST_MFC_CRYPTO_H
#define FANTASI_HOST_MFC_CRYPTO_H

#include <stdint.h>

#define MC_LF_POLY_ODD  0x29CE5Cu
#define MC_LF_POLY_EVEN 0x870804u
#define MC_BIT(x, n)    ((uint32_t)((x) >> (n) & 1))
#define MC_BEBIT(x, n)  MC_BIT(x, (n) ^ 24)

static inline uint8_t  mc_oddparity8(uint8_t x)   { return (uint8_t)(!__builtin_parity(x)); }
static inline uint8_t  mc_evenpar8(uint8_t x)     { return (uint8_t)(__builtin_parity(x) & 1); }
static inline uint32_t mc_evenparity32(uint32_t x){ return __builtin_parity(x) & 1; }

static inline int mc_filter(uint32_t x)
{
    uint32_t f;
    f  = 0xf22c0u >> (x       & 0xf) & 16;
    f |= 0x6c9c0u >> (x >>  4 & 0xf) &  8;
    f |= 0x3c8b0u >> (x >>  8 & 0xf) &  4;
    f |= 0x1e458u >> (x >> 12 & 0xf) &  2;
    f |= 0x0d938u >> (x >> 16 & 0xf) &  1;
    return (int)MC_BIT(0xEC57E80Au, f);
}

typedef struct { uint32_t odd, even; } mc_t;

static inline void mc_init(mc_t *s, uint64_t key)
{
    s->odd = s->even = 0;
    for (int i = 47; i > 0; i -= 2) {
        s->odd  = s->odd  << 1 | MC_BIT(key, (i - 1) ^ 7);
        s->even = s->even << 1 | MC_BIT(key, i ^ 7);
    }
}

static inline uint8_t mc_bit(mc_t *s, uint8_t in, int enc)
{
    uint32_t feedin, t;
    uint8_t ret = (uint8_t)mc_filter(s->odd);
    feedin  = ret & (!!enc);
    feedin ^= !!in;
    feedin ^= MC_LF_POLY_ODD  & s->odd;
    feedin ^= MC_LF_POLY_EVEN & s->even;
    s->even = s->even << 1 | mc_evenparity32(feedin);
    t = s->odd; s->odd = s->even; s->even = t;
    return ret;
}

static inline uint32_t mc_word(mc_t *s, uint32_t in, int enc)
{
    uint32_t r = 0;
    for (int i = 0; i < 32; i++) r |= (uint32_t)mc_bit(s, (uint8_t)MC_BEBIT(in, i), enc) << (24 ^ i);
    return r;
}

static inline uint8_t mc_rollback_bit(mc_t *s, uint32_t in, int fb)
{
    uint8_t ret;
    uint32_t t, out;
    s->odd &= 0xffffff;
    t = s->odd; s->odd = s->even; s->even = t;

    out  = s->even & 1;
    out ^= MC_LF_POLY_EVEN & (s->even >>= 1);
    out ^= MC_LF_POLY_ODD & s->odd;
    out ^= (uint32_t)(!!in);
    out ^= (ret = (uint8_t)mc_filter(s->odd)) & (uint32_t)(!!fb);

    s->even |= mc_evenparity32(out) << 23;
    return ret;
}

static inline uint32_t mc_rollback_word(mc_t *s, uint32_t in, int fb)
{
    uint32_t ret = 0;
    for (int i = 31; i >= 0; i--)
        ret |= (uint32_t)mc_rollback_bit(s, MC_BEBIT(in, i), fb) << (24 ^ i);
    return ret;
}

/* Recover a nested tag nonce's plaintext under a candidate key (Flipper crypto1_decrypt_nt_enc). */
static inline uint32_t mc_decrypt_nt_enc(uint32_t uid, uint32_t nt_enc, uint64_t key)
{
    mc_t c; mc_init(&c, key);
    mc_word(&c, nt_enc ^ uid, 1);
    return nt_enc ^ mc_rollback_word(&c, nt_enc ^ uid, 1);
}

/* Is `nonce` a valid 16-bit-LFSR (weak) PRNG nonce? (== pm3 validate_prng_nonce.) */
static inline int mc_is_weak_prng_nonce(uint32_t nonce)
{
    if (nonce == 0) return 0;
    uint16_t x = (uint16_t)(nonce >> 16);
    x = (uint16_t)((x & 0xff) << 8 | x >> 8);
    for (int i = 0; i < 16; i++)
        x = (uint16_t)(x >> 1 | (x ^ x >> 2 ^ x >> 3 ^ x >> 5) << 15);
    x = (uint16_t)((x & 0xff) << 8 | x >> 8);
    return x == (nonce & 0xFFFF);
}

/* Does plaintext nonce `nt` (keystream `ks = nt ^ nt_enc`) agree with the 3 checkable encrypted parity bits
 * `parpk` (packed MSB-first, byte0 at bit3, each = raw air bit XOR 1)? Flipper nonce_matches_encrypted_parity. */
static inline int mc_nonce_parity_ok(uint32_t nt, uint32_t ks, uint8_t parpk)
{
    return mc_evenpar8((uint8_t)(nt >> 24)) == (((parpk >> 3) & 1) ^ (uint8_t)MC_BIT(ks, 16)) &&
           mc_evenpar8((uint8_t)(nt >> 16)) == (((parpk >> 2) & 1) ^ (uint8_t)MC_BIT(ks,  8)) &&
           mc_evenpar8((uint8_t)(nt >>  8)) == (((parpk >> 1) & 1) ^ (uint8_t)MC_BIT(ks,  0));
}

#endif /* FANTASI_HOST_MFC_CRYPTO_H */
