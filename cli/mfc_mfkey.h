/* Host-side mfkey64: recover a MIFARE Classic sector key from one sniffed authentication (uid, nt, nr, ar,
 * at). Used by `collect mfc sniff` to turn captured reader<->card auths into keys for /nfc/mfc.dict. This is
 * pure host computation (crapto1 lfsr_recovery64) - the device only sniffs. Ported from proxmark3
 * common/crapto1/crapto1.c (lfsr_recovery64) + client/src/mifare/mfkey.c (mfkey64). */
#ifndef FANTASI_HOST_MFC_MFKEY_H
#define FANTASI_HOST_MFC_MFKEY_H

#include <stdint.h>
#include <stdlib.h>
#include "mfc_crypto.h"     /* mc_t, mc_filter, mc_evenparity32, mc_rollback_word, MC_BIT/MC_BEBIT */

/* crapto1 recovery tables (crapto1.c). */
static const uint32_t MK_S1[] = { 0x62141, 0x310A0, 0x18850, 0x0C428, 0x06214, 0x0310A, 0x85E30, 0xC69AD,
    0x634D6, 0xB5CDE, 0xDE8DA, 0x6F46D, 0xB3C83, 0x59E41, 0xA8995, 0xD027F, 0x6813F, 0x3409F, 0x9E6FA };
static const uint32_t MK_S2[] = { 0x3A557B00, 0x5D2ABD80, 0x2E955EC0, 0x174AAF60, 0x0BA557B0, 0x05D2ABD8,
    0x0449DE68, 0x048464B0, 0x42423258, 0x278192A8, 0x156042D0, 0x0AB02168, 0x43F89B30, 0x61FC4D98,
    0x765EAD48, 0x7D8FDD20, 0x7EC7EE90, 0x7F63F748, 0x79117020 };
static const uint32_t MK_T1[] = { 0x4F37D, 0x279BE, 0x97A6A, 0x4BD35, 0x25E9A, 0x12F4D, 0x097A6, 0x80D66,
    0xC4006, 0x62003, 0xB56B4, 0x5AB5A, 0xA9318, 0xD0F39, 0x6879C, 0xB057B, 0x582BD, 0x2C15E, 0x160AF,
    0x8F6E2, 0xC3DC4, 0xE5857, 0x72C2B, 0x39615, 0x98DBF, 0xC806A, 0xE0680, 0x70340, 0x381A0, 0x98665,
    0x4C332, 0xA272C };
static const uint32_t MK_T2[] = { 0x3C88B810, 0x5E445C08, 0x2982A580, 0x14C152C0, 0x4A60A960, 0x253054B0,
    0x52982A58, 0x2FEC9EA8, 0x1156C4D0, 0x08AB6268, 0x42F53AB0, 0x217A9D58, 0x161DC528, 0x0DAE6910,
    0x46D73488, 0x25CB11C0, 0x52E588E0, 0x6972C470, 0x34B96238, 0x5CFC3A98, 0x28DE96C8, 0x12CFC0E0,
    0x4967E070, 0x64B3F038, 0x74F97398, 0x7CDC3248, 0x38CE92A0, 0x1C674950, 0x0E33A4A8, 0x01B959D0,
    0x40DCACE8, 0x26CEDDF0 };
static const uint32_t MK_C1[] = { 0x846B5, 0x4235A, 0x211AD };
static const uint32_t MK_C2[] = { 0x1A822E0, 0x21A822E0, 0x21A822E0 };

static uint32_t mk_prng_successor(uint32_t x, uint32_t n)
{
    x = (x >> 8 & 0xff00ff) | (x & 0xff00ff) << 8; x = x >> 16 | x << 16;
    while (n--) x = x >> 1 | (x >> 16 ^ x >> 18 ^ x >> 19 ^ x >> 21) << 31;
    x = (x >> 8 & 0xff00ff) | (x & 0xff00ff) << 8; return x >> 16 | x << 16;
}

static void mk_get_lfsr(const mc_t *s, uint64_t *lfsr)
{
    *lfsr = 0;
    for (int i = 23; i >= 0; --i) {
        *lfsr = *lfsr << 1 | MC_BIT(s->odd, i ^ 3);
        *lfsr = *lfsr << 1 | MC_BIT(s->even, i ^ 3);
    }
}

static inline void mk_extend_table_simple(uint32_t *tbl, uint32_t **end, int bit)
{
    for (*tbl <<= 1; tbl <= *end; *++tbl <<= 1) {
        uint8_t f = (uint8_t)mc_filter(*tbl);
        if (f ^ mc_filter(*tbl | 1)) *tbl |= f ^ bit;
        else if (f == bit) { *++*end = *++tbl; *tbl = tbl[-1] | 1; }
        else { *tbl-- = *(*end)--; }
    }
}

/* Recover cipher states from 64 bits of keystream (crapto1 lfsr_recovery64). Returns a malloc'd list; the
 * first entry with odd|even set is the (usually unique) solution. Caller frees. */
static mc_t *mk_lfsr_recovery64(uint32_t ks2, uint32_t ks3)
{
    mc_t *statelist = calloc(1, sizeof(mc_t) << 4);
    uint32_t *table = malloc((1u << 16) * sizeof(uint32_t));
    if (!statelist || !table) { free(statelist); free(table); return NULL; }
    mc_t *sl = statelist; sl->odd = sl->even = 0;
    uint8_t oks[32], eks[32], hi[32];
    uint32_t low = 0, win = 0, *tail;
    int i, j;

    for (i = 30; i >= 0; i -= 2) { oks[i >> 1] = (uint8_t)MC_BEBIT(ks2, i); oks[16 + (i >> 1)] = (uint8_t)MC_BEBIT(ks3, i); }
    for (i = 31; i >= 0; i -= 2) { eks[i >> 1] = (uint8_t)MC_BEBIT(ks2, i); eks[16 + (i >> 1)] = (uint8_t)MC_BEBIT(ks3, i); }

    for (i = 0xfffff; i >= 0; --i) {
        if (mc_filter(i) != oks[0]) continue;
        *(tail = table) = i;
        for (j = 1; tail >= table && j < 29; ++j) mk_extend_table_simple(table, &tail, oks[j]);
        if (tail < table) continue;

        for (low = 0, j = 0; j < 19; ++j) low = low << 1 | mc_evenparity32(i & MK_S1[j]);
        for (j = 0; j < 32; ++j) hi[j] = (uint8_t)mc_evenparity32(i & MK_T1[j]);

        for (; tail >= table; --tail) {
            for (j = 0; j < 3; ++j) {
                *tail = *tail << 1;
                *tail |= mc_evenparity32((i & MK_C1[j]) ^ (*tail & MK_C2[j]));
                if (mc_filter(*tail) != oks[29 + j]) goto next;
            }
            for (win = 0, j = 0; j < 19; ++j) win = win << 1 | mc_evenparity32(*tail & MK_S2[j]);
            win ^= low;
            for (j = 0; j < 32; ++j) {
                win = win << 1 ^ hi[j] ^ mc_evenparity32(*tail & MK_T2[j]);
                if (mc_filter(win) != eks[j]) goto next;
            }
            *tail = *tail << 1 | mc_evenparity32(MC_LF_POLY_EVEN & *tail);
            sl->odd = *tail ^ mc_evenparity32(MC_LF_POLY_ODD & win);
            sl->even = win;
            ++sl; sl->odd = sl->even = 0;
next:       ;
        }
    }
    free(table);
    return statelist;
}

/* Recover the sector key from one sniffed authentication (mfkey.c mfkey64). Returns 0 + *key, or -1. */
static int mc_mfkey64(uint32_t uid, uint32_t nt, uint32_t nr, uint32_t ar, uint32_t at, uint64_t *key)
{
    uint32_t ks2 = ar ^ mk_prng_successor(nt, 64);
    uint32_t ks3 = at ^ mk_prng_successor(nt, 96);
    mc_t *sl = mk_lfsr_recovery64(ks2, ks3);
    if (!sl) return -1;
    if (!sl->odd && !sl->even) { free(sl); return -1; }   /* no candidate */
    mc_rollback_word(sl, 0, 0);
    mc_rollback_word(sl, 0, 0);
    mc_rollback_word(sl, nr, 1);
    mc_rollback_word(sl, uid ^ nt, 0);
    mk_get_lfsr(sl, key);
    free(sl);
    return 0;
}

#endif /* FANTASI_HOST_MFC_MFKEY_H */
