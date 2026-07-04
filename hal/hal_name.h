#ifndef FANTASI_HAL_NAME_H
#define FANTASI_HAL_NAME_H

#include <stdint.h>
#include <stddef.h>

#define _NAME_N_CONS   21
#define _NAME_N_VOWELS  5

static inline void hal_name_generate(const uint32_t *uid, int n_words,
                                     char *buf, size_t len)
{
    static const char * const cons[_NAME_N_CONS] = {
        "b","d","f","g","h","j","k","l","m","n",
        "p","r","s","t","v","w","y","z","ch","sh","th"
    };
    static const char vowels[_NAME_N_VOWELS] = {'a','e','1','0','u'};

    if (len < 2) { buf[0] = '\0'; return; }

    uint32_t s = 0x811c9dc5u;
    for (int i = 0; i < n_words; i++) {
        s ^= uid[i];
        s *= 0x01000193u;
    }

    uint8_t r[12];
    for (int i = 0; i < 12; i++) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        r[i] = (uint8_t)(s >> 8);
    }

    int syllables = (r[0] < 26) ? 2 : 3;
    char *p = buf;
    char *end = buf + len - 1;

    for (int i = 0; i < syllables && p < end; i++) {
        const char *c = cons[r[2*i + 1] % _NAME_N_CONS];
        while (*c && p < end) *p++ = *c++;
        if (p < end) *p++ = vowels[r[2*i + 2] % _NAME_N_VOWELS];
    }

    int ti = 2 * syllables + 1;
    if (r[ti] % 5 < 2 && p < end) {
        const char *c = cons[r[ti + 1] % _NAME_N_CONS];
        while (*c && p < end) *p++ = *c++;
    }

    *p = '\0';
    if (buf[0] >= 'a' && buf[0] <= 'z')
        buf[0] -= 32;
}

#endif
