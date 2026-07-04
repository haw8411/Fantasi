#include "ble_common.h"

#include <stdlib.h>

int parse_mac(const char *s, uint8_t *out)
{
    for (int i = 5; i >= 0; i--) {
        char *end;
        unsigned long v = strtoul(s, &end, 16);
        if (v > 0xFF) return -1;
        out[i] = (uint8_t)v;
        if (i > 0) {
            if (*end != ':') return -1;
            s = end + 1;
        } else {
            if (*end != '\0') return -1;
        }
    }
    return 0;
}
