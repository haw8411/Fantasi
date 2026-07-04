#ifndef CORE_COMMANDS_BLE_COMMON_H
#define CORE_COMMANDS_BLE_COMMON_H

#include <stdint.h>

/* Parse "AA:BB:CC:DD:EE:FF" into out[6], stored little-endian (out[0] = FF) to
 * match the HAL address convention. Returns 0 on success, -1 on malformed input.
 * Shared by the pair / connect / unpair commands. */
int parse_mac(const char *s, uint8_t *out);

#endif
