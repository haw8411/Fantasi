#include "rfid.h"

/* ISO/IEC 14443-A CRC ("CRC_A"): reflected poly 0x8408, preset 0x6363, no final
 * XOR/reflection. Emitted LSB first, which is the on-air byte order. */
void rfid_crc_a(const uint8_t *data, int len, uint8_t *crc_out)
{
    uint16_t crc = 0x6363;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i] ^ (uint8_t)(crc & 0xFF);
        b ^= (uint8_t)(b << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)b << 8) ^ ((uint16_t)b << 3) ^ ((uint16_t)b >> 4));
    }
    crc_out[0] = (uint8_t)(crc & 0xFF);
    crc_out[1] = (uint8_t)((crc >> 8) & 0xFF);
}
