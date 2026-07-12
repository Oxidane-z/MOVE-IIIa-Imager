/*
 * CRC-16/CCITT-FALSE — poly=0x1021, init=0xFFFF, no reflect, xor-out=0x0000.
 * Check vector: input "123456789" (9 bytes) → 0x29B1.
 *
 * Used on the SPI frame payload (pixel words only, no header). The MI48
 * publishes its own CRC over the same range in header word index 7; the
 * driver compares the two to flag corrupt frames.
 *
 * Byte order within a word: LSB first. Although the MI48 transmits the
 * pixel words big-endian on the wire, its CRC engine (and crcmod on the
 * host, which sees the numpy array's native little-endian buffer) feeds the
 * LOW byte first. Verified against a real frame: CCITT-FALSE over the pixel
 * payload LSB-first == the MI48's own header CRC word 7 (e.g. 0x69b4);
 * MSB-first gives 0x166e and never matches. See pysenxor mi48.py:541-546.
 */
#include <stddef.h>
#include <stdint.h>

#include "mi1602_internal.h"

uint16_t mi1602_crc16_ccitt_false(const uint16_t *data_words, size_t nwords)
{
    uint16_t crc = 0xFFFF;
    for (size_t w = 0; w < nwords; ++w) {
        uint16_t word = data_words[w];
        uint8_t b_hi = (uint8_t)(word >> 8);
        uint8_t b_lo = (uint8_t)(word & 0xFF);

        /* LOW byte first, then HIGH byte (little-endian feed). */
        crc ^= ((uint16_t)b_lo) << 8;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
        crc ^= ((uint16_t)b_hi) << 8;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}
