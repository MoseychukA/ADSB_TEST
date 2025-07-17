#include "decode_utils.h"

#include <cmath>

#include "unit_conversions.h"


uint16_t IdentityCodeToSquawk(uint16_t identity_code) 
{
    uint8_t d1 = (identity_code & (0b1 << 4)) >> 4;
    uint8_t d2 = (identity_code & (0b1 << 2)) >> 2;
    uint8_t d4 = identity_code & 0b1;

    uint8_t a1 = (identity_code & (0b1 << 11)) >> 11;
    uint8_t a2 = (identity_code & (0b1 << 9)) >> 9;
    uint8_t a4 = (identity_code & (0b1 << 7)) >> 7;

    uint8_t b1 = (identity_code & (0b1 << 5)) >> 5;
    uint8_t b2 = (identity_code & (0b1 << 3)) >> 3;
    uint8_t b4 = (identity_code & (0b1 << 1)) >> 1;

    uint8_t c1 = (identity_code & (0b1 << 12)) >> 12;
    uint8_t c2 = (identity_code & (0b1 << 10)) >> 10;
    uint8_t c4 = (identity_code & (0b1 << 8)) >> 8;

    return (a4 << 11) | (a2 << 10) | (a1 << 9) | (b4 << 8) | (b2 << 7) | (b1 << 6) | (c4 << 5) | (c2 << 4) | (c1 << 3) |
           (d4 << 2) | (d2 << 1) | d1;
}