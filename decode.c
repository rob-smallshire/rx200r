//
// Created by Robert Smallshire on 6/12/16.
//

#include <math.h>
#include <stdint.h>

/**
 * Convert a 16-bit decimal float to a IEEE float.
 *
 *   lo        hi
 * 76543210 76543210
 * mmmmmmmm eemmmmmm
 * 76543210 10111198
 *            3210
 *
 * m7-0; e1-e0, m12-m8
 *
 * @param lo
 * @param hi
 * @return
 */
float decimal_float(uint8_t lo, uint8_t hi) {
    int16_t mantissa = lo | ((hi & 0b00111111) << 8);
    int8_t exponent = hi >> 6;
    float value = mantissa / pow(10.0f, exponent);
    return value;
}

