//
// Created by Robert Smallshire on 6/12/16.
//

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Convert a 16-bit decimal float to a IEEE float.
 *
 * The value consists of a 14-bit mantissa and
 * a 2-bit base-10 exponent.
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
float decimal_to_float(uint8_t lo, uint8_t hi) {
    int16_t mantissa = lo | ((hi & 0x3f) << 8);
    int8_t exponent = hi >> 6;
    float value = mantissa / pow(10.0f, exponent);
    return value;
}

/**
 * Convert a 16-bit decimal float to a C string.
 *
 * The value consists of a 14-bit mantissa and
 * a 2-bit base-10 exponent.
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
 * @param s A pointer to a string buffer with at least 7 bytes free.
 * @return A null-terminated ASCII representation of the decimal.
 */
void decimal_to_str(uint8_t lo, uint8_t hi, char* s) {
    int16_t mantissa = lo | ((hi & 0x3f) << 8);
    int8_t exponent = hi >> 6;
    char* t = itoa(mantissa, s, 10);
    if (exponent != 0) {
        // Copy the null-terminator and the digits comprising the fraction part
        // of the number one character to the right.
        while (*t++); // t now points to the character after the null terminator
        char* u = t - 1;
        for (int8_t i = 0; i <= exponent; ++i) {
            *t = *u;
            --t;
            --u;
        }
        // Insert the decimal point in the gap
        *t = '.';
    }
}