//
// Created by Robert Smallshire on 13/12/16.
//

#include <stdio.h>
#include <inttypes.h>

#include "checksum.h"

uint8_t checksum(uint8_t* begin, uint8_t* end) {
    uint16_t sum = 0;
    for(uint8_t* p = begin; p != end; ++p) {
        uint8_t b = *p;
        sum += b;
        //printf("%02x %" PRIu16 "\n", b, sum);
    }
    return sum % 256;
}
