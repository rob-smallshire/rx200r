//
// Created by rjs on 17.08.16.
//

#include <avr/io.h>

#include <stdint.h>


#include "spi.h"


void spi_send(uint8_t data) {
    spi_send_receive(data);
}

uint8_t spi_receive() {
    return spi_send_receive(0x00);
}

uint8_t spi_send_receive(uint8_t data) {
    // transmit the byte to be sent
    SPDR = data;
    // wait for the transfer to complete
    while (!(SPSR & (1<<SPIF)));
    // then return the byte the slave just returned
    return SPDR;
}