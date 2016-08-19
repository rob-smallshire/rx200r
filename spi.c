//
// Created by rjs on 17.08.16.
//

#include <avr/io.h>

#include <stdint.h>
#include <util/delay.h>


#include "spi.h"


void spi_send_2(uint8_t hi, uint8_t lo);

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

void spi_send_2(uint8_t hi, uint8_t lo) {
    spi_select_slave();
    _delay_us(5.0);
    spi_send(hi);
    _delay_us(7.0);
    spi_send(lo);
    _delay_us(5.0);
    spi_deselect_slave();
}

uint16_t spi_send_receive_2(uint8_t hi, uint8_t lo) {
    spi_select_slave();
    _delay_us(5.0);
    unsigned int status_hi = spi_send_receive(hi);
    _delay_us(7.0);
    unsigned int status_lo = spi_send_receive(lo);
    _delay_us(5.0);
    spi_deselect_slave();
    return status_hi << 8 | status_lo;
}