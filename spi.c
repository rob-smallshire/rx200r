//
// Created by rjs on 17.08.16.
//

#include <stdio.h>

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
    //printf("spi_send_2(0x%02x, 0x%02x)\n", hi, lo);
    spi_select_slave();
    _delay_us(5.0);
    spi_send(hi);
    _delay_us(7.0);
    spi_send(lo);
    _delay_us(5.0);
    spi_deselect_slave();
}

uint16_t spi_send_receive_2(uint8_t hi, uint8_t lo) {
    //printf("spi_send_receive_2(0x%02x, 0x%02x)", hi, lo);
    spi_select_slave();
    _delay_us(5.0);
    uint16_t status_hi = spi_send_receive(hi);
    _delay_us(7.0);
    uint16_t status_lo = spi_send_receive(lo);
    _delay_us(5.0);
    spi_deselect_slave();
    uint16_t result = status_hi << 8 | status_lo;
    //printf("= 0x%04X\n", result);
    return result;
}