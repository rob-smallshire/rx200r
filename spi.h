//
// Created by rjs on 17.08.16.
//

#ifndef RX200R_SPI_H
#define RX200R_SPI_H

#include <stdint.h>

#include "twiddle.h"

void spi_send(uint8_t data);
uint8_t spi_receive();
uint8_t spi_send_receive(uint8_t data);

inline void spi_select_slave() {
    CLR(PORTB, DDB2);
}

inline void spi_deselect_slave() {
    SET(PORTB, DDB2);
}

void spi_send_2(uint8_t hi, uint8_t low);
uint16_t spi_send_receive_2(uint8_t hi, uint8_t low);

#endif //RX200R_SPI_H
