//
// Created by rjs on 17.08.16.
//

#ifndef RX200R_SPI_H
#define RX200R_SPI_H

#include <stdint.h>

void spi_send(uint8_t data);
uint8_t spi_receive();
uint8_t spi_send_receive(uint8_t data);

#endif //RX200R_SPI_H
