//
// Created by rjs on 17.08.16.
//

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <avr/io.h>

#include <util/delay.h>

#include "twiddle.h"
#include "spi.h"

#include "alpha_rx_commands.h"

uint16_t alpha_tx_get_status_command() {
    static const unsigned int GET_STATUS_COMMAND_HI = 0x00;
    static const unsigned int GET_STATUS_COMMAND_LO = 0x00;
    CLR(PORTB, DDB5);
    _delay_us(5.0);
    unsigned int status_hi = spi_send_receive(GET_STATUS_COMMAND_HI);
    _delay_us(7.0);
    unsigned int status_lo = spi_send_receive(GET_STATUS_COMMAND_LO);
    _delay_us(5.0);
    SET(PORTB, DDB5);
    return status_hi << 8 | status_lo;
}

void alpha_tx_configuration_setting_command(
        enum Band band,
        bool enable_low_battery_detection,
        bool enable_wake_up_timer,
        bool enable_crystal_oscillator,
        enum CrystalLoadCapacitor crystal_load_capacitor,
        enum BasebandBandwidth baseband_bandwidth,
        bool disable_clock_output) {
    unsigned int hi = 0x80
                 | ((unsigned int)band << 3)
                 | (enable_low_battery_detection << 2)
                 | (enable_wake_up_timer << 1)
                 | (enable_crystal_oscillator);
    unsigned int lo = ((unsigned int)crystal_load_capacitor << 4)
                 | ((unsigned int)baseband_bandwidth << 1)
                 | (disable_clock_output);
    CLR(PORTB, DDB5);
    _delay_us(5.0);
    spi_send(hi);
    _delay_us(7.0);
    spi_send(lo);
    _delay_us(5.0);
    SET(PORTB, DDB5);
}

uint16_t alpha_rx_frequency_to_f(enum Band band, float frequency) {
    int c1 = 0;
    int c2 = 0;
    switch (band) {
        case BAND_315_MHz:
            c1 = 1;
            c2 = 31;
            break;
        case BAND_433_MHz:
            c1 = 1;
            c2 = 43;
            break;
        case BAND_868_MHz:
            c1 = 2;
            c2 = 43;
            break;
        case BAND_915_MHz:
            // check! Values missing from Alpha TX datasheet
            // error
            break;
    }
    return (uint16_t) ((((frequency / (ALPHA_RX_CLOCK_MHZ * c1)) - c2) * 4000) + 0.5);
}

void alpha_rx_frequency_setting_command(uint16_t f) {
    if (f < 96 || f > 3903) {
        return;
    }
    static const unsigned int FREQUENCY_COMMAND_HI = 0xa0;
    uint8_t hi = (uint8_t) ((f >> 8) & 0x0f) | FREQUENCY_COMMAND_HI;
    uint8_t lo = (uint8_t) (f & 0xff);
    CLR(PORTB, DDB5);
    _delay_us(5.0);
    spi_send(hi);
    _delay_us(7.0);
    spi_send(lo);
    _delay_us(5.0);
    SET(PORTB, DDB5);
}