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
    return spi_send_receive_2(GET_STATUS_COMMAND_HI, GET_STATUS_COMMAND_LO);
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
    spi_send_2(hi, lo);
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
    spi_send_2(hi, lo);
}

void alpha_rx_receiver_setting_command(
        enum VdiSource vdi_source,
        enum LnaGain lna_gain,
        enum DrssiTheshold drssi_threshold,
        enum ReceiverState receiver_state
) {
    static const uint8_t RECEIVER_SETTING_COMMAND_HI = 0xc0;
    uint8_t lo = (((uint8_t)vdi_source) << 6)
                 | (((uint8_t)lna_gain) << 4)
                 | (((uint8_t)drssi_threshold) << 1)
                 | (((uint8_t)receiver_state));
    spi_send_2(RECEIVER_SETTING_COMMAND_HI, lo);
}

uint8_t alpha_rx_data_rate_to_cs_r(float data_rate) {
    uint8_t cs = 0; // cs can be either 0 or 1 - should really be a parameter or a loop to find best
    uint8_t r = (uint8_t) (((10e6 / 29 / (1 + cs*7)/data_rate) - 1) + 0.5);
    uint8_t cs_r = r | (cs << 7);
    return cs_r;
}

void alpha_rx_data_rate_command(uint8_t cs_r) {
    static const uint8_t DATA_RATE_COMMAND_HI = 0xc8;
    spi_send_2(DATA_RATE_COMMAND_HI, cs_r);
}
