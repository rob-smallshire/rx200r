//
// Created by rjs on 17.08.16.
//

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include <avr/io.h>

#include <util/delay.h>

#include "twiddle.h"
#include "spi.h"

#include "alpha_rx_commands.h"

uint16_t alpha_rx_get_status_command() {
    //printf("alpha_rx_get_status_command()\n");
    static const unsigned int GET_STATUS_COMMAND_HI = 0x00;
    static const unsigned int GET_STATUS_COMMAND_LO = 0x00;
    uint16_t result = spi_send_receive_2(GET_STATUS_COMMAND_HI, GET_STATUS_COMMAND_LO);
    _delay_us(7.0);
    return result;
}

const char* BASEBAND_BANDWIDTH[7] = {
        "--",
        "400 kHz",
        "340 kHz",
        "270 kHz",
        "200 kHz",
        "134 kHz",
        " 67 kHz"
};

void alpha_rx_configuration_setting_command(
        enum Band band,
        bool enable_low_battery_detection,
        bool enable_wake_up_timer,
        bool enable_crystal_oscillator,
        enum CrystalLoadCapacitor crystal_load_capacitor,
        enum BasebandBandwidth baseband_bandwidth,
        bool disable_clock_output) {
    //printf("alpha_rx_configuration_setting_command(%d, %d, %d, %d, %d, %d, %d)\n",
    //    band, enable_low_battery_detection, enable_wake_up_timer, enable_crystal_oscillator,
    //    crystal_load_capacitor, baseband_bandwidth, disable_clock_output);
    uint8_t hi = 0x80
                 | (((uint8_t)band) << 3)
                 | (((uint8_t)enable_low_battery_detection) << 2)
                 | (((uint8_t)enable_wake_up_timer) << 1)
                 | ((uint8_t)enable_crystal_oscillator);
    uint8_t lo = (((uint8_t)crystal_load_capacitor) << 4)
                 | (((uint8_t)baseband_bandwidth) << 1)
                 | ((uint8_t)disable_clock_output);
    spi_send_2(hi, lo);
    _delay_us(7.0);
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
    printf("alpha_rx_frequency_setting_command(%" PRIu16 ")\n", f);
    if (f < 96 || f > 3903) {
        return;
    }
    static const uint8_t FREQUENCY_COMMAND_HI = 0xa0;
    uint8_t hi = ((uint8_t) ((f >> 8) & 0x0f)) | FREQUENCY_COMMAND_HI;
    uint8_t lo = (uint8_t) (f & 0xff);
    spi_send_2(hi, lo);
    _delay_us(7.0);
}

const char* LNA_GAIN[4] = {
        "-0 dBm",
        "-14 dBm",
        "-6 dBm",
        "-20 dBm"
};

const enum LnaGain LNA_GAIN_ORDERED[4] = {
        LNA_GAIN_0_DBM,
        LNA_GAIN_MINUS_6_DBM,
        LNA_GAIN_MINUS_14_DBM,
        LNA_GAIN_MINUS_20_DBM};

const char* DRSSI_THESHOLD[8] = {
        "-130 dBm",
        "-97 dBm",
        "-91 dBm",
        "-85 dBm",
        "-79 dBm",
        "-73 dBm",
        "-67 dBm",
        "-61 dBm"
};

void alpha_rx_receiver_setting_command(
        enum VdiSource vdi_source,
        enum LnaGain lna_gain,
        enum DrssiTheshold drssi_threshold,
        enum ReceiverState receiver_state
) {
    //printf("alpha_rx_receiver_setting_command(%d, %d, %d, %d)\n", vdi_source, lna_gain, drssi_threshold, receiver_state);
    static const uint8_t RECEIVER_SETTING_COMMAND_HI = 0xc0;
    uint8_t lo = (((uint8_t)vdi_source) << 6)
                 | (((uint8_t)lna_gain) << 4)
                 | (((uint8_t)drssi_threshold) << 1)
                 | (((uint8_t)receiver_state));
    spi_send_2(RECEIVER_SETTING_COMMAND_HI, lo);
    _delay_us(7.0);
}

uint8_t alpha_rx_data_rate_to_cs_r(float data_rate) {
    uint8_t cs = 0; // cs can be either 0 or 1 - should really be a parameter or a loop to find best
    uint8_t r = (uint8_t) (((10e6 / 29 / (1 + cs*7)/data_rate) - 1) + 0.5);
    uint8_t cs_r = r | (cs << 7);
    return cs_r;
}

void alpha_rx_afc_command(
        enum AfcAutoMode afc_auto_mode,
        enum AfcRangeLimit afc_range_limit,
        enum AfcStoreOffset afc_store_offset,
        enum AfcAccuracy afc_hi_accuracy,
        enum AfcOutputRegister afc_output_register,
        enum AfcEnable afc_enable) {
    printf("alpha_rx_afc_command(%d, %d, %d, %d, %d, %d)\n",
           afc_auto_mode, afc_range_limit, afc_store_offset, afc_hi_accuracy, afc_output_register, afc_enable);
    static const uint8_t AFC_COMMAND_HI = 0xc6;
    uint8_t lo = (((uint8_t) afc_auto_mode) << 6)
                 | (((uint8_t) afc_range_limit) << 4)
                 | (((uint8_t) afc_store_offset) << 3)
                 | (((uint8_t) afc_hi_accuracy) << 2)
                 | (((uint8_t) afc_output_register) << 1)
                 | (((uint8_t) afc_enable));
    spi_send_2(AFC_COMMAND_HI, lo);
    _delay_us(7.0);
}

void alpha_rx_data_filter_command(
        enum ClockRecoveryAutoLock clock_recovery_auto_lock,
        enum ClockRecoveryMode clock_recovery_mode,
        enum FilterType filter_type,
        enum DqdThreshold dqd_threshold
) {
    printf("alpha_rx_data_filter_command(%d, %d, %d, %d)\n",
           clock_recovery_auto_lock, clock_recovery_mode, filter_type, dqd_threshold);
    static const uint8_t DATA_FILTER_COMMAND_HI = 0xc4;
    static const uint8_t DATA_FILTER_COMMAND_LO = 0x20;
    uint8_t lo = DATA_FILTER_COMMAND_LO
                 | (((uint8_t) clock_recovery_auto_lock) << 7)
                 | (((uint8_t) clock_recovery_mode) << 6)
                 | (((uint8_t) filter_type << 3));
    spi_send_2(DATA_FILTER_COMMAND_HI, lo);
    _delay_us(7.0);
}


void alpha_rx_data_rate_command(uint8_t cs_r) {
    printf("alpha_rx_data_rate_command(%" PRId8 ")\n", cs_r);
    static const uint8_t DATA_RATE_COMMAND_HI = 0xc8;
    spi_send_2(DATA_RATE_COMMAND_HI, cs_r);
    _delay_us(7.0);
}

void alpha_rx_output_and_fifo_command(
        uint8_t fifo_interrupt_level,
        enum FifoStartFillCondition fifo_start_fill_condition,
        bool fill_after_synchron_word,
        bool enable_16_bit_fifo_mode) {
    printf("alpha_rx_output_and_fifo_command(%" PRId8 ", %d, %d, %d)\n",
           fifo_interrupt_level, fifo_start_fill_condition, fill_after_synchron_word, enable_16_bit_fifo_mode);
    static const uint8_t FIFO_COMMAND_HI = 0xce;
    uint8_t lo = fifo_interrupt_level << 4
               | (((uint8_t)fifo_start_fill_condition) << 2)
               | (((uint8_t)fill_after_synchron_word) << 1)
               | (((uint8_t)enable_16_bit_fifo_mode));
    spi_send_2(FIFO_COMMAND_HI, lo);
    _delay_us(7.0);
}

void alpha_rx_reset_fifo_command(
        uint8_t fifo_interrupt_level,
        enum FifoStartFillCondition fifo_start_fill_condition) {
    printf("alpha_rx_reset_fifo_command(%" PRId8 ", %d)\n",
           fifo_interrupt_level, fifo_start_fill_condition);
    alpha_rx_output_and_fifo_command(fifo_interrupt_level, fifo_start_fill_condition, false, false);
    alpha_rx_output_and_fifo_command(fifo_interrupt_level, fifo_start_fill_condition, true, true);
}

void alpha_rx_reset() {
    printf("alpha_rx_reset()\n");
    spi_send_2(0xff, 0x00);
    _delay_ms(2000);
}

void alpha_rx_tune(int num_runs,
                   enum VdiSource vdi_source,
                   enum Band band,
                   bool enable_low_battery_detection,
                   bool enable_wake_up_timer,
                   bool enable_crystal_oscillator,
                   enum CrystalLoadCapacitor crystal_load_capacitor,
                   bool disable_clock_output) {
    for (int i = 0; i != num_runs; ++i) {
        printf("Run %d\n", i + 1);
        printf("%10s%10s", "LNA Gain", "DRSSI");
        for (int bandwidth = BASEBAND_BANDWIDTH_400kHz; bandwidth <= BASEBAND_BANDWIDTH_67kHz; ++bandwidth) {
            printf("%10s", BASEBAND_BANDWIDTH[bandwidth]);
        }
        putchar('\n');
        for (int drssi_threshold = DRSSI_MINUS_103_DBM; drssi_threshold <= DRSSI_MINUS_61_DBM; ++drssi_threshold) {
            for (int lna_gain_index = 0; lna_gain_index <= 3; ++lna_gain_index) {
                enum LnaGain lna_gain = LNA_GAIN_ORDERED[lna_gain_index];
                alpha_rx_receiver_setting_command(
                        vdi_source,
                        lna_gain,
                        (enum DrssiTheshold) drssi_threshold,
                        RECEIVER_ENABLE);
                printf("%10s%10s", LNA_GAIN[lna_gain], DRSSI_THESHOLD[drssi_threshold]);
                for (int bandwidth = BASEBAND_BANDWIDTH_400kHz; bandwidth <= BASEBAND_BANDWIDTH_67kHz; ++bandwidth) {
                    alpha_rx_configuration_setting_command(
                            band,
                            enable_low_battery_detection,
                            enable_wake_up_timer,
                            enable_crystal_oscillator,
                            crystal_load_capacitor,
                            (enum BasebandBandwidth) bandwidth,
                            disable_clock_output);
                    float frssi = sample_rssi(5000);
                    int rssi = (int) (frssi * 100.0f + 0.5f);
                    printf("%8d%% ", rssi);
                }
                putchar('\n');
            }
        }
    }
}

void alpha_rx_monitor_rssi(int num_periods, int num_times) {
    for (int i = 0; i < num_periods; ++i) {
        float frssi = sample_rssi(num_times);
        int rssi = (int) (frssi * 100.0f + 0.5f);
        printf("%d%%\n ", rssi);
    }
}

float sample_rssi(int num_times) {

    int rssi_count = 0;
    for (int i = 0; i < num_times; ++i) {
        uint16_t status = alpha_rx_get_status_command();
        if (status & 0x0400) {
            ++rssi_count;
        }
        _delay_us(100);
    }
    float proportion = (float)rssi_count / (float)num_times;
    return proportion;
}

