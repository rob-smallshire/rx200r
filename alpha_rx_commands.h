//
// Created by rjs on 17.08.16.
//

#ifndef RX200R_ALPHA_RX_COMMANDS_H
#define RX200R_ALPHA_RX_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

enum Band {
    BAND_315_MHz = 0,
    BAND_433_MHz = 1,
    BAND_868_MHz = 2,
    BAND_915_MHz = 3
};

enum CrystalLoadCapacitor {
    XTAL_LOAD_CAP_8p5 = 0,
    XTAL_LOAD_CAP_9p0 = 1,
    XTAL_LOAD_CAP_9p5 = 2,
    XTAL_LOAD_CAP_10p0 = 3,
    XTAL_LOAD_CAP_10p5 = 4,
    XTAL_LOAD_CAP_11p0 = 5,
    XTAL_LOAD_CAP_11p5 = 6,
    XTAL_LOAD_CAP_12p0 = 7,
    XTAL_LOAD_CAP_12p5 = 8,
    XTAL_LOAD_CAP_13p0 = 9,
    XTAL_LOAD_CAP_13p5 = 10,
    XTAL_LOAD_CAP_14p0 = 11,
    XTAL_LOAD_CAP_14p5 = 12,
    XTAL_LOAD_CAP_15p0 = 13,
    XTAL_LOAD_CAP_15p5 = 14,
    XTAL_LOAD_CAP_16p0 = 15
};
enum BasebandBandwidth {
    BASEBAND_BANDWIDTH_400kHz = 1,
    BASEBAND_BANDWIDTH_340kHz = 2,
    BASEBAND_BANDWIDTH_270kHz = 3,
    BASEBAND_BANDWIDTH_200kHz = 4,
    BASEBAND_BANDWIDTH_134kHz = 5,
    BASEBAND_BANDWIDTH_67kHz = 6
};

uint16_t alpha_tx_get_status_command();

void alpha_tx_configuration_setting_command(
        enum Band band,
        bool enable_low_battery_detection,
        bool enable_wake_up_timer,
        bool enable_crystal_oscillator,
        enum CrystalLoadCapacitor crystal_load_capacitor,
        enum BasebandBandwidth baseband_bandwidth,
        bool disable_clock_output);

static const int ALPHA_RX_CLOCK_MHZ = 10;

uint16_t alpha_rx_frequency_to_f(enum Band band, float frequency);

void alpha_rx_frequency_setting_command(uint16_t f);

enum VdiSource {
    VDI_DRSSI = 0,
    VDI_DATA_QUALITY_DETECTOR = 1,
    VDI_CLOCK_RECOVERY_LOCK_OUTPUT = 2,
    VDI_DRSS_AND_DATA_QUALITY_DETECTOR = 3
};

enum LnaGain {
    LNA_GAIN_0_DBM = 0,
    LNA_GAIN_MINUS_14_DBM = 1,
    LNA_GAIN_MINUS_6_DBM = 2,
    LNA_GAIN_MINUS_20_DBM = 3
};

enum DrssiTheshold {
    DRSSI_MINUS_103_DBM = 0,
    DRSSI_MINUS_97_DBM = 1,
    DRSSI_MINUS_91_DBM = 2,
    DRSSI_MINUS_85_DBM = 3,
    DRSSI_MINUS_79_DBM = 4,
    DRSSI_MINUS_73_DBM = 5,
    DRSSI_MINUS_67_DBM = 6,
    DRSSI_MINUS_61_DBM = 7
};

enum ReceiverState {
    RECEIVER_DISABLE = 0,
    RECEIVER_ENABLE = 1
};

void alpha_rx_receiver_setting_command(
        enum VdiSource vdi_source,
        enum LnaGain lna_gain,
        enum DrssiTheshold drssi_threshold,
        enum ReceiverState receiver_state
);

#endif //RX200R_ALPHA_RX_COMMANDS_H
