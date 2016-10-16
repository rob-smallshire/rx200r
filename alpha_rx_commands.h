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
    BASEBAND_BANDWIDTH_RESERVED = 0,
    BASEBAND_BANDWIDTH_400kHz = 1,
    BASEBAND_BANDWIDTH_340kHz = 2,
    BASEBAND_BANDWIDTH_270kHz = 3,
    BASEBAND_BANDWIDTH_200kHz = 4,
    BASEBAND_BANDWIDTH_134kHz = 5,
    BASEBAND_BANDWIDTH_67kHz = 6
};

extern const char* BASEBAND_BANDWIDTH[7];

uint16_t alpha_rx_get_status_command();

void alpha_rx_configuration_setting_command(
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

extern const char* LNA_GAIN[4];

extern const enum LnaGain LNA_GAIN_ORDERED[4];

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

extern const char* DRSSI_THESHOLD[8];

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

enum AfcAutoMode {
    AFC_AUTO_MODE_CONTROLLED_BY_MCU = 0,
    AFC_AUTO_MODE_RUN_ONCE_AT_POWER_ON = 1,
    AFC_AUTO_MODE_KEEP_OFFSET_WHEN_VDI_HI = 2,
    AFC_AUTO_MODE_KEEPS_INDEPENDENTLY_FROM_VDI = 3
};

enum AfcRangeLimit {
    AFC_RANGE_LIMIT_NO_RESTRICTION = 0,
    AFC_RANGE_LIMIT_PLUS_15_MINUS_16 = 1,
    AFC_RANGE_LIMIT_PLUS_7_MINUS_8 = 2,
    AFC_RANGE_LIMIT_PLUS_3_MINUS_4 = 3
};

enum AfcStoreOffset {
    AFC_DISABLE_STORE_OFFSET = 0,
    AFC_ST_GOES_HI_WILL_STORE_OFFSET = 1
};

enum AfcAccuracy {
    AFC_HI_ACCURACY_DISABLE = 0,
    AFC_HI_ACCURACY_ENABLE = 1,
};

enum AfcOutputRegister {
    AFC_OUTPUT_REGISTER_DISABLE = 0,
    AFC_OUTPUT_REGISTER_ENABLE = 1
};

enum AfcEnable {
    AFC_DISABLE = 0,
    AFC_ENABLE = 1
};

void alpha_rx_afc_command(
        enum AfcAutoMode afc_auto_mode,
        enum AfcRangeLimit afc_range_limit,
        enum AfcStoreOffset afc_store_offset,
        enum AfcAccuracy afc_hi_accuracy,
        enum AfcOutputRegister afc_output_register,
        enum AfcEnable afc_enable);

uint8_t alpha_rx_data_rate_to_cs_r(float data_rate);

void alpha_rx_data_rate_command(uint8_t cs_r);

enum ClockRecoveryAutoLock {
    CLOCK_RECOVERY_MANUAL = 0,
    CLOCK_RECOVERY_AUTO_LOCK = 1
};

enum ClockRecoveryMode {
    CLOCK_RECOVERY_SLOW_MODE = 0,
    CLOCK_RECOVERY_FAST_MODE = 1
};

enum FilterType {
    DATA_FILTER_OOK = 0,
    DATA_FILTER_DIGITAL = 1,
    DATA_FILTER_RESERVED = 2
};

enum DqdThreshold {
    DQD_0 = 0,
    DQD_1 = 1,
    DQD_2 = 2,
    DQD_3 = 3,
    DQD_4 = 4,
    DQD_5 = 5,
    DQD_6 = 6,
    DQD_7 = 7
};

void alpha_rx_data_filter_command(
  enum ClockRecoveryAutoLock clock_recovery_auto_lock,
  enum ClockRecoveryMode clock_recovery_mode,
  enum FilterType filter_type,
  enum DqdThreshold dqd_threshold
);

enum FifoStartFillCondition {
    FIFO_START_FILL_ON_VALID_DATA_INDICATOR = 0,
    FIFO_START_FILL_ON_SYNC_WORD = 1,
    FIFO_START_FILL_ON_VALID_DATA_INDICATOR_AND_SYNC_WORD = 2,
    FIFO_START_FILL_ALWAYS = 3
};

void alpha_rx_output_and_fifo_command(
        uint8_t fifo_interrupt_level,
        enum FifoStartFillCondition fifo_start_fill_condition,
        bool fill_after_synchron_word,
        bool enable_16_bit_fifo_mode);

void alpha_rx_reset_fifo_command(
        uint8_t fifo_interrupt_level,
        enum FifoStartFillCondition fifo_start_fill_condition);

void alpha_rx_reset();

void alpha_rx_tune(int num_runs,
                   enum VdiSource vdi_source,
                   enum Band band,
                   bool enable_low_battery_detection,
                   bool enable_wake_up_timer,
                   bool enable_crystal_oscillator,
                   enum CrystalLoadCapacitor crystal_load_capacitor,
                   bool disable_clock_output);

void alpha_rx_monitor_rssi(int num_periods, int num_times);

float sample_rssi(int num_times);

#endif //RX200R_ALPHA_RX_COMMANDS_H
