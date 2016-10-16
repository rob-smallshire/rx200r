#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <util/atomic.h>

#include "uart.h"
#include "alpha_rx_commands.h"
#include "spi.h"

#ifdef __GNUC__
#  define UNUSED(x) UNUSED_ ## x __attribute__((__unused__))
#else
#  define UNUSED(x) UNUSED_ ## x
#endif

#ifdef __GNUC__
#  define UNUSED_FUNCTION(x) __attribute__((__unused__)) UNUSED_ ## x
#else
#  define UNUSED_FUNCTION(x) UNUSED_ ## x
#endif

uint16_t CTC_MATCH_OVERFLOW = ((F_CPU / 1000) / 8);

volatile unsigned long timer1_millis;
long milliseconds_since;

ISR (TIMER1_COMPA_vect)
{
    timer1_millis++;
}

unsigned long millis()
{
    unsigned long millis_return = 0;

    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        millis_return = timer1_millis;
    }

    return millis_return;
}

void init_millis() {
    // CTC mode, Clock/8
    TCCR1B |= (1 << WGM12) | (1 << CS11);

    // Load the high byte, then the low byte
    // into the output compare
    OCR1AH = (CTC_MATCH_OVERFLOW >> 8);
    OCR1AL = CTC_MATCH_OVERFLOW;

    // Enable the compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
}

int uart0_send_byte(char data, FILE* UNUSED(stream))
{
    if (data == '\n')
    {
        uart0_putc('\r');
    }
    uart0_putc((uint8_t)data);
    return 0;
}

int uart0_receive_byte(FILE* UNUSED(stream))
{
    uint8_t data = uart0_getc();
    return data;
}

static FILE uart0_stream = FDEV_SETUP_STREAM(
        uart0_send_byte,
        uart0_receive_byte,
        _FDEV_SETUP_RW);


void green_led_off();

void red_led_off();

void deselect_fifo();

void enable_spi();

void disable_spi();

uint8_t test_fifo_interrupt();

void select_fifo();

void sck_lo();

void sck_hi();

enum {
 BLINK_DELAY_MS = 1000,
};

uint8_t test_fifo_interrupt() {
    return TST(PINC, PINC1);
}

uint8_t test_nirq_interrupt() {
    return TST(PINC, PINC3);
}

void enable_spi() {
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // fOSC / 16
}

void disable_spi() {
    CLR(SPCR, SPE);
}

void sck_hi() {
    SET(PORTB, PORTB5);
}

void sck_lo() {
    CLR(PORTB, PORTB5);
}

void select_fifo() {
    CLR(PORTC, PORTC2);
}

void deselect_fifo() {
    SET(PORTC, PORTC2); // Deselect FIFO
}

void red_led_off() {
    SET(PORTB, PORTB1); // Red LED off
}

void green_led_off() {
    SET(PORTB, PORTB0); // Green LED off
}

void red_led_on() {
    CLR(PORTB, PORTB1); // Red LED off
}

void green_led_on() {
    CLR(PORTB, PORTB0); // Green LED off
}

#define BUFFER_LENGTH 256

static uint8_t buffer[BUFFER_LENGTH];


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main (void)
{
    init_millis();

    sei();

    stdin = stdout = &uart0_stream;

    // USB Serial 0
    uart0_init(UART_BAUD_SELECT(9600, F_CPU));

    printf("\n\nStart!\n");

    DDRB = 0;
    SET(DDRB, DDB0);  // Green LED - configure bit 0 of PORTB for output
    SET(DDRB, DDB1);  // Red LED   - configure bit 1 of PORTB for output
    SET(DDRB, DDB2);  // nSS       - configure bit 2 of PORTB for output
    SET(DDRB, DDB3);  // MOSI      - configure bit 3 of PORTB for output
    CLR(DDRB, DDB4);  // MISO      - configure bit 4 of PORTB for input
    SET(DDRB, DDB5);  // SCK       - configure bit 5 of PORTB for output
    // Bits 6 and 7 of PORTB are unavailable - pins used by crystal clock

    DDRC = 0;
    CLR(DDRC, DDC0);  // VDI  - configure bit 0 of PORTC for input
    CLR(DDRC, DDC1);  // FFIT - configure bit 1 of PORTC for input
    SET(DDRC, DDC2);  // nFFS - configure bit 2 of PORTC for output
    CLR(DDRC, DDC3);  // nIRQ - configure bit 3 of PORTC for input
    // Bits 4 to 7 of PORTC are unavailable

    green_led_off();
    red_led_off();

    deselect_fifo();
    SET(PORTC, PORTC3); // Pull-up on nIRQ

    /* Setup SPI */
    spi_deselect_slave();

    /* SPCR - SPI Control Register */
    /* CPOL = 0 - Clock Polarity - idle clock is low */
    /* CPHA = 0 - Clock Phase    - data on rising edge */

    /* Enable SPI, Master, MSB first, set clock rate fck/64 */
    enable_spi();
    _delay_ms(100);

//    CLR(PORTB, PORTB1);
//    while (!TST(PINC, DDC3)) {
//        uint16_t status = alpha_rx_get_status_command();
//        printf("status = 0x%hx\n", status);
//    }
//    SET(PORTB, PORTB1);

    alpha_rx_get_status_command();

    alpha_rx_reset();  // Also resets the AVR on which this code is running!

    green_led_off();
    red_led_off();

    const enum Band band = BAND_433_MHz;
    const bool enable_low_battery_detection = false;
    const bool enable_wake_up_timer = false;
    const bool enable_crystal_oscillator = true;
    const enum CrystalLoadCapacitor crystal_load_capacitor = XTAL_LOAD_CAP_12p0;
    const enum BasebandBandwidth baseband_bandwidth = BASEBAND_BANDWIDTH_67kHz;
    const bool disable_clock_output = true;

    alpha_rx_configuration_setting_command(
            band,
            enable_low_battery_detection,
            enable_wake_up_timer,
            enable_crystal_oscillator,
            crystal_load_capacitor,
            baseband_bandwidth,
            disable_clock_output);

    uint16_t f = alpha_rx_frequency_to_f(band, 434.2f);
    printf("f = %hu\n", f);
    alpha_rx_frequency_setting_command(f);

    uint8_t cs_r = alpha_rx_data_rate_to_cs_r(17238);
    printf("cs_r = %hu\n", cs_r);
    alpha_rx_data_rate_command(cs_r);

    // RFM01 command #3 CC0E (6. low duty-cycle command)
    // en = 0: disable low duty cycle mode
    spi_send_2(0xcc, 0x0e);

    _delay_us(7.0);

    alpha_rx_afc_command(
            AFC_AUTO_MODE_KEEP_OFFSET_WHEN_VDI_HI,
            AFC_RANGE_LIMIT_PLUS_15_MINUS_16,
            AFC_ST_GOES_HI_WILL_STORE_OFFSET,
            AFC_HI_ACCURACY_ENABLE,
            AFC_OUTPUT_REGISTER_ENABLE,
            AFC_ENABLE
    );

    alpha_rx_data_filter_command(
            CLOCK_RECOVERY_AUTO_LOCK,
            CLOCK_RECOVERY_FAST_MODE,
            DATA_FILTER_DIGITAL,
            DQD_4
    );

    const enum VdiSource vdi_data_quality_detector = VDI_DRSSI;
    const enum LnaGain lna_gain = LNA_GAIN_MINUS_20_DBM;
    const enum DrssiTheshold drssi_threshold = DRSSI_MINUS_85_DBM;

    alpha_rx_receiver_setting_command(
            vdi_data_quality_detector,
            lna_gain,
            drssi_threshold,
            RECEIVER_DISABLE);

    // TODO: Try to disable weird reset mode
    spi_send_2(0xda, 0x01);

    _delay_us(7.0);

    enum FifoStartFillCondition start_fifo_fill = FIFO_START_FILL_ON_VALID_DATA_INDICATOR;
    uint8_t fifo_interrupt_level = 8;
    alpha_rx_reset_fifo_command(fifo_interrupt_level, start_fifo_fill);

    _delay_us(7.0);

    alpha_rx_receiver_setting_command(
            vdi_data_quality_detector,
            lna_gain,
            drssi_threshold,
            RECEIVER_ENABLE);

    alpha_rx_get_status_command();
    alpha_rx_get_status_command();

    green_led_on();

//    alpha_rx_tune(
//            -1,
//            vdi_data_quality_detector,
//            band,
//            enable_low_battery_detection,
//            enable_wake_up_timer,
//            enable_crystal_oscillator,
//            crystal_load_capacitor,
//            disable_clock_output);

//    alpha_rx_monitor_rssi(1000, 1000);

    red_led_on();
    green_led_off();

//    while (1) {
//        red_led_on();
//        _delay_ms(1000);
//        red_led_off();
//        _delay_ms(1000);
//    }


    alpha_rx_reset_fifo_command(fifo_interrupt_level, start_fifo_fill);

    _delay_us(7.0);
    unsigned long most_recent_millis = 0;
    bool in_packet = false;
    int packet_index = 0;
    while (1) {
        //if (!test_nirq_interrupt()) {
            unsigned long now = millis();
            spi_select_slave();
            //bool full = false; // is the buffer full after receiving the byte waiting for us?
            const uint8_t status_hi = spi_receive(); // get status word MSB
            /*const uint8_t status_lo = */ spi_receive(); // get status word LSB
            //printf("0x%02x 0x%02x\n", status_hi, status_lo);
            //printf("%lu\n", millis());
            if ((status_hi & 0x40) != 0) { // FIFO overflow
                //printf("FIFO overflow\n");
                red_led_on();
                //full  = PACKET_BUFFER.add(data_1);
                //full |= PACKET_BUFFER.add(spi_transfer_byte(0x00));
                //spi_receive();
                in_packet = false;
                //packet_index = 0;
            }
            else {
                red_led_off();
            }

            if ((status_hi & 0x80) != 0) { // FIFO has 8 bits ready
                //printf("FIFO ready\n");
                //in_packet = true;
                const uint8_t data_1 = spi_receive(); // get next byte of data
                //printf("%02x ", data_1);
                //full = PACKET_BUFFER.add(data_1);
                buffer[packet_index] = data_1;
                ++packet_index;
                in_packet = true;
                most_recent_millis = now;
            }
            else {
                //green_led_off();
            }

            spi_deselect_slave();

            unsigned long elapsed_since_most_recent = now - most_recent_millis;
            //printf("%lu\n", elapsed_since_most_recent);
            if (in_packet && elapsed_since_most_recent > 100) {
                // Packet over
                in_packet = false;
                alpha_rx_reset_fifo_command(8, start_fifo_fill);
                for (int p = 0; p < packet_index; ++p) {
                    printf("%02x", buffer[p]);
                }
                printf("  [%d]\n", packet_index);
                packet_index = 0;
            }

            if (in_packet) {
                green_led_on();
            }
            else {
                green_led_off();
            }

            if (packet_index == BUFFER_LENGTH) {
                printf("Buffer overflow!");
                packet_index = 0;
            }

            _delay_us(7);
        //}
    }

//    while (1) {
//        uint8_t value = 0;
//        if (test_fifo_interrupt()) {
//            green_led_on();
//            disable_spi();
//            select_fifo();
//            uint8_t bit_counter = 0;
//            while (test_fifo_interrupt()) {
//                _delay_us(2);
//                sck_lo();
//                _delay_us(2);
//                sck_hi();
//                uint8_t bit = TST(PORTB, PINB4) >> 4;
//                value <<= 1;
//                value |= bit;
//                ++bit_counter;
//                if (bit_counter == 8) {
//                    break;
//                }
//            }
//            deselect_fifo();
//            enable_spi();
//        }
//        else {
//            green_led_off();
//        }
//    }
}

#pragma clang diagnostic pop
