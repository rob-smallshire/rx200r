#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
//#include <util/atomic.h>

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
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
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

static uint8_t buffer[100];


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main (void)
{
    sei();

    stdin = stdout = &uart0_stream;

    // USB Serial 0
    uart0_init(UART_BAUD_SELECT(9600, F_CPU));

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

    _delay_us(7.0);

    //alpha_rx_reset();  // Also resets the AVR on which this code is running!

    //_delay_ms(2000);

    green_led_on();

    alpha_rx_configuration_setting_command(
            BAND_433_MHz,
            false,
            false,
            true,
            XTAL_LOAD_CAP_12p5,
            BASEBAND_BANDWIDTH_200kHz,
            true);

    _delay_us(7.0);

    uint16_t f = alpha_rx_frequency_to_f(BAND_433_MHz, 434.2f);
    printf("f = %hu\n", f);
    alpha_rx_frequency_setting_command(f);

    _delay_us(7.0);

    uint8_t cs_r = alpha_rx_data_rate_to_cs_r(34482.75862068966);
    printf("cs_r = %hu\n", cs_r);
    alpha_rx_data_rate_command(cs_r);

    _delay_us(7.0);

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

    _delay_us(7.0);

    alpha_rx_data_filter_command(
            CLOCK_RECOVERY_MANUAL,
            CLOCK_RECOVERY_FAST_MODE,
            DATA_FILTER_DIGITAL,
            DQD_2
    );

    _delay_us(7.0);

    alpha_rx_receiver_setting_command(
            VDI_CLOCK_RECOVERY_LOCK_OUTPUT,
            LNA_GAIN_MINUS_6_DBM,
            DRSSI_MINUS_103_DBM,
            RECEIVER_DISABLE);

    _delay_us(7.0);

    // TODO: Try to disable weird reset mode
    spi_send_2(0xda, 0x01);

    _delay_us(7.0);

    alpha_rx_reset_fifo_command(8, FIFO_START_FILL_ON_SYNC_WORD);

    _delay_us(7.0);

    alpha_rx_receiver_setting_command(
            VDI_CLOCK_RECOVERY_LOCK_OUTPUT,
            LNA_GAIN_MINUS_6_DBM,
            DRSSI_MINUS_103_DBM,
            RECEIVER_ENABLE);

    _delay_ms(5);

    alpha_rx_get_status_command();

    _delay_us(7.0);

    alpha_rx_get_status_command();

    _delay_ms(5);

    int index = 0;

    while (1) {
        green_led_on();
        _delay_ms(1000);
        green_led_off();
        red_led_on();
        _delay_ms(1000);
        red_led_off();
    }

    while (1) {
        //if (!test_nirq_interrupt()) {
            spi_select_slave();
            //bool full = false; // is the buffer full after receiving the byte waiting for us?
            const uint8_t status_hi = spi_receive(); // get status word MSB
            const uint8_t status_lo = spi_receive(); // get status word LSB
            printf("%d %d\n", status_hi, status_lo);

            if ((status_hi & 0x40) != 0) { // FIFO overflow
                printf("FIFO overflow\n");
                //full  = PACKET_BUFFER.add(data_1);
                //full |= PACKET_BUFFER.add(spi_transfer_byte(0x00));
                spi_receive();
            } else if ((status_hi & 0x80) != 0) { // FIFO has 8 bits ready
                printf("FIFO ready\n");
                const uint8_t data_1 = spi_receive(); // get 1st byte of data
                //full = PACKET_BUFFER.add(data_1);
                buffer[index] = data_1;
                ++index;
            }
            spi_deselect_slave();

            _delay_us(7);

            if (index == 10) {
                alpha_rx_reset_fifo_command(8, FIFO_START_FILL_ON_SYNC_WORD);
            }

            _delay_us(7);
        //}
    }

//    while (1) {
//        uint8_t value = 0;
//        if (test_fifo_interrupt()) {
//            disable_spi();
//            select_fifo();
//            uint8_t bit_counter = 0;
//            while(test_fifo_interrupt()) {
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
//    }
}

#pragma clang diagnostic pop
