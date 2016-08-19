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


enum {
 BLINK_DELAY_MS = 1000,
};





#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main (void)
{
    sei();

    stdin = stdout = &uart0_stream;

    // USB Serial 0
    uart0_init(UART_BAUD_SELECT(9600, F_CPU));

    DDRB |= _BV(DDB0); /* Green LED - set bit 0 of PORTB for output */
    DDRB |= _BV(DDB1); /* Red LED   - set bit 1 of PORTB for output */
    DDRB |= _BV(DDB2); /* nSS       - set bit 2 of PORTB for output */
    DDRB |= _BV(DDB3); /* MOSI      - set bit 3 of PORTB for output */
                       /* MISO      - set bit 4 of PORTB for input  */
    DDRB |= _BV(DDB5); /* SCK       - set bit 5 of PORTB for output */
    /* Bits 6 and 7 of PORTB are unavailble - pins used by crystal clock */

    /* Setup SPI */
    spi_deselect_slave();

    /* SPCR - SPI Control Register */
    /* CPOL = 0 - Clock Polarity - idle clock is low */
    /* CPHA = 0 - Clock Phase    - data on rising edge */

    /* Enable SPI, Master, MSB first, set clock rate fck/64 */
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
    _delay_ms(100);
    alpha_tx_configuration_setting_command(BAND_433_MHz, false, false, true, XTAL_LOAD_CAP_12p0, BASEBAND_BANDWIDTH_200kHz, true);
    _delay_us(7.0);
    uint16_t f = alpha_rx_frequency_to_f(BAND_433_MHz, 434.2f);
    printf("f = %hu\n", f);
    alpha_rx_frequency_setting_command(f);
    _delay_us(7.0);
    alpha_rx_receiver_setting_command(
            VDI_DRSS_AND_DATA_QUALITY_DETECTOR,
            LNA_GAIN_0_DBM,
            DRSSI_MINUS_79_DBM,
            RECEIVER_ENABLE);
    _delay_us(7.0);
    uint8_t cs_r = alpha_rx_data_rate_to_cs_r(34482.75862068966);
    printf("cs_r = %hu\n", cs_r);
    alpha_rx_data_rate_command(cs_r);

    while (1) {
        /* set pin 20 low to turn led on */
        PORTB &= ~_BV(PORTB0);
        _delay_ms(BLINK_DELAY_MS);

        /* set pin 0 high to turn led off */
        PORTB |= _BV(PORTB0);
        _delay_ms(BLINK_DELAY_MS);

        /* set pin 20 low to turn led on */
        PORTB &= ~_BV(PORTB1);
        _delay_ms(BLINK_DELAY_MS);

        /* set pin 20 high to turn led off */
        PORTB |= _BV(PORTB1);
        _delay_ms(BLINK_DELAY_MS);

        alpha_tx_get_status_command();
        printf("Hello, World! at rate %d\n", uart0_get_baud_rate());
    }
}

#pragma clang diagnostic pop
