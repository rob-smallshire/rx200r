#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
//#include <util/atomic.h>

#include "uart.h"
#include "twiddle.h"

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


void get_status_command();

enum {
 BLINK_DELAY_MS = 1000,
};


uint8_t spi_send_receive(uint8_t data) {
    // transmit the byte to be sent
    SPDR = data;
    // wait for the transfer to complete
    while (!(SPSR & (1<<SPIF)));
    // then return the byte the slave just returned
    return SPDR;
}


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
    /* SPCR - SPI Control Register */
    /* CPOL = 0 - Clock Polarity - idle clock is low */
    /* CPHA = 0 - Clock Phase    - data on rising edge */

    /* Enable SPI, Master, MSB first, set clock rate fck/64 */
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);

    // Send Read Configuration register



    _delay_ms(100);

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

        get_status_command();
        printf("Hello, World! at rate %d\n", uart0_get_baud_rate());
    }
}

void get_status_command() {
    static const uint8_t GET_STATUS_COMMAND_HI = 0x00;
    static const uint8_t GET_STATUS_COMMAND_LO = 0x00;
    CLR(PORTB, DDB5);
    _delay_us(5.0);
    uint8_t status_hi = spi_send_receive(GET_STATUS_COMMAND_HI);
    _delay_us(7.0);
    uint8_t status_lo = spi_send_receive(GET_STATUS_COMMAND_LO);
    _delay_us(5.0);
    SET(PORTB, DDB5);

    printf("(0x%04X 0x%04X)\n", status_hi, status_lo);
}

#pragma clang diagnostic pop
