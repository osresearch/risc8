#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>

void uart_putc(char c)
{
	while((USISR & 1) == 0)
		;
	USIDR = c;
}

void uart_puts(const char * s)
{
	while(*s)
		uart_putc(*s++);
}

static void __attribute__((__noinline__)) pwm(uint8_t b, uint8_t led)
{
	for(uint8_t i = 0 ; i < b ; i++)
		PORTB = led;

	for(uint8_t i = b ; i < 255 ; i++)
		PORTB = 0;
}

static char hexdigit(uint8_t x)
{
	static const char PROGMEM hexdigit[16] = "0123456789abcdef";
	return pgm_read_byte_near(hexdigit + (x & 0xF));
}

int main(void)
{
	uint16_t cycle = 0;

	while(1)
	{
		uart_putc(hexdigit(cycle >> 12));
		uart_putc(hexdigit(cycle >>  8));
		uart_putc(hexdigit(cycle >>  4));
		uart_putc(hexdigit(cycle >>  0));
		pwm(cycle >> 4, cycle >> 12);

		uart_puts(" Hello world\r\n");
		cycle++;
	}
}
