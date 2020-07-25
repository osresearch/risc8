#include <avr/io.h>
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


int main(void)
{
	uint16_t cycle = 0;
	static const char hexdigit[16] = "0123456789abcdef";

	while(1)
	{
		uart_putc(hexdigit[(cycle >> 12) & 0xF]);
		uart_putc(hexdigit[(cycle >>  8) & 0xF]);
		uart_putc(hexdigit[(cycle >>  4) & 0xF]);
		uart_putc(hexdigit[(cycle >>  0) & 0xF]);
		pwm(cycle >> 4, 1);

		uart_puts(" Hello world\r\n");
		cycle++;
	}
}
