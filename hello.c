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

int main(void)
{
	uint8_t cycle = 0;
	static const char hexdigit[16] = "0123456789abcdef";

	while(1)
	{
		PORTB = 1;
		uart_putc(hexdigit[(cycle >> 4) & 0xF]);
		uart_putc(hexdigit[(cycle >> 0) & 0xF]);
		PORTB = 0;
		uart_puts(" Hello world\r\n");
		cycle++;
	}
}
