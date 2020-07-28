#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdio.h>

void uart_putc(char c)
{
	while((USISR & 1) == 0)
		;
	for(uint16_t i = 0 ; i < 100 ; i++)
		asm("nop");
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

static char __attribute__((__noinline__)) hexdigit(uint8_t x)
{
	static const char PROGMEM hexdigit[16] = "0123456789abcdef";
	return pgm_read_byte_near(hexdigit + (x & 0xF));
}

int main(void)
{
	uint16_t cycle = 0xFADE;

	*(volatile uint8_t*) 0x20 = hexdigit(cycle >> 4);
	*(volatile uint8_t*) 0x20 = hexdigit(cycle >> 0);

	while(1)
	{
		uint8_t bright = cycle >> 4;
		if (bright & 0x80)
			bright = ~bright;
		pwm(bright, cycle >> 12);

		cycle++;
		if ((cycle & 0xFFF) != 0)
			continue;

		uart_putc(hexdigit(bright >> 4));
		uart_putc(hexdigit(bright >> 0));
		uart_putc(' ');
		uart_putc(hexdigit(cycle >> 12));
		uart_putc(hexdigit(cycle >>  8));
		uart_putc(hexdigit(cycle >>  4));
		uart_putc(hexdigit(cycle >>  0));
		uart_puts(" Hello world\r\n");
	}
}
