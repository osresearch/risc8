#include <avr/io.h>
#include <stdint.h>

static void __attribute__((__noinline__)) pwm(uint8_t b, uint8_t led)
{
	for(uint8_t i = 0 ; i < b ; i++)
		PORTB = led;

	for(uint8_t i = b ; i < 255 ; i++)
		PORTB = 0;
}

int main(void)
{
	uint8_t led = 1;
	while(1)
	{
		for(uint8_t b = 1 ; b < 64; b++)
		{
			for(uint8_t s = 0 ; s < 64 ; s++)
				pwm(b, led);
		}

		for(uint8_t b = 64 ; b > 0 ; b--)
		{
			for(uint8_t s = 0 ; s < 64 ; s++)
				pwm(b, led);
		}

		if (led == 4)
			led = 1;
		else
			led <<= 1;
		PINB = led;
	}
}
