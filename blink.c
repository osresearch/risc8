#include <avr/io.h>
#include <stdint.h>

int main(void)
{
	uint8_t cycle = 0;

	while(1)
	{
		PORTB = cycle;
		cycle = (cycle + 1) & 7;

		for(unsigned i = 0 ; i < 60000  ; i++)
		{
			asm("nop\n nop\n nop\n nop");
			asm("nop\n nop\n nop\n nop");
			asm("nop\n nop\n nop\n nop");
			asm("nop\n nop\n nop\n nop");
		}
	}
}
