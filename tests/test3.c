#include <stdint.h>

//                         123456789abcd
static const char msg[] = "Hello, world!";

//int __attribute__((__noinline__)) my_strlen(const char * s)
int my_strlen(const char * s)
{
	int rc = 0;
	while(*s++)
		rc++;
	return rc;
}

void main(void)
{
	*(volatile uint8_t*) 0x1234 = 0xAB;

	*(volatile uint8_t *) 0x5aa5 = my_strlen(msg);

	while(1);
}
