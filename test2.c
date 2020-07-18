typedef unsigned char uint8_t;

static const char msg[] = "Hello, world!";

void main(void)
{
	*(volatile uint8_t*) 0x1234 = 0xAB;

	const char * c = msg;
	while(*c != '\0')
		*(volatile uint8_t *) 0x5aa5 = *c++;

	while(1);
}
