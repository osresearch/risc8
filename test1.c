typedef unsigned char uint8_t;

uint8_t __attribute__((__noinline__)) foo(uint8_t n)
{
	uint8_t rc = 0;
	for(uint8_t i = 0 ; i < n ; i++)
		rc += i;
	return rc;
}

void __attribute__((__section__(".text.entry"))) _start(void)
{
	*(volatile uint8_t *) 0x100 = foo(5);
	while(1)
		;
}
