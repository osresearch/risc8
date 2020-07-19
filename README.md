FPGA AVR
---

This is a small 8-bit CPU that is mostly compatible with the
[AVR instruction set](http://ww1.microchip.com/downloads/en/devicedoc/atmel-0856-avr-instruction-set-manual.pdf).
It does not have 100% of the architecture implemented, although
enough to run `avr-gcc` compiled programs in the soft-core.

Most instructions are single cycle, with a `LD`/`ST` and `CALL`/`RET`
instructions that require multiple cycles.  On an iCE40 it runs around
12 MHz.

Rough resource utiliziation on an ice40up5k is 2000 LUTs.

