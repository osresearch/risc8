# RISC-8 softcore

The RISC-8 is a small 8-bit CPU that is mostly compatible with the
[AVR instruction set](http://ww1.microchip.com/downloads/en/devicedoc/atmel-0856-avr-instruction-set-manual.pdf).
It does not have 100% of the architecture implemented, although
enough to run many `avr-gcc` compiled programs in the soft-core.

Sometimes you want all the power of a 32-bit RISC-V like Claire's
[picorv32](https://github.com/cliffordwolf/picorv32), although sometimes
you want faster synthesis times (9 seconds vs 45 seconds) and to use
fewer FPGA resources (1200 LC vs 4500 LC), and the 8-bit CPU isn't a limitation.

## CPU overview

The CPU has a two stage pipeline (instruction decode and register fetch, operation and
register write) and can retire one instruction per clock.
Most instructions are single cycle, with some complex instructions like `IN`/`OUT`,
`LD`/`ST` and `CALL`/`RET` that require multiple cycles.

On an ice40up5k it uses approximately 1400 LC and can be run at 12 MHz.
64 KB of the SPRAM can be used for the SOC's data memory, although the
program memory has to be in DPRAM so that it can be stored in the bitstream.

The two stage pipeline allows the register file to be stored in block RAM,
which greatly reducing the number of logic cells required.  The register
file has a forward-feed to support immediate use-after-write with no
wait states.

## Limitations

* No interrupts
* No `SPM` instruction support
* Not very many built in peripherals
* Probably full of bugs

## Examples

## Instruction set

