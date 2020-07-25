
all:

TEST-y += test-risc8
TEST-y += test-reg

ice40-risc8.bit: program.hex

include Makefile.icestorm

CROSS ?= avr-

CFLAGS ?= \
	-O3 \
	-Wall \
	-mmcu=attiny85 \

NO=\
	-Wl,-T,sections.lds \
	-ffreestanding \
	-nostdinc \
	-nostdlib \

%.bin: %.elf
	$(CROSS)objcopy -Obinary $< $@
%.elf: %.o
	$(CROSS)gcc $(CFLAGS) -o $@ $^ \

%.o: %.c
	$(CROSS)gcc $(CFLAGS) -c -o $@ $^
%.elf: %.S
	$(CROSS)gcc $(CFLAGS)  -o $@ $^ \
	-ffreestanding \
	-nostdinc \
	-nostdlib \

%.hex: %.bin
	xxd -g2 -c2 -e $< | cut -d' ' -f2 > $@
