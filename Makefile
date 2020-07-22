
all: top.bin

TEST-y += test-risc8

include Makefile.icestorm

CROSS ?= avr-
#CROSS ?= /home/hudson/bin/arduino-1.8.10/hardware/tools/avr/bin/avr-

CFLAGS ?= \
	-O3 \
	-Wall \
	-mmcu=attiny85 \

NO=\

%.bin: %.elf
	$(CROSS)objcopy -Obinary $< $@
%.elf: %.o
	$(CROSS)gcc $(CFLAGS) -o $@ $^ \
	-Wl,-T,sections.lds \
	-ffreestanding \
	-nostdinc \
	-nostdlib \

%.o: %.c
	$(CROSS)gcc $(CFLAGS) -c -o $@ $^
%.elf: %.S
	$(CROSS)gcc $(CFLAGS)  -o $@ $^ \
	-ffreestanding \
	-nostdinc \
	-nostdlib \

%.hex: %.bin
	xxd -g2 -c2 -e $< | cut -d' ' -f2 > $@
