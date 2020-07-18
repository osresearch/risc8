
all: top.bin

TEST-y += test-avr
top.bin: firmware.hex

include Makefile.icestorm

#CROSS ?= avr-
CROSS ?= /home/hudson/bin/arduino-1.8.10/hardware/tools/avr/bin/avr-

CFLAGS ?= \
	-O2 \
	-Wall \
	-mmcu=attiny85 \

NO=\
	-ffreestanding \
	-nostdinc \
	-nostdlib \

%.bin: %.elf
	$(CROSS)objcopy -Obinary $< $@
%.elf: %.o
	$(CROSS)gcc $(CFLAGS) -o $@ $^ #-Wl,-T,sections.lds
%.o: %.c
	$(CROSS)gcc $(CFLAGS) -c -o $@ $^

%.hex: %.bin
	xxd -g2 -c2 -e $< | cut -d' ' -f2 > $@
