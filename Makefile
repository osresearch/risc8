
all: top.bin

TEST-y += test-avr
top.bin: firmware.hex

include Makefile.icestorm

CROSS ?= avr-
CFLAGS ?= \
	-O2 \
	-Wall \
	-mmcu=attiny85 \
	-ffreestanding \
	-nostdinc \
	-nostdlib \

test1.bin: test1.elf
	$(CROSS)objcopy -Obinary $< $@
%.elf: %.o
	$(CROSS)gcc $(CFLAGS) -o $@ $^ -Wl,-T,sections.lds
%.o: %.c
	$(CROSS)gcc $(CFLAGS) -c -o $@ $^

%.hex: %.bin
	xxd -g2 -c2 -e $< | cut -d' ' -f2 > $@
