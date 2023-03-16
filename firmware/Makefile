TARGET = leak-detector
MCU = attiny45
PMCU = t45
SOURCES = main.c twi.c swuart.c bmp581.c

PROGRAMMER = buspirate
PORT = /dev/ttyUSB0
BAUD = 115200
F_CPU = 1000000

OBJECTS = $(SOURCES:.c=.o)
CFLAGS = -g -Os -Wall -ffunction-sections -DF_CPU=$(F_CPU) $(EXTRA_CFLAGS)
LDFLAGS = -Wl,--gc-sections

all: hex eeprom

hex: $(TARGET).hex

eeprom: $(TARGET)_eeprom.hex

$(TARGET).hex: $(TARGET).elf
	avr-objcopy -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --change-section-lma .eeprom=1 $(TARGET).elf $(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	avr-gcc $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(TARGET).elf

%.o: %.c
	avr-gcc $(CFLAGS) -std=c99 -mmcu=$(MCU) -c $< -o $@

size: $(TARGET).elf
	avr-size --mcu=$(MCU) -C $(TARGET).elf

program: $(TARGET).hex
	avrdude -p$(PMCU) -P$(PORT) -B$(BAUD) -c$(PROGRAMMER) -Uflash:w:$(TARGET).hex:a

fuse:
	avrdude -p$(PMCU) -P$(PORT) -B$(BAUD) -c$(PROGRAMMER) -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

clean:
	rm -rf *.o
	rm -rf *.elf
	rm -rf *.hex
