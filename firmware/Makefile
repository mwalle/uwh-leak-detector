TARGET = leak-detector
MCU = attiny412
PMCU = t412
CONFIG_ENABLE_UART = y
CONFIG_ENABLE_DEMO = y
CONFIG_ENABLE_BMP581 = y
CONFIG_ENABLE_LPS22HB = y

SOURCES-y = main.c twi.c
SOURCES-$(CONFIG_ENABLE_DEMO) += demo.c
SOURCES-$(CONFIG_ENABLE_UART) += uart.c
SOURCES-$(CONFIG_ENABLE_BMP581) += bmp581.c
SOURCES-$(CONFIG_ENABLE_LPS22HB) += lps22hb.c

PROGRAMMER = serialupdi
PORT = /dev/ttyUSB0
BAUD = 230400
F_CPU = 5000000

OBJECTS = $(SOURCES-y:.c=.o)
CFLAGS = -g -Os -Werror -Wno-error=unused-function -Wall -ffunction-sections -DF_CPU=$(F_CPU) $(EXTRA_CFLAGS)
LDFLAGS = -Wl,--gc-sections

all: hex eeprom $(TARGET)_fuses.hex

hex: $(TARGET).hex
eeprom: $(TARGET)_eeprom.hex

config.h: Makefile
	@echo Generating config.h
	@if [ "$(CONFIG_ENABLE_UART)" = "y" ]; then \
		echo "#define CONFIG_ENABLE_UART 1" > config.h; \
	else \
		echo "#define CONFIG_ENABLE_UART 0" > config.h; \
	fi
	@if [ "$(CONFIG_ENABLE_DEMO)" = "y" ]; then \
		echo "#define CONFIG_ENABLE_DEMO 1" >> config.h; \
	else \
		echo "#define CONFIG_ENABLE_DEMO 0" >> config.h; \
	fi
	@if [ "$(CONFIG_ENABLE_BMP581)" = "y" ]; then \
		echo "#define CONFIG_ENABLE_BMP581 1" >> config.h; \
	else \
		echo "#define CONFIG_ENABLE_BMP581 0" >> config.h; \
	fi
	@if [ "$(CONFIG_ENABLE_LPS22HB)" = "y" ]; then \
		echo "#define CONFIG_ENABLE_LPS22HB 1" >> config.h; \
	else \
		echo "#define CONFIG_ENABLE_LPS22HB 0" >> config.h; \
	fi

$(TARGET).hex: $(TARGET).elf
	avr-objcopy -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex

$(TARGET)_fuses.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .fuse --change-section-lma .fuse=0 $(TARGET).elf $(TARGET)_fuses.hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --change-section-lma .eeprom=1 $(TARGET).elf $(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	avr-gcc $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(TARGET).elf

%.o: %.c config.h
	avr-gcc $(CFLAGS) -std=c99 -mmcu=$(MCU) -c $< -o $@

size: $(TARGET).elf
	avr-size --mcu=$(MCU) -C $(TARGET).elf

program: $(TARGET).hex
	avrdude -p$(PMCU) -P$(PORT) -B$(BAUD) -c$(PROGRAMMER) -Uflash:w:$(TARGET).hex:a

#fuse:
fuse: $(TARGET)_fuses.hex
	avrdude -p$(PMCU) -P$(PORT) -B$(BAUD) -c$(PROGRAMMER) -Ufuses:w:$(TARGET)_fuses.hex:a

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
	rm -f config.h
