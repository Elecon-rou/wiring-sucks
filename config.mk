MCU= atmega128 

SERIAL= /dev/ttyUSB0

CFLAGS= -Os -mmcu=$(MCU) -I. -Iinclude

CC=avr-gcc $(CFLAGS) -fpermissive
CXX=avr-g++ $(CFLAGS) -fpermissive
LD=avr-gcc $(CFLAGS) -s -Wl,--gc-sections,-u,-Map=$(<:.o=.map),--cref
OBJCOPY= avr-objcopy -O srec -R .eeprom 