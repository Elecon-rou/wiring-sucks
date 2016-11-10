MCU=atmega328p

SERIAL= /dev/ttyACM0

CFLAGS= -Os -mmcu=$(MCU) -I. -Iinclude -DF_CPU=16000000L 

CC=avr-gcc $(CFLAGS)
CXX=avr-g++ $(CFLAGS)
OBJCOPY=avr-objcopy -j .text -j .data -O ihex