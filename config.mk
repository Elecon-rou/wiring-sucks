MCU=atmega328p

SERIAL= /dev/ttyACM0

CFLAGS= -Os -mmcu=$(MCU) -I. 

CC=avr-gcc $(CFLAGS)
CXX=avr-g++ $(CFLAGS)
OBJCOPY=avr-objcopy -j .text -j .data -O ihex