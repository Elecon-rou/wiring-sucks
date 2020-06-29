SERIAL=/dev/ttyACM0

LIBS=Core.a SoftwareSerial.a

CC=avr-gcc $(CFLAGS)
CXX=avr-g++ $(CXXFLAGS)
AR=avr-ar rcs
OBJCOPY=avr-objcopy -j .text -j .data -O ihex
PROG=avrdude $(PROGFLAGS) -P $SERIAL
