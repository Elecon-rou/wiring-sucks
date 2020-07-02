SERIAL=/dev/ttyACM0

LIBS=Core.a Ethernet.a Firmata.a GSM.a LiquidCrystal.a \
SD.a Servo.a SoftwareSerial.a SPI.a Stepper.a Wire.a

CC=avr-gcc $(CFLAGS)
CXX=avr-g++ $(CXXFLAGS)
AR=avr-ar rcs
OBJCOPY=avr-objcopy -j .text -j .data -O ihex
PROG=avrdude $(PROGFLAGS) -P $(SERIAL)
