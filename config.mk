MCU=atmega328p

SERIAL=/dev/ttyACM0

LIBS=Core.a
#Core.a Bridge.a EEPROM.a Esplora.a Ethernet.a Firmata.a GSM.a HID.a Keyboard.a
#LiquidCrystal.a Mouse.a RobotIRremote.a Robot_Control.a Robot_Motor.a 
#SD.a SPI.a Servo.a SofwareSerial.a SpacebrewYun.a Stepper.a TFT.a
#Temboo.a WiFi.a Wire.a

CFLAGS=-Os -mmcu=$MCU -DF_CPU=16000000L 

CC=avr-gcc $CFLAGS
CXX=avr-g++ $CFLAGS
AR=avr-ar rcs
OBJCOPY=avr-objcopy -j .text -j .data -O ihex