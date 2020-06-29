# wiring-sucks
Build Wiring programs for Arduino without any IDE

## How to use
* Copy your device config from ```./board/``` to ```board.mk```. Default is Arduino Uno R3
* Write your program in ```program.cpp```
* Edit ```config.mk``` to specify port and other settings
* ```$ make``` to build
* ```# make flash``` to flash your Arduino

## Requirements
AVR Toolchain: avr-gcc, avr-g++, avr-libc, avrdude
