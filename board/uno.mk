CFLAGS=-Os -mmcu=atmega328p -DF_CPU=16000000L -Iinclude/Core
CXXFLAGS=$(CFLAGS)
PROGFLAGS=-p m328p -c arduino
