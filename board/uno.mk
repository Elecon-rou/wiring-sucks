CFLAGS=-Os -mmcu=atmega328p -DF_CPU=16000000L -I../../include/Core/pins_uno -Iinclude/Core/pins_uno
CXXFLAGS=$(CFLAGS)
PROGFLAGS=-p m328p -c arduino
