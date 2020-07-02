CFLAGS=-Os -mmcu=atmega2560 -DF_CPU=16000000L -I../../include/Core/pins_mega -Iinclude/Core/pins_mega
CXXFLAGS=$(CFLAGS)
PROGFLAGS=-p m2560 -c wiring -D 