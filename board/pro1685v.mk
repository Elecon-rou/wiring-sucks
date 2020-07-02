CFLAGS=-Os -mmcu=atmega168 -DF_CPU=16000000L -I../../include/Core/pins_uno -Iinclude/Core/pins_uno
CXXFLAGS=$(CFLAGS)
PROGFLAGS=-p m168 -c arduino -b19200