include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core
CXXFLAGS=$(CFLAGS)

OBJS=abi.o CDC.o HardwareSerial.o HardwareSerial0.o \
HardwareSerial1.o HardwareSerial2.o HardwareSerial3.o hooks.o \
IPAddress.o new.o PluggableUSB.o Print.o Stream.o Tone.o USBCore.o \
WInterrupts.o wiring_analog.o wiring.o wiring_digital.o wiring_pulse.o \
wiring_pulse_asm.o wiring_shift.o WMath.o WString.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

Core.a: $(OBJS)
	$(AR) ../../$@ $^

all: Core.a
