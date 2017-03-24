<config.mk

OBJS = \
abi.o \
CDC.o \
HardwareSerial.o \
HardwareSerial0.o \
HardwareSerial1.o \
HardwareSerial2.o \
HardwareSerial3.o \
hooks.o \
IPAddress.o \
main.o \
new.o \
PluggableUSB.o \
Print.o \
Stream.o \
Tone.o \
USBCore.o \
WInterrupts.o \
wiring_analog.o \
wiring.o \
wiring_digital.o \
wiring_pulse.o \
wiring_pulse_asm.o \
wiring_shift.o \
WMath.o \
WString.o \
program.o \

all:V: applet.bin

%.o : %.S
	$CC -x assembler-with-cpp -c $stem.S
%.o : %.cpp
	$CXX -c $stem.cpp
%.o : %.c
	$CC -c $stem.c

applet.o : $OBJS
	$CXX $prereq -o $target

applet.bin : applet.o
	$OBJCOPY $prereq $target

flash:V: applet.bin
	avrdude -p m328p -c arduino -P $SERIAL -U flash:w:$target

clean:V:
	rm -f *.o

nuke:V: clean
	rm -f applet.bin
 