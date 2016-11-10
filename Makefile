include config.mk

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
wiring_shift.o \
WMath.o \
WString.o \
program.o \

%.o : %.S
	$(CC) -x assembler-with-cpp -c -o $@ $<
%.o : %.cpp
	$(CXX) -c -o $@ $<
%.o : %.c
	$(CC) -c -o $@ $<

applet: $(OBJS)
	$(CXX) $^ -o $@.o
	$(OBJCOPY) $@.o $@.bin

flash: 
	avrdude -p m328p -c arduino -P $(SERIAL) -U flash:w:applet.bin 

clean:
	rm -f applet.bin *.o
