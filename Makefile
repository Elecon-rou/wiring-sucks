include config.mk

OBJS = main.o program.o\
WApplet.o WTimer.o WInterrupts.o \
WMath.o WMemory.o WString.o WPulse.o \
WShift.o Print.o Tone.o

%.o : %.cpp
	$(CXX) -c -o $@ $<
%.o : %.c
	$(CC) -c -o $@ $<


WApplet: $(OBJS)
	$(LD) -o $@ $^
	$(OBJCOPY) $@ WApplet.bin

flash:
	uisp -dprog=stk500 -v=2 -dserial=$(SERIAL) -dpart=$(MCU) if=WApplet.bin --upload  

clean:
	rm -f WApplet WApplet.bin *.o  