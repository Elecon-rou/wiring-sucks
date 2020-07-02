include config.mk
include board.mk

.PHONY: all clean flash

CFLAGS+=-Iinclude/Core
CXXFLAGS=$(CFLAGS)

all: hex.bin

%.o: %.S
	$(CC) -x assembler-with-cpp -c $<

%.o: %.cpp
	$(CXX) -c $<

%.o: %.c
	$(CC) -c $<

%.a: lib/%
	$(MAKE) -C $<

hex.o: main.o program.o $(LIBS)
	$(CXX) $(CXXFLAGS) -o $@ $^

hex.bin: hex.o
	$(OBJCOPY) $^ hex.bin

flash: hex.bin
	$(PROG) -U flash:w:$<

clean:
	rm -f *.a *.o hex.bin
	rm -f lib/Core/*.o \
	      lib/Ethernet/*.o \
	      lib/Firmata/*.o \
	      lib/GSM/*.o \
	      lib/LiquidCrystal/*.o \
	      lib/SD/*.o \
	      lib/Servo/*.o \
	      lib/SoftwareSerial/*.o \
	      lib/SPI/*.o \
	      lib/Stepper/*.o \
	      lib/Wire/*.o