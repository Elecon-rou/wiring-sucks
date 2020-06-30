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
	rm *.a *.o hex.bin
