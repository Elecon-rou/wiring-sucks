include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/Wire -I../../include/Wire/utility
CXXFLAGS=$(CFLAGS)

OBJS=Wire.o utility/twi.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

Wire.a: $(OBJS)
	$(AR) ../../$@ $^

all: Wire.a

clean:
	rm -f *.o
