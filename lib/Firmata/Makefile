include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/Firmata
CXXFLAGS=$(CFLAGS)

OBJS=Firmata.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

Firmata.a: $(OBJS)
	$(AR) ../../$@ $^

all: Firmata.a
