include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/SPI
CXXFLAGS=$(CFLAGS)

OBJS=SPI.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

SPI.a: $(OBJS)
	$(AR) ../../$@ $^

all: SPI.a

clean:
	rm -f *.o
