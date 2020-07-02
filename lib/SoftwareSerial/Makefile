include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/SoftwareSerial
CXXFLAGS=$(CFLAGS)

OBJS=SoftwareSerial.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

SoftwareSerial.a: $(OBJS)
	$(AR) ../../$@ $^

all: SoftwareSerial.a
