include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/SD
CXXFLAGS=$(CFLAGS)

OBJS=File.o SD.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

SD.a: $(OBJS)
	$(AR) ../../$@ $^

all: SD.a
