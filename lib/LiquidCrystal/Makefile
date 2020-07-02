include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/LiquidCrystal
CXXFLAGS=$(CFLAGS)

OBJS=LiquidCrystal.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

LiquidCrystal.a: $(OBJS)
	$(AR) ../../$@ $^

all: LiquidCrystal.a
