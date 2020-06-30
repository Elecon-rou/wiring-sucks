include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/Stepper
CXXFLAGS=$(CFLAGS)

OBJS=Stepper.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

Stepper.a: $(OBJS)
	$(AR) ../../$@ $^

all: Stepper.a

clean:
	rm -f *.o