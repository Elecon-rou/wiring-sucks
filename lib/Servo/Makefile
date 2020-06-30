include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Servo
CXXFLAGS=$(CFLAGS)

OBJS=Servo.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

Servo.a: $(OBJS)
	$(AR) ../../$@ $^

all: Servo.a

clean:
	rm -f *.o
