include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/Ethernet -I../../include/Ethernet/utility -I../../include/SPI
CXXFLAGS=$(CFLAGS)

OBJS=Ethernet.o EthernetClient.o EthernetServer.o EthernetUdp.o \
Dhcp.o Dns.o utility/socket.o utility/w5100.o

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
	echo $@
%.o : %.c
	$(CC) $(CFLAGS) -c $<

Ethernet.a: $(OBJS)
	$(AR) ../../$@ $^

all: Ethernet.a
