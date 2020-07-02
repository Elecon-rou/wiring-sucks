include ../../config.mk
include ../../board.mk

CFLAGS+=-I../../include/Core -I../../include/GSM
CXXFLAGS=$(CFLAGS)

OBJS=GSM3CircularBuffer.o GSM3MobileAccessProvider.o GSM3MobileCellManagement.o \
GSM3MobileClientProvider.o GSM3MobileClientService.cpp GSM3MobileDataNetworkProvider.o \
GSM3MobileNetworkRegistry.o GSM3MobileServerProvider.o GSM3MobileServerService.o \
GSM3MobileVoiceProvider.o GSM3ShieldV1.o GSM3ShieldV1AccessProvider.o \
GSM3ShieldV1BandManagement.o GSM3ShieldV1BaseProvider.o GSM3ShieldV1CellManagement.o \
GSM3ShieldV1ClientProvider.o GSM3ShieldV1DataNetworkProvider.o GSM3ShieldV1DirectModemProvider.o \
GSM3ShieldV1ModemCore.o GSM3ShieldV1ModemVerification.o GSM3ShieldV1MultiClientProvider.o \
GSM3ShieldV1MultiServerProvider.o GSM3ShieldV1PinManagement.o GSM3ShieldV1ScanNetworks.o \
GSM3ShieldV1ServerProvider.o GSM3ShieldV1SMSProvider.o GSM3ShieldV1VoiceProvider.o \
GSM3SMSService.o GSM3SoftSerial.o GSM3VoiceCallService.o 

%.o : %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $<
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<
%.o : %.c
	$(CC) $(CFLAGS) -c $<

GSM.a: $(OBJS)
	$(AR) ../../$@ $^

all: GSM.a
