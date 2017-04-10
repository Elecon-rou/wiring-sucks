<config.mk

INC=-Iinclude/Core

all:V: applet.bin

%.o : %.S
	$CC $INC -x assembler-with-cpp -c $stem.S
%.o : %.cpp
	$CXX $INC -c $stem.cpp
%.o : %.c
	$CC $INC -c $stem.c
%.a :
	cd lib/$stem
	mk -f mklib
	cd ../..

applet.o : main.o program.o $LIBS
	$CXX $INC $prereq -o $target

applet.bin : applet.o
	$OBJCOPY $prereq $target

flash:V: applet.bin
	avrdude -p m328p -c arduino -P $SERIAL -U flash:w:$target

clean:V:
	rm -f *.o

nuke:V: clean
	rm -f applet.bin *.a lib/*/*.o
 