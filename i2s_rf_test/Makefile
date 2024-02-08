SUBMODULE=YES
SRCPREFIX=../nosdk8266/
MAIN_MHZ=173
SRCS=main.c

COMPILECOUNT=$(shell cat compilecount.txt || true)
BUILDCOUNTCMD=echo 1+0$(COMPILECOUNT) | bc > compilecount.txt | true Compiled $(COMPILECOUNT) times

DEPS=chirpbuff.h

include ../nosdk8266/Makefile

rf_data_gen : rf_data_gen.c
	gcc -o $@ $^ -lm

chirpbuff.dat : rf_data_gen
	./rf_data_gen

chirpbuff.h : chirpbuff.dat

chirpbuff.dat.flash : chirpbuff.dat
	$(ESPUTIL) flash 0x20000 chirpbuff.dat

localclean : clean
	rm -rf chirpbuff.h rf_data_gen chirpbuff.dat
