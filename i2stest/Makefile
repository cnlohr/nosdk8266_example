SUBMODULE=YES
SRCPREFIX=../nosdk8266/
MAIN_MHZ=173
SRCS=main.c

COMPILECOUNT=$(shell cat compilecount.txt || true)
BUILDCOUNTCMD=echo 1+0$(COMPILECOUNT) | bc > compilecount.txt | true Compiled $(COMPILECOUNT) times
include ../nosdk8266/Makefile

