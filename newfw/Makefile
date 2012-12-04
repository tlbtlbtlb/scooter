#!/usr/local/bin/gmake

LIBATMEL ?= ../../libatmel
TARG ?= SCOOTER8

ifeq ($(TARG),SCOOTER8)
  ARCH = atmega128
  UISP_PART = ATmega128

  CLOCK_SPEED = 14745600
  BAUD_RATE = 115200

  FUSE_HIGH = 0xcc
  FUSE_LOW = 0x3f
  FUSE_EXTENDED = 0xff
endif

ifeq ($(TARG),TACKLIND_W0)
  ARCH = atmega128
  UISP_PART = ATmega128

  CLOCK_SPEED = 14745600
  BAUD_RATE = 115200

  FUSE_HIGH = 0xcc
  FUSE_LOW = 0x3f
  FUSE_EXTENDED = 0xff
endif

include $(LIBATMEL)/makedefs.inc

all :: $(ODIR)/scooter.srec $(ODIR)/scooter.dis $(ODIR)/scooter.sout $(ODIR)/uart.sout

.PRECIOUS: $(ODIR)/scooter.srec $(ODIR)/scooter.elf

dump-balconf:
	@gcc $(CFLAGS) -E balconf.h

install: all scooter.install

all :: $(ODIR)/scooter.srec $(ODIR)/scooter.dis
scooter.objs = $(ODIR)/scooter.o $(ODIR)/uart.o $(ODIR)/dsp_32.o
$(ODIR)/scooter.elf: $(scooter.objs)


all :: $(ODIR)/t_dsp.srec $(ODIR)/t_dsp.dis
t_dsp.objs = $(ODIR)/t_dsp.o $(ODIR)/dsp_32.o $(ODIR)/uart.o
$(ODIR)/t_dsp.elf: $(t_dsp.objs)


include $(LIBATMEL)/makerules.inc
