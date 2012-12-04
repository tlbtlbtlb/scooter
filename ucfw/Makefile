#!/usr/local/bin/gmake

HOSTNAME := $(shell uname -n | tr -cd .a-zA-Z0-9)

TARG=BDMICRO

ifeq ($(TARG),STK_PROTO)
ARCH = atmega16
UISP_PART = ATmega16
CLOCK_SPEED = 8000000
BAUD_RATE=115200
FUSE_HIGH=0xd9
FUSE_LOW=0xe0
endif

ifeq ($(TARG),ROTOMOTION)
ARCH = atmega32
UISP_PART = ATmega32
CLOCK_SPEED = 16000000
BAUD_RATE=115200
FUSE_HIGH=0xd9 # this disables its bootloader
FUSE_LOW=0xef
endif

ifeq ($(TARG),BDMICRO)
# The BD Micro MAVRIC board
ARCH = atmega128
UISP_PART = ATmega128
CLOCK_SPEED = 16000000
BAUD_RATE=115200

USE_BOOTLOADER=yes
ifeq ($(HOSTNAME),spiro.anybots.com)
BOOTLOADHOST:=$(HOSTNAME)
BOOTLOADPORT:=/dev/cuaa0
else
BOOTLOADHOST:=lab
BOOTLOADPORT:=/dev/cuac01
endif

ifneq ($(USE_BOOTLOADER),'')
FUSE_HIGH=0x88  # same as BD ships
else
FUSE_HIGH=0x89 
endif
FUSE_LOW=0x3f   # sams as BD ships
endif

INCLUDES = -I../../robot/libatmel
VPATH = ../../robot/libatmel

include ../../robot/libatmel/makedefs.inc

LDFLAGS	= -lm

all: $(ODIR)/unicycle.srec $(ODIR)/unicycle.dis $(ODIR)/bootloader.srec

.PRECIOUS: $(ODIR)/unicycle.srec $(ODIR)/unicycle.elf

debug:
	echo $(HOSTNAME) $(BOOTLOADHOST)

install: all unicycle.bootdownload

unicycle.objs = $(ODIR)/scooter.o $(ODIR)/adc.o $(ODIR)/uart.o
bootloader.objs = $(ODIR)/bootloader.o

$(ODIR)/unicycle.elf: $(unicycle.objs)
$(ODIR)/bootloader.elf: $(bootloader.objs)

include ../../robot/libatmel/makerules.inc
