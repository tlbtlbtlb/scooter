#!/usr/bin/make
# $Id: Makefile 86 2005-02-07 02:36:29Z tlb $
#
# Rev2 control board and IMU makefile
#

ARCH		= atmega32

atmega163_base	= 0x3C00
atmega163_clock	= 8

atmega16_base	= 0x3C00
atmega16_clock	= 16

atmega32_base	= 0x7C00
atmega32_clock	= 16

CC		= avr-gcc -mmcu=$(ARCH)
LD		= $(CC)
SIZE		= avr-size

UISP		= uisp -v

UISP_FLAGS	=							\
	-dprog=stk500							\
	-dpart=ATmega32							\
	-dserial=/dev/cuaa0						\
	-dspeed=115200							\


CFLAGS		=							\
	-O4								\
	-W								\
	-Wall								\
	-DBOOT_BASE=$($(ARCH)_base)					\
	-DCLOCK_SPEED=$($(ARCH)_clock)					\


LDFLAGS		=							\
	-lm								\

all: scooter.hex

install: scooter.install


scooter.srcs	=							\
	scooter.c							\
	uart.c								\
	string.c							\
	adc.c								\

scooter.hdrs = debug.h timer.h string.h adc.h uart.h

scooter.objs	=							\
	$(scooter.srcs:.c=.o)						\

scooter.elf: $(scooter.objs)


%.elf:
	ln -sf /usr/local/avr/lib/avr5/crtm32.o .
	$(LD)								\
		$($(@:.elf=).objs)					\
		-o $@							\
		$(LDFLAGS)						\

	$(SIZE) $@


%.srec: %.elf
	avr-objcopy -O srec $< $@

%.hex: %.elf
	avr-objcopy -O ihex $< $@

%.install: %.srec
	# stk200 needs to be erased first
	$(UISP) $(UISP_FLAGS) --erase
	$(UISP) $(UISP_FLAGS) --upload if=$<

clean:
	rm -f *.srec *.elf a.out core *.o *.a *.hex

depend:
	makedepend $(scooter.srcs)

distname=scooter-0.1a

dist.srcs = $(scooter.srcs) $(scooter.hdrs) Makefile README

$(distname).tgz: $(dist.srcs)
	rm -rf $(distname)
	mkdir $(distname)
	cp $(dist.srcs) $(distname)
	tar czf $@ $(distname)
	rm -rf $(distname)

# DO NOT DELETE

scooter.o: debug.h /usr/include/math.h /usr/include/sys/cdefs.h timer.h
scooter.o: uart.h /usr/include/inttypes.h /usr/include/machine/ansi.h
scooter.o: string.h adc.h
uart.o: debug.h uart.h /usr/include/inttypes.h /usr/include/machine/ansi.h
uart.o: timer.h
string.o: /usr/include/stdlib.h /usr/include/sys/cdefs.h
string.o: /usr/include/machine/ansi.h string.h uart.h /usr/include/inttypes.h
adc.o: adc.h uart.h /usr/include/inttypes.h /usr/include/machine/ansi.h
adc.o: string.h
