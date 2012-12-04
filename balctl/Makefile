
ifeq (RELEASE, '')
LIBATMEL:=libatmel
LIBTLB:=libtlb
else
LIBATMEL:=$(HOME)/libatmel
LIBTLB:=$(HOME)/libtlb
endif

include $(LIBTLB)/stdmake.inc0

INCLUDES += -I/usr/local/include -I. -I$(HOME)

PYTHON := /usr/local/bin/python
INCLUDES += -I/usr/local/include/python2.4

PYTHON_PLATFORM:=$(shell $(PYTHON) -c 'import sys; print sys.platform')
PYODIR=build.$(PYTHON_PLATFORM)

SLIBEXT=so
PYMODEXT=so

CFLAGS += -fPIC -ftemplate-depth-50 $(PYTHON_INCLUDES) -Wno-non-virtual-dtor -ggdb -O0

default : $(PYODIR)/boost_packet.so

$(ODIR)/packet_conn.o : $(LIBATMEL)/packet_conn.cc
	$(CXX) -c -I. $(CFLAGS) $(CFLAGS_$*) -o $@ $<

$(ODIR)/bootloader_conn.o : $(LIBATMEL)/bootloader_conn.cc
	$(CXX) -c -I. $(CFLAGS) $(CFLAGS_$*) -o $@ $<

$(PYODIR)/boost_packet.so : $(ODIR)/boost_packet.o \
	$(ODIR)/packet_conn.o $(ODIR)/bootloader_conn.o
	@mkdir -p $(PYODIR) 
	$(LDPYMOD) -o $@ $+ -L/usr/local/lib -lboost_python $(TLB_LIBS)

VERSION=0.1a

balctl-$(VERSION) : force
	rm -rf $@
	-mkdir $@
	-mkdir $@/libatmel
	-mkdir $@/libtlb
	cp LICENSE $@
	for x in balctl.py boost_packet.cc process_log.py; do \
	    add_copyright.py COPYRIGHT $$x $@/$$x; \
	done
	(echo RELEASE=$(VERSION); cat Makefile) > $@/Makefile
	for x in \
	         $(LIBATMEL)/makerules.inc $(LIBATMEL)/makedefs.inc \
	         $(LIBATMEL)/bootloader_conn.h \
	         $(LIBATMEL)/bootloader_conn.cc \
	         $(LIBATMEL)/packet_conn.h \
	         $(LIBATMEL)/packet_conn.cc \
	         $(LIBATMEL)/crc32.c ; \
	    do \
	    add_copyright.py COPYRIGHT $$x $@/`basename $$x`; \
	done
	for x in \
	         $(LIBTLB)/stdmake.inc0 $(LIBTLB)/stdmake.inc1 \
	         $(LIBTLB)/tty_serial.cc \
	         $(LIBTLB)/debugentity.h \
	         $(LIBTLB)/abstract_serial.h ; \
	    do \
	    add_copyright.py COPYRIGHT $$x $@/libtlb/`basename $$x`; \
	done
	cd $@ && gmake && gmake clean

balctl-$(VERSION).tgz: balctl-$(VERSION)
	tar czf $@ $+

dist: balctl-$(VERSION).tgz
	cp $+ ~/website/tlb

include $(HOME)/libtlb/stdmake.inc1
